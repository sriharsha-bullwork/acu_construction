import json
import os
import threading
import time
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from flask import Flask, jsonify, request, Response, render_template, make_response

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import NavSatFix

# Support running as a script or as a package module
try:
    from .wp_follower import gps_point_to_local, gps_points_to_local, build_goal_pose
except ImportError:  # pragma: no cover - fallback for direct script runs
    from wp_follower import gps_point_to_local, gps_points_to_local, build_goal_pose


DEFAULT_JSON_PATH = (Path(__file__).parent / 'demo_wp.json')

# Serialize rclpy spinning across threads to avoid re-entrancy errors
ROS_SPIN_LOCK = threading.Lock()


class NavBridge:
    """Holds ROS2 resources and exposes simple Nav2 goal operations.

    - Keeps a helper Node to call /fromLL
    - Keeps a BasicNavigator instance to send goals
    - Tracks last goal and status for polling
    """

    def __init__(self, waypoints: List[Dict], routes: List[Dict]):
        if not rclpy.ok():
            rclpy.init()

        self._helper = Node('wp_dashboard_helper')
        self._navigator = BasicNavigator()
        self._waypoints = waypoints
        self._routes = routes  # list of {id, waypoint_ids: [..]}

        self._lock = threading.Lock()
        self._last_goal_id: Optional[str] = None
        self._status: str = 'idle'  # idle|active|succeeded|failed|canceled|error
        self._last_error: Optional[str] = None
        self._mode: str = 'none'  # none|single|route
        self._route_id: Optional[str] = None
        self._route_total: int = 0
        self._route_current: Optional[int] = None
        # Pause/Resume support
        self._paused: Optional[Dict] = None  # {'mode': 'single'|'route', ...}
        # Mission flag
        self._mission_active: bool = False

    def list_waypoints(self) -> List[Dict]:
        # Return presentation-friendly fields
        return [
            {
                'id': w.get('id'),
                'name': w.get('name'),
                'lat': w.get('lat'),
                'lon': w.get('lon'),
                'yaw_deg': w.get('yaw_deg', 0.0),
            }
            for w in self._waypoints
        ]

    def _find_wp(self, wp_id: str) -> Optional[Dict]:
        for w in self._waypoints:
            if w.get('id') == wp_id:
                return w
        return None

    def list_routes(self) -> List[Dict]:
        return [
            {
                'id': r.get('id'),
                'waypoint_ids': list(r.get('waypoint_ids', [])),
                'size': len(r.get('waypoint_ids', [])),
            }
            for r in self._routes
        ]

    def send_goal(self, wp_id: str) -> Dict:
        """Convert and send a Nav2 goal for the waypoint id.
        Returns a short status dict immediately; use poll_status() to monitor.
        """
        with self._lock:
            wp = self._find_wp(wp_id)
            if not wp:
                return {'ok': False, 'error': f'waypoint not found: {wp_id}'}
            try:
                with ROS_SPIN_LOCK:
                    local = gps_point_to_local(self._helper, wp)
                    pose = build_goal_pose(
                        self._navigator,
                        local['x'], local['y'], local['yaw_rad']
                    )
                    self._navigator.goToPose(pose)
                self._last_goal_id = wp_id
                self._status = 'active'
                self._last_error = None
                # If mission is active, keep mode as 'mission' so UI stays enabled
                self._mode = 'mission' if getattr(self, '_mission_active', False) else 'single'
                self._route_id = None
                self._route_total = 0
                self._route_current = None
                return {'ok': True, 'status': self._status, 'goal_id': wp_id}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def start_route(self, route_id: str) -> Dict:
        with self._lock:
            route = next((r for r in self._routes if r.get('id') == route_id), None)
            if not route:
                return {'ok': False, 'error': f'route not found: {route_id}'}
            wp_ids = route.get('waypoint_ids', [])
            if not wp_ids:
                return {'ok': False, 'error': f'route has no waypoints: {route_id}'}

            try:
                poses = []
                with ROS_SPIN_LOCK:
                    for wid in wp_ids:
                        w = self._find_wp(wid)
                        if not w:
                            raise ValueError(f'waypoint id not found in route: {wid}')
                        local = gps_point_to_local(self._helper, w)
                        poses.append(
                            build_goal_pose(self._navigator, local['x'], local['y'], local['yaw_rad'])
                        )

                    self._navigator.followWaypoints(poses)
                self._last_goal_id = None
                self._status = 'active'
                self._last_error = None
                self._mode = 'route'
                self._route_id = route_id
                self._route_total = len(poses)
                self._route_current = 0
                return {'ok': True, 'status': self._status, 'route_id': route_id}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def start_mission(self) -> Dict:
        """Enable mission mode (manual waypoint selection only)."""
        with self._lock:
            if self._status == 'active':
                return {'ok': False, 'error': 'a task is already active'}
            if not self._waypoints:
                return {'ok': False, 'error': 'no waypoints loaded'}
            # Do not send any goals automatically; just enable mission mode
            self._mission_active = True
            self._last_goal_id = None
            self._last_error = None
            self._mode = 'mission'
            self._route_id = None
            self._route_total = 0
            self._route_current = None
            # Keep status idle; movement occurs when user clicks a waypoint
            return {'ok': True, 'status': self._status, 'mode': self._mode}

    def stop_mission(self) -> Dict:
        """Stop any running mission and reset dashboard to normal."""
        with self._lock:
            try:
                with ROS_SPIN_LOCK:
                    self._navigator.cancelTask()
                self._mission_active = False
                self._paused = None
                self._last_goal_id = None
                self._mode = 'none'
                self._route_id = None
                self._route_total = 0
                self._route_current = None
                self._status = 'idle'
                self._last_error = None
                return {'ok': True, 'status': self._status}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def export_data(self) -> Dict:
        """Return current waypoints and routes as a dict."""
        with self._lock:
            return {
                'waypoints': list(self._waypoints),
                'routes': list(self._routes),
            }

    def import_data(self, data: Dict) -> Dict:
        """Replace waypoints/routes with the provided JSON if valid.

        Only allowed when no active task is running.
        """
        with self._lock:
            if self._status == 'active':
                return {'ok': False, 'error': 'cannot import while a task is active'}
            if not isinstance(data, dict):
                return {'ok': False, 'error': 'invalid JSON'}
            wps = data.get('waypoints')
            if not isinstance(wps, list) or not wps:
                return {'ok': False, 'error': "JSON must contain a non-empty 'waypoints' list"}
            routes_raw = data.get('routes', [])
            routes: List[Dict] = []
            if isinstance(routes_raw, dict):
                for rid, arr in routes_raw.items():
                    if isinstance(arr, list):
                        routes.append({'id': rid, 'waypoint_ids': list(arr)})
            elif isinstance(routes_raw, list):
                for item in routes_raw:
                    if isinstance(item, dict) and 'id' in item:
                        ids = item.get('waypoints') or item.get('waypoint_ids') or []
                        if isinstance(ids, list):
                            routes.append({'id': item['id'], 'waypoint_ids': list(ids)})
            # Update
            self._waypoints = list(wps)
            self._routes = routes
            return {'ok': True}

    def cancel(self) -> Dict:
        with self._lock:
            try:
                with ROS_SPIN_LOCK:
                    self._navigator.cancelTask()
                # Mark as canceling; final status will settle on next poll
                self._status = 'canceled'
                return {'ok': True, 'status': self._status}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def pause(self) -> Dict:
        """Pause current task by canceling and storing resume state."""
        with self._lock:
            if self._status != 'active':
                return {'ok': False, 'error': 'no active task to pause'}
            try:
                fb = None
                try:
                    with ROS_SPIN_LOCK:
                        fb = self._navigator.getFeedback()
                except Exception:
                    fb = None

                # Capture progress info for routes
                if self._mode == 'route':
                    idx = None
                    if fb is not None and hasattr(fb, 'current_waypoint'):
                        try:
                            idx = int(fb.current_waypoint)
                        except Exception:
                            idx = None
                    if idx is None:
                        idx = self._route_current or 0
                    self._paused = {
                        'mode': 'route',
                        'route_id': self._route_id,
                        'index': max(0, int(idx)),
                    }
                else:  # single goal pause
                    self._paused = {
                        'mode': 'single',
                        'wp_id': self._last_goal_id,
                    }

                with ROS_SPIN_LOCK:
                    self._navigator.cancelTask()
                self._status = 'paused'
                return {'ok': True, 'status': self._status}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def resume(self) -> Dict:
        """Resume a previously paused task from stored state."""
        with self._lock:
            if not self._paused:
                return {'ok': False, 'error': 'no paused task to resume'}
            try:
                paused = self._paused
                self._paused = None
                if paused.get('mode') == 'single':
                    wp_id = paused.get('wp_id')
                    if not wp_id:
                        return {'ok': False, 'error': 'paused single goal missing id'}
                    # Re-send the same waypoint goal (robot continues from current pose)
                    wp = self._find_wp(wp_id)
                    if not wp:
                        return {'ok': False, 'error': f'waypoint not found: {wp_id}'}
                    with ROS_SPIN_LOCK:
                        local = gps_point_to_local(self._helper, wp)
                        pose = build_goal_pose(self._navigator, local['x'], local['y'], local['yaw_rad'])
                        self._navigator.goToPose(pose)
                    self._last_goal_id = wp_id
                    self._status = 'active'
                    self._last_error = None
                    self._mode = 'single'
                    self._route_id = None
                    self._route_total = 0
                    self._route_current = None
                    return {'ok': True, 'status': self._status, 'goal_id': wp_id}

                # Route resume
                route_id = paused.get('route_id')
                start_idx = int(paused.get('index', 0))
                route = next((r for r in self._routes if r.get('id') == route_id), None)
                if not route:
                    return {'ok': False, 'error': f'route not found: {route_id}'}
                wp_ids = route.get('waypoint_ids', [])
                if start_idx >= len(wp_ids):
                    # Nothing to resume; treat as success
                    self._status = 'succeeded'
                    self._mode = 'none'
                    self._route_id = None
                    self._route_total = 0
                    self._route_current = None
                    return {'ok': True, 'status': 'succeeded'}

                poses = []
                with ROS_SPIN_LOCK:
                    for wid in wp_ids[start_idx:]:
                        w = self._find_wp(wid)
                        if not w:
                            return {'ok': False, 'error': f'waypoint id not found in route: {wid}'}
                        local = gps_point_to_local(self._helper, w)
                        poses.append(build_goal_pose(self._navigator, local['x'], local['y'], local['yaw_rad']))

                    self._navigator.followWaypoints(poses)
                self._last_goal_id = None
                self._status = 'active'
                self._last_error = None
                self._mode = 'route'
                self._route_id = route_id
                # Track progress for the resumed segment
                self._route_total = len(poses)
                self._route_current = 0
                return {'ok': True, 'status': self._status, 'route_id': route_id}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def poll_status(self) -> Dict:
        """Check and return the current goal status without blocking."""
        with self._lock:
            if self._status not in ('active',):
                return {
                    'status': self._status,
                    'mode': self._mode,
                    'goal_id': self._last_goal_id,
                    'route_id': self._route_id,
                    'route_current': self._route_current,
                    'route_total': self._route_total,
                    'distance_remaining': None,
                    'eta_sec': None,
                    'error': self._last_error,
                }

            try:
                fb = None
                try:
                    with ROS_SPIN_LOCK:
                        fb = self._navigator.getFeedback()
                except Exception:
                    fb = None

                done = False
                with ROS_SPIN_LOCK:
                    done = self._navigator.isTaskComplete()
                if not done:
                    # Still active; include feedback if available
                    distance = getattr(fb, 'distance_remaining', None) if fb else None
                    eta = None
                    if fb and getattr(fb, 'estimated_time_remaining', None) is not None:
                        # Nav2 provides builtin_interfaces/Duration
                        dur = fb.estimated_time_remaining
                        eta = float(dur.sec) + float(dur.nanosec) / 1e9

                    # Route progress if available
                    if self._mode == 'route' and fb is not None and hasattr(fb, 'current_waypoint'):
                        self._route_current = int(fb.current_waypoint)

                    return {
                        'status': self._status,
                        'mode': self._mode,
                        'goal_id': self._last_goal_id,
                        'route_id': self._route_id,
                        'route_current': self._route_current,
                        'route_total': self._route_total,
                        'distance_remaining': distance,
                        'eta_sec': eta,
                        'error': None,
                    }

                # Completed; get final result
                with ROS_SPIN_LOCK:
                    result = self._navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self._status = 'succeeded'
                elif result == TaskResult.CANCELED:
                    self._status = 'canceled'
                elif result == TaskResult.FAILED:
                    code, msg = self._navigator.getTaskError()
                    self._status = 'failed'
                    self._last_error = f'{code}: {msg}'
                else:
                    self._status = 'error'
                    self._last_error = 'unknown result'

                return {
                    'status': self._status,
                    'mode': self._mode,
                    'goal_id': self._last_goal_id,
                    'route_id': self._route_id,
                    'route_current': self._route_current,
                    'route_total': self._route_total,
                    'distance_remaining': None,
                    'eta_sec': None,
                    'error': self._last_error,
                }
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {
                    'status': self._status,
                    'mode': self._mode,
                    'goal_id': self._last_goal_id,
                    'route_id': self._route_id,
                    'route_current': self._route_current,
                    'route_total': self._route_total,
                    'distance_remaining': None,
                    'eta_sec': None,
                    'error': self._last_error,
                }


def load_waypoints_and_routes(json_path: Path) -> Tuple[List[Dict], List[Dict]]:
    with open(json_path, 'r') as f:
        doc = json.load(f) or {}
    wps = doc.get('waypoints', [])
    if not isinstance(wps, list) or not wps:
        raise ValueError("JSON must contain a non-empty 'waypoints' list")
    # Normalize routes: support dict or list
    routes_raw = doc.get('routes', {})
    routes: List[Dict] = []
    if isinstance(routes_raw, dict):
        for rid, arr in routes_raw.items():
            if isinstance(arr, list):
                routes.append({'id': rid, 'waypoint_ids': list(arr)})
    elif isinstance(routes_raw, list):
        for item in routes_raw:
            if isinstance(item, dict) and 'id' in item:
                ids = item.get('waypoints') or item.get('waypoint_ids') or []
                if isinstance(ids, list):
                    routes.append({'id': item['id'], 'waypoint_ids': list(ids)})

    return wps, routes


def create_app() -> Flask:
    base_dir = Path(__file__).parent
    app = Flask(
        __name__,
        template_folder=str(base_dir / 'templates'),
        static_folder=str(base_dir / 'static'),
    )

    # Suppress verbose GET request logs in terminal
    logging.getLogger('werkzeug').setLevel(logging.ERROR)

    json_path = Path(os.environ.get('WP_JSON_PATH', str(DEFAULT_JSON_PATH)))
    waypoints, routes = load_waypoints_and_routes(json_path)
    nav = NavBridge(waypoints, routes)

    # On-demand telemetry sampling to avoid concurrent rclpy spinning
    _last_gps: Optional[Dict] = None
    _last_gps_ts: Optional[float] = None

    def _sample_gps_once(timeout_sec: float = 0.25) -> Optional[Dict]:
        nonlocal _last_gps, _last_gps_ts
        try:
            node = Node('wp_dashboard_telemetry_once')
            done = Future()
            data: Dict = {}

            def cb(msg: NavSatFix):
                data['gps'] = {
                    'lat': float(msg.latitude),
                    'lon': float(msg.longitude),
                    'alt': float(msg.altitude),
                    'status': getattr(getattr(msg, 'status', None), 'status', None),
                    'service': getattr(getattr(msg, 'status', None), 'service', None),
                }
                if not done.done():
                    done.set_result(True)

            node.create_subscription(NavSatFix, '/gps/fix', cb, 10)
            with ROS_SPIN_LOCK:
                rclpy.spin_until_future_complete(node, done, timeout_sec=timeout_sec)
            node.destroy_node()
            if done.done() and 'gps' in data:
                _last_gps = data['gps']
                _last_gps_ts = time.time()
                return _last_gps
            return _last_gps
        except Exception:
            return _last_gps

    @app.get('/')
    def index() -> Response:
        return render_template('index.html')

    @app.get('/api/waypoints')
    def api_waypoints():
        return jsonify(nav.list_waypoints())

    @app.post('/api/goal/<wp_id>')
    def api_goal(wp_id: str):
        return jsonify(nav.send_goal(wp_id))

    @app.get('/api/status')
    def api_status():
        return jsonify(nav.poll_status())

    @app.get('/api/routes')
    def api_routes():
        return jsonify(nav.list_routes())

    @app.post('/api/route/<route_id>')
    def api_start_route(route_id: str):
        return jsonify(nav.start_route(route_id))

    @app.post('/api/cancel')
    def api_cancel():
        return jsonify(nav.cancel())

    @app.post('/api/pause')
    def api_pause():
        return jsonify(nav.pause())

    @app.post('/api/resume')
    def api_resume():
        return jsonify(nav.resume())

    @app.get('/api/telemetry')
    def api_telemetry():
        gps = _sample_gps_once()
        return jsonify({'gps': gps})

    # Mission endpoints
    @app.post('/api/mission/start')
    def api_mission_start():
        return jsonify(nav.start_mission())

    @app.post('/api/mission/stop')
    def api_mission_stop():
        return jsonify(nav.stop_mission())

    # Export/Import endpoints
    @app.get('/api/export')
    def api_export():
        data = nav.export_data()
        resp = make_response(json.dumps(data, indent=2))
        resp.headers['Content-Type'] = 'application/json'
        resp.headers['Content-Disposition'] = 'attachment; filename=waypoints.json'
        return resp

    @app.post('/api/import')
    def api_import():
        try:
            payload = request.get_json(silent=True)
            if payload is None and request.data:
                payload = json.loads(request.data.decode('utf-8'))
        except Exception:
            payload = None
        if payload is None:
            return jsonify({'ok': False, 'error': 'invalid JSON'}), 400
        return jsonify(nav.import_data(payload))

    return app


if __name__ == '__main__':
    # Run Flask app without reloader to avoid duplicate rclpy init
    app = create_app()
    app.run(host='0.0.0.0', port=int(os.environ.get('PORT', '5000')), debug=False)
