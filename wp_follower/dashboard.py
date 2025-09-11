import json
import os
import threading
import time
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from flask import Flask, jsonify, request, Response, render_template

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Support running as a script or as a package module
try:
    from .wp_follower import gps_point_to_local, gps_points_to_local, build_goal_pose
except ImportError:  # pragma: no cover - fallback for direct script runs
    from wp_follower import gps_point_to_local, gps_points_to_local, build_goal_pose


DEFAULT_JSON_PATH = (Path(__file__).parent / 'demo_wp.json')


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
                local = gps_point_to_local(self._helper, wp)
                pose = build_goal_pose(
                    self._navigator,
                    local['x'], local['y'], local['yaw_rad']
                )
                self._navigator.goToPose(pose)
                self._last_goal_id = wp_id
                self._status = 'active'
                self._last_error = None
                self._mode = 'single'
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

    def cancel(self) -> Dict:
        with self._lock:
            try:
                self._navigator.cancelTask()
                # Mark as canceling; final status will settle on next poll
                self._status = 'canceled'
                return {'ok': True, 'status': self._status}
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
                    fb = self._navigator.getFeedback()
                except Exception:
                    fb = None

                if not self._navigator.isTaskComplete():
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

    return app


if __name__ == '__main__':
    # Run Flask app without reloader to avoid duplicate rclpy init
    app = create_app()
    app.run(host='0.0.0.0', port=int(os.environ.get('PORT', '5000')), debug=False)
