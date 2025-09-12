import json
import os
import threading
import time
import logging
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from flask import Flask, jsonify, request, Response, render_template, make_response

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Twist

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
        # Joystick/cmd_vel publisher
        self._cmd_pub = self._helper.create_publisher(Twist, '/cmd_vel', 10)
        self._max_lin = float(os.environ.get('JOY_MAX_LIN', '0.5'))
        self._max_ang = float(os.environ.get('JOY_MAX_ANG', '1.0'))
        # GPS routes: map (from_id, to_id) -> list of {lat, lon, yaw_deg}
        self._gps_routes: Dict[Tuple[str, str], List[Dict]] = {}
        # Track active GPS route pair if following one
        self._active_gps_pair: Optional[Tuple[str, str]] = None

    # --- Waypoint management helpers ---
    def _gen_new_wp_id(self) -> str:
        base = 'wp'
        max_n = 0
        for w in self._waypoints:
            wid = str(w.get('id', ''))
            if wid.startswith(base + '_'):
                try:
                    n = int(wid.split('_')[-1])
                    if n > max_n:
                        max_n = n
                except Exception:
                    continue
        return f'{base}_{max_n + 1}'

    def add_waypoint(self, lat: float, lon: float, yaw_deg: float, name: Optional[str] = None) -> Dict:
        with self._lock:
            if self._status == 'active' or self._mode == 'mission':
                return {'ok': False, 'error': 'cannot add waypoint during active task or mission'}
            wid = self._gen_new_wp_id()
            wp = {
                'id': wid,
                'name': name or wid,
                'lat': float(lat),
                'lon': float(lon),
                'yaw_deg': float(yaw_deg or 0.0),
            }
            self._waypoints.append(wp)
            return {'ok': True, 'waypoint': wp}

    def rename_waypoint(self, wp_id: str, new_name: str) -> Dict:
        with self._lock:
            if self._status == 'active':
                return {'ok': False, 'error': 'cannot rename during active task'}
            w = self._find_wp(wp_id)
            if not w:
                return {'ok': False, 'error': f'waypoint not found: {wp_id}'}
            w['name'] = str(new_name)
            return {'ok': True, 'waypoint': w}

    def delete_waypoint(self, wp_id: str) -> Dict:
        with self._lock:
            if self._status == 'active':
                return {'ok': False, 'error': 'cannot delete during active task'}
            before = len(self._waypoints)
            self._waypoints = [w for w in self._waypoints if str(w.get('id')) != str(wp_id)]
            if len(self._waypoints) == before:
                return {'ok': False, 'error': f'waypoint not found: {wp_id}'}
            # Also scrub routes that reference it
            for r in self._routes:
                ids = r.get('waypoint_ids', [])
                r['waypoint_ids'] = [i for i in ids if str(i) != str(wp_id)]
            return {'ok': True}

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

    def list_gps_routes(self) -> List[Dict]:
        with self._lock:
            out = []
            for (frm, to), pts in self._gps_routes.items():
                out.append({'from': frm, 'to': to, 'size': len(pts)})
            return out

    def delete_gps_route(self, from_id: str, to_id: str) -> Dict:
        with self._lock:
            removed = 0
            for key in [(str(from_id), str(to_id)), (str(to_id), str(from_id))]:
                if key in self._gps_routes:
                    del self._gps_routes[key]
                    removed += 1
            return {'ok': True, 'removed': removed}

    def save_gps_route(self, from_id: str, to_id: str, points: List[Dict], tolerance_m: float = 2.0) -> Dict:
        """Save a GPS route between two waypoints, also store reverse.

        Points are expected as [{lat, lon, yaw_deg}]. If start/end are not near
        the waypoints' GPS (within tolerance_m), the route is extended by
        prepending/appending the waypoint GPS as straight segments.
        """
        with self._lock:
            if not from_id or not to_id:
                return {'ok': False, 'error': 'missing from_id/to_id'}
            if not isinstance(points, list) or len(points) < 1:
                return {'ok': False, 'error': 'points must be non-empty list'}
            if str(from_id) == str(to_id):
                return {'ok': False, 'error': 'from and to cannot be the same (no self route)'}
            w_from = self._find_wp(from_id)
            w_to = self._find_wp(to_id)
            if not w_from or not w_to:
                return {'ok': False, 'error': 'from/to waypoint not found'}

            def haversine_m(lat1, lon1, lat2, lon2):
                from math import radians, sin, cos, sqrt, atan2
                R = 6371000.0
                dlat = radians(lat2 - lat1)
                dlon = radians(lon2 - lon1)
                a = sin(dlat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2)**2
                c = 2 * atan2(sqrt(a), sqrt(1 - a))
                return R * c

            cleaned = []
            for p in points:
                if isinstance(p, dict) and 'lat' in p and 'lon' in p:
                    cleaned.append({'lat': float(p['lat']), 'lon': float(p['lon']), 'yaw_deg': float(p.get('yaw_deg', 0.0))})
            if not cleaned:
                return {'ok': False, 'error': 'no valid points provided'}

            def bearing_deg(lat1, lon1, lat2, lon2):
                from math import radians, degrees, sin, cos, atan2
                y = sin(radians(lon2 - lon1)) * cos(radians(lat2))
                x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(lon2 - lon1))
                brng = degrees(atan2(y, x))
                return (brng + 360.0) % 360.0

            def interpolate_line(lat1, lon1, lat2, lon2, step_m=3.0):
                d = haversine_m(lat1, lon1, lat2, lon2)
                if d <= step_m:
                    return []
                n = int(d // step_m)
                pts = []
                for i in range(1, n + 1):
                    t = i / (n + 1)
                    lat = lat1 + (lat2 - lat1) * t
                    lon = lon1 + (lon2 - lon1) * t
                    pts.append({'lat': lat, 'lon': lon})
                return pts

            # Ensure connection from waypoint to first point (insert straight-line segments broken into few meters)
            wlat1, wlon1 = float(w_from['lat']), float(w_from['lon'])
            d0 = haversine_m(wlat1, wlon1, cleaned[0]['lat'], cleaned[0]['lon'])
            if d0 > tolerance_m:
                brg = bearing_deg(wlat1, wlon1, cleaned[0]['lat'], cleaned[0]['lon'])
                segs = [{'lat': wlat1, 'lon': wlon1}] + interpolate_line(wlat1, wlon1, cleaned[0]['lat'], cleaned[0]['lon'])
                cleaned = [{'lat': p['lat'], 'lon': p['lon'], 'yaw_deg': brg} for p in segs] + cleaned

            # Ensure connection to end waypoint (insert straight-line segments)
            wlat2, wlon2 = float(w_to['lat']), float(w_to['lon'])
            de = haversine_m(cleaned[-1]['lat'], cleaned[-1]['lon'], wlat2, wlon2)
            if de > tolerance_m:
                brg2 = bearing_deg(cleaned[-1]['lat'], cleaned[-1]['lon'], wlat2, wlon2)
                segs2 = interpolate_line(cleaned[-1]['lat'], cleaned[-1]['lon'], wlat2, wlon2) + [{'lat': wlat2, 'lon': wlon2}]
                cleaned = cleaned + [{'lat': p['lat'], 'lon': p['lon'], 'yaw_deg': brg2} for p in segs2]

            # Recompute yaw along forward path
            for i in range(len(cleaned)):
                if i < len(cleaned) - 1:
                    cleaned[i]['yaw_deg'] = bearing_deg(cleaned[i]['lat'], cleaned[i]['lon'], cleaned[i+1]['lat'], cleaned[i+1]['lon'])
                elif i > 0:
                    cleaned[i]['yaw_deg'] = cleaned[i-1]['yaw_deg']
            key = (str(from_id), str(to_id))
            self._gps_routes[key] = cleaned
            # Build reverse with recomputed yaw
            rev = list(reversed([{ 'lat': p['lat'], 'lon': p['lon'], 'yaw_deg': p.get('yaw_deg', 0.0) } for p in cleaned]))
            for i in range(len(rev)):
                if i < len(rev) - 1:
                    rev[i]['yaw_deg'] = bearing_deg(rev[i]['lat'], rev[i]['lon'], rev[i+1]['lat'], rev[i+1]['lon'])
                elif i > 0:
                    rev[i]['yaw_deg'] = rev[i-1]['yaw_deg']
            self._gps_routes[(str(to_id), str(from_id))] = rev
            return {'ok': True, 'size': len(cleaned)}

    def follow_gps_route(self, from_id: str, to_id: str) -> Dict:
        with self._lock:
            key = (str(from_id), str(to_id))
            pts = self._gps_routes.get(key)
            if not pts:
                return {'ok': False, 'error': 'gps route not found'}
            try:
                poses = []
                with ROS_SPIN_LOCK:
                    for p in pts:
                        local = gps_point_to_local(self._helper, {'lat': p['lat'], 'lon': p['lon'], 'yaw_deg': p.get('yaw_deg', 0.0)})
                        poses.append(build_goal_pose(self._navigator, local['x'], local['y'], local['yaw_rad']))
                    self._navigator.goThroughPoses(poses)
                self._last_goal_id = None
                self._status = 'active'
                self._last_error = None
                # Keep mission mode if active
                self._mode = 'mission' if getattr(self, '_mission_active', False) else 'route'
                self._route_id = f"{from_id}->{to_id}"
                self._route_total = len(poses)
                self._route_current = 0
                self._active_gps_pair = (str(from_id), str(to_id))
                return {'ok': True, 'status': self._status, 'route_id': self._route_id}
            except Exception as e:
                self._status = 'error'
                self._last_error = str(e)
                return {'ok': False, 'error': str(e)}

    def publish_cmd_vel(self, lx_norm: float, az_norm: float) -> Dict:
        """Publish normalized cmd_vel from joystick to /cmd_vel.

        - lx_norm: linear.x in [-1, 1]
        - az_norm: angular.z in [-1, 1]
        If both are near zero, do not publish (per requirement).
        """
        with self._lock:
            try:
                lx = max(-1.0, min(1.0, float(lx_norm)))
                az = max(-1.0, min(1.0, float(az_norm)))
                if abs(lx) < 1e-6 and abs(az) < 1e-6:
                    return {'ok': True, 'skipped': True}
                msg = Twist()
                msg.linear.x = lx * self._max_lin
                msg.angular.z = az * self._max_ang
                self._cmd_pub.publish(msg)
                return {'ok': True}
            except Exception as e:
                return {'ok': False, 'error': str(e)}

    def list_waypoints_local(self) -> List[Dict]:
        """Return waypoints converted to local map frame coordinates.

        Fields: id, name, x, y, yaw_deg
        """
        with self._lock:
            try:
                with ROS_SPIN_LOCK:
                    locals_ = gps_points_to_local(self._helper, self._waypoints)
                out = []
                for item in locals_:
                    out.append({
                        'id': item.get('id'),
                        'name': item.get('name'),
                        'x': float(item['x']),
                        'y': float(item['y']),
                        'yaw_deg': float((item.get('yaw_rad') or 0.0) * 180.0 / 3.141592653589793),
                    })
                return out
            except Exception as e:
                logging.exception('Failed to convert waypoints to local frame: %s', e)
                return []

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
                        poses.append(build_goal_pose(self._navigator, local['x'], local['y'], local['yaw_rad']))

                    self._navigator.goThroughPoses(poses)
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
                self._active_gps_pair = None
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
                'gps_routes': [
                    {'from': k[0], 'to': k[1], 'points': list(v)} for k, v in self._gps_routes.items()
                ],
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
            # gps_routes
            self._gps_routes = {}
            gps_routes_raw = data.get('gps_routes', [])
            if isinstance(gps_routes_raw, list):
                for gr in gps_routes_raw:
                    if isinstance(gr, dict) and 'from' in gr and 'to' in gr and isinstance(gr.get('points'), list):
                        key = (str(gr['from']), str(gr['to']))
                        pts = []
                        for p in gr['points']:
                            try:
                                pts.append({'lat': float(p['lat']), 'lon': float(p['lon']), 'yaw_deg': float(p.get('yaw_deg', 0.0))})
                            except Exception:
                                continue
                        if pts:
                            self._gps_routes[key] = pts
            return {'ok': True}

    def cancel(self) -> Dict:
        with self._lock:
            try:
                with ROS_SPIN_LOCK:
                    self._navigator.cancelTask()
                # Mark as canceling; final status will settle on next poll
                self._status = 'canceled'
                self._active_gps_pair = None
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

                # Capture progress info for routes (JSON or GPS) when available
                if (self._route_total and self._route_total > 0) or self._active_gps_pair is not None or self._mode == 'route':
                    idx = None
                    if fb is not None and hasattr(fb, 'current_waypoint'):
                        try:
                            idx = int(fb.current_waypoint)
                        except Exception:
                            idx = None
                    if idx is None:
                        idx = self._route_current or 0
                    if self._active_gps_pair is not None:
                        self._paused = {
                            'mode': 'gps_route',
                            'from': self._active_gps_pair[0],
                            'to': self._active_gps_pair[1],
                            'index': max(0, int(idx)),
                        }
                    else:
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
                    # Preserve mission mode if mission is active
                    self._mode = 'mission' if getattr(self, '_mission_active', False) else 'single'
                    self._route_id = None
                    self._route_total = 0
                    self._route_current = None
                    return {'ok': True, 'status': self._status, 'goal_id': wp_id}

                # GPS route resume
                if paused.get('mode') == 'gps_route':
                    from_id = paused.get('from'); to_id = paused.get('to')
                    start_idx = int(paused.get('index', 0))
                    key = (str(from_id), str(to_id))
                    pts = self._gps_routes.get(key)
                    if not pts or start_idx >= len(pts):
                        self._status = 'succeeded'
                        self._mode = 'none'
                        self._route_id = None
                        self._route_total = 0
                        self._route_current = None
                        self._active_gps_pair = None
                        return {'ok': True, 'status': 'succeeded'}
                    poses = []
                    with ROS_SPIN_LOCK:
                        for p in pts[start_idx:]:
                            local = gps_point_to_local(self._helper, {'lat': p['lat'], 'lon': p['lon'], 'yaw_deg': p.get('yaw_deg', 0.0)})
                            poses.append(build_goal_pose(self._navigator, local['x'], local['y'], local['yaw_rad']))
                        self._navigator.goThroughPoses(poses)
                    self._last_goal_id = None
                    self._status = 'active'
                    self._last_error = None
                    self._mode = 'mission' if getattr(self, '_mission_active', False) else 'route'
                    self._route_id = f"{from_id}->{to_id}"
                    self._route_total = len(poses)
                    self._route_current = 0
                    self._active_gps_pair = (str(from_id), str(to_id))
                    return {'ok': True, 'status': self._status, 'route_id': self._route_id}

                # JSON route resume
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

                    self._navigator.goThroughPoses(poses)
                self._last_goal_id = None
                self._status = 'active'
                self._last_error = None
                self._mode = 'mission' if getattr(self, '_mission_active', False) else 'route'
                self._route_id = route_id
                # Track progress for the resumed segment
                self._route_total = len(poses)
                self._route_current = 0
                self._active_gps_pair = None
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

                    # Route progress if available (update regardless of mode)
                    if fb is not None and hasattr(fb, 'current_waypoint'):
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

    # Startup: default to no waypoints unless WP_JSON_PATH is provided
    wp_env = os.environ.get('WP_JSON_PATH')
    if wp_env and Path(wp_env).exists():
        json_path = Path(wp_env)
        waypoints, routes = load_waypoints_and_routes(json_path)
    else:
        waypoints, routes = [], []
    nav = NavBridge(waypoints, routes)

    # On-demand telemetry sampling to avoid concurrent rclpy spinning
    _last_gps: Optional[Dict] = None
    _last_gps_ts: Optional[float] = None
    _last_heading_deg: Optional[float] = None
    _last_heading_ts: Optional[float] = None
    _last_odom: Optional[Dict] = None
    _last_odom_ts: Optional[float] = None
    _last_plan: Optional[Dict] = None  # {'points': [{'x','y'}], 'frame': str}
    _last_plan_ts: Optional[float] = None

    def _quat_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        deg = math.degrees(yaw)
        if deg < 0:
            deg += 360.0
        return deg

    def _sample_telem_once(timeout_sec: float = 0.5) -> Tuple[Optional[Dict], Optional[float], Optional[Dict], Optional[Dict]]:
        nonlocal _last_gps, _last_gps_ts, _last_heading_deg, _last_heading_ts, _last_odom, _last_odom_ts, _last_plan, _last_plan_ts
        try:
            node = Node('wp_dashboard_telemetry_once')
            got_gps = Future()
            got_imu = Future()
            got_odom = Future()
            got_plan = Future()
            data: Dict = {}

            def gps_cb(msg: NavSatFix):
                data['gps'] = {
                    'lat': float(msg.latitude),
                    'lon': float(msg.longitude),
                    'alt': float(msg.altitude),
                    'status': getattr(getattr(msg, 'status', None), 'status', None),
                    'service': getattr(getattr(msg, 'status', None), 'service', None),
                }
                if not got_gps.done():
                    got_gps.set_result(True)

            def imu_cb(msg: Imu):
                q = msg.orientation
                try:
                    hdg = _quat_to_yaw_deg(float(q.x), float(q.y), float(q.z), float(q.w))
                    data['heading_deg'] = hdg
                except Exception:
                    pass
                if not got_imu.done():
                    got_imu.set_result(True)

            def odom_cb(msg: Odometry):
                try:
                    px = float(msg.pose.pose.position.x)
                    py = float(msg.pose.pose.position.y)
                    oq = msg.pose.pose.orientation
                    yawd = _quat_to_yaw_deg(float(oq.x), float(oq.y), float(oq.z), float(oq.w))
                    data['odom'] = {'x': px, 'y': py, 'yaw_deg': yawd}
                except Exception:
                    pass
                if not got_odom.done():
                    got_odom.set_result(True)

            node.create_subscription(NavSatFix, '/gps/fix', gps_cb, 10)
            node.create_subscription(Imu, '/imu', imu_cb, 10)
            node.create_subscription(Odometry, '/odometry/global', odom_cb, 10)
            def plan_cb(msg: NavPath):
                try:
                    pts = []
                    for ps in list(msg.poses)[:2000]:
                        pts.append({'x': float(ps.pose.position.x), 'y': float(ps.pose.position.y)})
                    data['plan'] = {'points': pts, 'frame': str(msg.header.frame_id)}
                except Exception:
                    pass
                if not got_plan.done():
                    got_plan.set_result(True)
            node.create_subscription(NavPath, '/plan', plan_cb, 10)
            with ROS_SPIN_LOCK:
                rclpy.spin_until_future_complete(node, got_gps, timeout_sec=timeout_sec)
                rclpy.spin_until_future_complete(node, got_imu, timeout_sec=timeout_sec)
                rclpy.spin_until_future_complete(node, got_odom, timeout_sec=timeout_sec)
                rclpy.spin_until_future_complete(node, got_plan, timeout_sec=timeout_sec)
            node.destroy_node()
            now = time.time()
            if got_gps.done() and 'gps' in data:
                _last_gps = data['gps']
                _last_gps_ts = now
            if got_imu.done() and 'heading_deg' in data:
                _last_heading_deg = float(data['heading_deg'])
                _last_heading_ts = now
            if got_odom.done() and 'odom' in data:
                _last_odom = data['odom']
                _last_odom_ts = now
            if got_plan.done() and 'plan' in data:
                _last_plan = data['plan']
                _last_plan_ts = now
            return _last_gps, _last_heading_deg, _last_odom, _last_plan
        except Exception:
            return _last_gps, _last_heading_deg, _last_odom, _last_plan

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

    @app.get('/api/gps_routes')
    def api_gps_routes():
        return jsonify(nav.list_gps_routes())

    @app.get('/api/waypoints_local')
    def api_waypoints_local():
        return jsonify(nav.list_waypoints_local())

    @app.post('/api/route/<route_id>')
    def api_start_route(route_id: str):
        return jsonify(nav.start_route(route_id))

    @app.post('/api/gps_routes/save')
    def api_gps_routes_save():
        payload = request.get_json(silent=True) or {}
        frm = payload.get('from_id'); to = payload.get('to_id'); points = payload.get('points') or []
        tol = float(payload.get('tolerance_m', 2.0))
        return jsonify(nav.save_gps_route(str(frm), str(to), points, tolerance_m=tol))

    @app.post('/api/gps_routes/follow')
    def api_gps_routes_follow():
        payload = request.get_json(silent=True) or {}
        frm = payload.get('from_id'); to = payload.get('to_id')
        return jsonify(nav.follow_gps_route(str(frm), str(to)))

    @app.post('/api/gps_routes/delete')
    def api_gps_routes_delete():
        payload = request.get_json(silent=True) or {}
        frm = payload.get('from_id'); to = payload.get('to_id')
        if not frm or not to:
            return jsonify({'ok': False, 'error': 'from_id and to_id required'}), 400
        return jsonify(nav.delete_gps_route(str(frm), str(to)))

    @app.post('/api/gps_routes/preview_local')
    def api_gps_routes_preview_local():
        payload = request.get_json(silent=True) or {}
        points = payload.get('points') or []
        try:
            with ROS_SPIN_LOCK:
                locals_ = []
                for p in points:
                    if not isinstance(p, dict):
                        continue
                    if 'lat' not in p or 'lon' not in p:
                        continue
                    local = gps_point_to_local(nav._helper, p)
                    locals_.append({'x': float(local['x']), 'y': float(local['y'])})
            return jsonify({'ok': True, 'points': locals_})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500

    @app.get('/api/gps_routes/local')
    def api_gps_routes_local():
        frm = request.args.get('from_id'); to = request.args.get('to_id')
        if not frm or not to:
            return jsonify({'ok': False, 'error': 'from_id and to_id required'}), 400
        key = (str(frm), str(to))
        pts = nav._gps_routes.get(key) or []
        try:
            with ROS_SPIN_LOCK:
                locals_ = []
                for p in pts:
                    local = gps_point_to_local(nav._helper, p)
                    locals_.append({'x': float(local['x']), 'y': float(local['y'])})
            return jsonify({'ok': True, 'points': locals_})
        except Exception as e:
            return jsonify({'ok': False, 'error': str(e)}), 500

    @app.post('/api/mission/goal/<dest_id>')
    def api_mission_goal(dest_id: str):
        # Determine nearest waypoint to current GPS fix; if a route exists, follow it, else send single goal
        gps, heading, _, _ = _sample_telem_once()
        from_id = None
        try:
            data = nav.list_waypoints()
            best_d = 1e18
            if gps and 'lat' in gps and 'lon' in gps:
                glat = float(gps['lat']); glon = float(gps['lon'])
                from math import radians, sin, cos, sqrt, atan2
                def haversine(a,b,c,d):
                    R=6371000.0; dlat=radians(c-a); dlon=radians(d-b)
                    aa = sin(dlat/2)**2 + cos(radians(a))*cos(radians(c))*sin(dlon/2)**2
                    return 2*R*atan2(sqrt(aa), sqrt(1-aa))
                for w in data:
                    try:
                        d = haversine(glat, glon, float(w['lat']), float(w['lon']))
                        if d < best_d:
                            best_d = d; from_id = w['id']
                    except Exception:
                        continue
        except Exception:
            from_id = None
        if from_id and (from_id, dest_id) in nav._gps_routes:
            return jsonify(nav.follow_gps_route(from_id, dest_id))
        # Fallback to single goal
        return jsonify(nav.send_goal(dest_id))

    @app.post('/api/clear_all')
    def api_clear_all():
        # Cancel any running task and clear all waypoints, routes, gps_routes
        try:
            with ROS_SPIN_LOCK:
                nav._navigator.cancelTask()
        except Exception:
            pass
        with nav._lock:
            nav._mission_active = False
            nav._paused = None
            nav._last_goal_id = None
            nav._mode = 'none'
            nav._route_id = None
            nav._route_total = 0
            nav._route_current = None
            nav._status = 'idle'
            nav._last_error = None
            nav._waypoints = []
            nav._routes = []
            nav._gps_routes = {}
        return jsonify({'ok': True})

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
        gps, heading, odom, plan = _sample_telem_once()
        return jsonify({'gps': gps, 'heading_deg': heading, 'odom': odom, 'plan': plan})

    # Waypoint management endpoints
    @app.post('/api/waypoints/add_current')
    def api_waypoints_add_current():
        # Only allow when not in mission and not active
        if nav._status == 'active' or nav._mode == 'mission':
            return jsonify({'ok': False, 'error': 'not allowed during active task or mission'}), 400
        gps, heading, _, _ = _sample_telem_once()
        if not gps or 'lat' not in gps or 'lon' not in gps:
            return jsonify({'ok': False, 'error': 'no GPS fix available'}), 400
        yaw = float(heading) if heading is not None else 0.0
        name = request.json.get('name') if request.is_json else None
        return jsonify(nav.add_waypoint(float(gps['lat']), float(gps['lon']), yaw, name=name))

    @app.post('/api/waypoints/rename/<wp_id>')
    def api_waypoints_rename(wp_id: str):
        payload = request.get_json(silent=True) or {}
        new_name = payload.get('name')
        if not new_name:
            return jsonify({'ok': False, 'error': 'missing name'}), 400
        return jsonify(nav.rename_waypoint(wp_id, str(new_name)))

    @app.delete('/api/waypoints/<wp_id>')
    def api_waypoints_delete(wp_id: str):
        return jsonify(nav.delete_waypoint(wp_id))

    @app.post('/api/cmd_vel')
    def api_cmd_vel():
        payload = request.get_json(silent=True) or {}
        try:
            lx = float(payload.get('lx', 0.0))
            az = float(payload.get('az', 0.0))
        except Exception:
            return jsonify({'ok': False, 'error': 'invalid payload'}), 400
        return jsonify(nav.publish_cmd_vel(lx, az))

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
