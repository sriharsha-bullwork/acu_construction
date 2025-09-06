#!/usr/bin/env python3
import logging
import math
import threading
import time
from typing import List, Dict, Optional
from collections import deque

from flask import Flask, jsonify, request, send_from_directory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from nav2_msgs.action import NavigateToPose, FollowPath, NavigateThroughPoses
from sensor_msgs.msg import NavSatFix, Imu
from robot_localization.srv import FromLL, ToLL


def clamp_angle_deg(a: float) -> float:
    return (a + 360.0) % 360.0


class GPSNavCommander(Node):
    """Dashboard/Commander using GPS (lat/lon) + IMU heading.

    - Subscribes to `/gps/fix` and `/imu` instead of `/odom`.
    - Records routes in GPS (lat/lon/yaw_deg) and mirrors to XY for UI.
    - Converts GPS waypoints to map-frame XY PoseStamped and sends to Nav2.

    The web UI remains untouched (expects XY in meters). We compute XY from
    GPS using a local tangent plane (ENU) w.r.t. the first GPS fix.
    """

    def __init__(self):
        super().__init__('nav_dashboard_gps_commander')
        # Use simulation time to align with TF and Nav2
        try:
            self.declare_parameter('use_sim_time', True)
        except Exception:
            pass
        self.log_node_filter = ['bt_navigator', 'nav2_controller', 'nav2_planner', 'nav_dashboard_gps_commander']
        self.status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }

        # Current pose state: both GPS and local XY for UI drawing
        self.pose = {
            'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0,
            'lat': None, 'lon': None
        }

        # Local tangent plane origin (lat/lon) for ENU conversion
        self._origin_lat: Optional[float] = None
        self._origin_lon: Optional[float] = None

        # Logged nav2 path (XY) for UI rendering
        self.nav2_path = []

        # Console log buffer
        self.log_messages = deque(maxlen=100)

        # Route data for UI (XY meters) and mirror in GPS for navigation
        self.route_data = {
            'waypoints': [],
            'routes': {},
            'settings': {'proximity': 0.2, 'recordDensity': 0.1}
        }
        # GPS mirror: { waypoints_gps: [{id, name, lat, lon, yaw_deg}], routes_gps: { "A-B": [{lat,lon,yaw_deg}, ...] } }
        self._waypoints_gps: Dict[str, Dict] = {}
        self._routes_gps: Dict[str, List[Dict]] = {}

        # Mission state
        self.mission_mode = 'idle'
        self.is_moving = False
        self.is_paused = False
        self.is_recording = False
        self.recorded_path = []          # XY for UI
        self._recorded_path_gps = []     # GPS for Nav2 conversion
        self.current_goal_info = {}
        self._record_from_id = None
        self._goal_handle = None

        # Publishers / action clients
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self._through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._last_goal_type = None  # 'nav_to_pose' | 'follow_path' | 'through_poses'

        # Services: precise LL <-> map conversions from navsat_transform
        # Try multiple canonical names; pick whichever becomes available.
        self._fromll_candidates = [
            self.create_client(FromLL, '/fromLL'),
            self.create_client(FromLL, '/navsat_transform/fromLL'),
        ]
        self._toll_candidates = [
            self.create_client(ToLL, '/toLL'),
            self.create_client(ToLL, '/navsat_transform/toLL'),
        ]
        self._fromll_client = None
        self._toll_client = None
        self.geo_ready = False
        self._geo_logged_ready = False
        self.create_timer(1.0, self._probe_geo_services)

        # Subscriptions: GPS + IMU, Nav2 plan, rosout
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 10)

        self.log_message('GPS Dashboard node started and ready.')

    def _probe_geo_services(self):
        if self._fromll_client is None:
            for c in self._fromll_candidates:
                if c.service_is_ready():
                    self._fromll_client = c
                    break
        if self._toll_client is None:
            for c in self._toll_candidates:
                if c.service_is_ready():
                    self._toll_client = c
                    break
        prev = self.geo_ready
        self.geo_ready = (self._fromll_client is not None and self._toll_client is not None)
        if self.geo_ready and not self._geo_logged_ready:
            # Log once when both services are ready
            self._geo_logged_ready = True
            self.log_message('fromLL/toLL services are ready; using accurate LL<->map conversions.')

    # --- Geo conversions (WGS84 -> local ENU XY) ---
    def _ensure_origin(self, lat: float, lon: float):
        if self._origin_lat is None or self._origin_lon is None:
            self._origin_lat = float(lat)
            self._origin_lon = float(lon)
            self.log_message(f'Set GPS origin lat={self._origin_lat:.8f}, lon={self._origin_lon:.8f}')

    def _meters_per_deg(self, lat_rad: float):
        # Approximate meters per degree using spherical Earth model
        # x uses cos(lat0) scaling; y uses constant radius
        R = 6378137.0  # WGS84 equatorial radius in meters
        m_per_deg_lat = (math.pi / 180.0) * R
        m_per_deg_lon = (math.pi / 180.0) * R * math.cos(lat_rad)
        return m_per_deg_lat, m_per_deg_lon

    # Approximate conversions used as fallback if services unavailable
    def _gps_to_xy_approx(self, lat: float, lon: float):
        if self._origin_lat is None or self._origin_lon is None:
            return 0.0, 0.0
        lat0 = math.radians(self._origin_lat)
        m_per_deg_lat, m_per_deg_lon = self._meters_per_deg(lat0)
        dx = (float(lon) - self._origin_lon) * m_per_deg_lon
        dy = (float(lat) - self._origin_lat) * m_per_deg_lat
        return dx, dy

    def _xy_to_gps_approx(self, x: float, y: float):
        if self._origin_lat is None or self._origin_lon is None:
            return None, None
        lat0 = math.radians(self._origin_lat)
        m_per_deg_lat, m_per_deg_lon = self._meters_per_deg(lat0)
        lat = self._origin_lat + (float(y) / m_per_deg_lat)
        lon = self._origin_lon + (float(x) / m_per_deg_lon)
        return lat, lon

    def _try_fromLL(self, lat: float, lon: float, alt: float = 0.0, timeout: float = 0.5):
        try:
            if self._fromll_client is None:
                return None
            req = FromLL.Request()
            req.ll_point.latitude = float(lat)
            req.ll_point.longitude = float(lon)
            req.ll_point.altitude = float(alt)
            fut = self._fromll_client.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < timeout:
                time.sleep(0.01)
            if fut.done():
                res = fut.result()
                return float(res.map_point.x), float(res.map_point.y), float(res.map_point.z)
        except Exception:
            pass
        return None

    def _try_toLL(self, x: float, y: float, z: float = 0.0, timeout: float = 0.5):
        try:
            if self._toll_client is None:
                return None
            req = ToLL.Request()
            req.map_point = Point(x=float(x), y=float(y), z=float(z))
            fut = self._toll_client.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < timeout:
                time.sleep(0.01)
            if fut.done():
                res = fut.result()
                return float(res.ll_point.latitude), float(res.ll_point.longitude), float(res.ll_point.altitude)
        except Exception:
            pass
        return None

    def ll_to_xy(self, lat: float, lon: float, alt: float = 0.0):
        """Accurate lat/lon -> map XY conversion. Requires fromLL service.
        Returns (x, y) or None if unavailable.
        """
        out = self._try_fromLL(lat, lon, alt)
        if out is not None:
            return out[0], out[1]
        return None

    def xy_to_ll(self, x: float, y: float, z: float = 0.0):
        """Accurate map XY -> lat/lon conversion. Requires toLL service.
        Returns (lat, lon) or None if unavailable.
        """
        out = self._try_toLL(x, y, z)
        if out is not None:
            return out[0], out[1]
        return None

    # --- Subscriptions ---
    def _gps_cb(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self._ensure_origin(msg.latitude, msg.longitude)
        # Compute XY using navsat_transform service if available; do not block in callback
        # Update pose immediately with last known XY; then update asynchronously when service returns
        x_y = self.ll_to_xy(msg.latitude, msg.longitude)
        # Preserve yaw from IMU callback; ensure keys exist
        yaw = float(self.pose.get('yaw', 0.0))
        yaw_deg = clamp_angle_deg(math.degrees(yaw))
        # Update lat/lon and yaw always. Only update XY if accurate conversion is available.
        p = dict(self.pose)
        p['lat'] = float(msg.latitude)
        p['lon'] = float(msg.longitude)
        p['yaw'] = yaw
        p['yaw_deg'] = yaw_deg
        if x_y is not None:
            p['x'], p['y'] = x_y
        self.pose = p
        # Async refine XY with precise service if the previous call fell back to approx
        def _refine_xy_cb(fut):
            try:
                res = fut.result()
                if not res:
                    return
                px = float(res.map_point.x); py = float(res.map_point.y)
                p = dict(self.pose)
                p['x'] = px; p['y'] = py
                self.pose = p
            except Exception:
                pass
        if self._fromll_client is not None and self._fromll_client.service_is_ready():
            req = FromLL.Request()
            req.ll_point.latitude = float(msg.latitude)
            req.ll_point.longitude = float(msg.longitude)
            req.ll_point.altitude = 0.0
            fut = self._fromll_client.call_async(req)
            fut.add_done_callback(_refine_xy_cb)
        # Recording in GPS and mirroring to XY for UI
        if self.is_recording:
            last = self._recorded_path_gps[-1] if self._recorded_path_gps else None
            if not last or self._gps_distance_m(last['lat'], last['lon'], msg.latitude, msg.longitude) > float(self.route_data.get('settings', {}).get('recordDensity', 0.1)):
                self._recorded_path_gps.append({'lat': float(msg.latitude), 'lon': float(msg.longitude), 'yaw_deg': yaw_deg})
                self.recorded_path.append({'x': x, 'y': y, 'lat': float(msg.latitude), 'lon': float(msg.longitude), 'yaw_deg': yaw_deg})

    def _imu_cb(self, msg: Imu):
        o = msg.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        # Update yaw while keeping last known lat/lon and xy
        p = dict(self.pose)
        p['yaw'] = float(yaw)
        p['yaw_deg'] = clamp_angle_deg(math.degrees(yaw))
        self.pose = p

    def _plan_cb(self, msg: Path):
        self.nav2_path = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]

    def _rosout_cb(self, msg: Log):
        if msg.name in self.log_node_filter and msg.level >= Log.INFO[0]:
            self.log_messages.append(f'[{msg.name}] {msg.msg}')

    # --- Utilities ---
    def _gps_distance_m(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        # Use local tangent approximation consistent with _gps_to_xy
        lat0 = math.radians(self._origin_lat if self._origin_lat is not None else lat1)
        m_per_deg_lat, m_per_deg_lon = self._meters_per_deg(lat0)
        dx = (lon2 - lon1) * m_per_deg_lon
        dy = (lat2 - lat1) * m_per_deg_lat
        return math.hypot(dx, dy)

    def _within_goal_tolerance(self, target_wp_xy):
        try:
            px, py = float(self.pose['x']), float(self.pose['y'])
            tx, ty = float(target_wp_xy['x']), float(target_wp_xy['y'])
            dist = math.hypot(px - tx, py - ty)
            tol = float(self.route_data.get('settings', {}).get('proximity', 0.2))
            yaw_tol_deg = 30.0
            tyaw = float(target_wp_xy.get('yaw_deg', 0.0))
            pyaw = float(self.pose.get('yaw_deg', 0.0))
            yaw_err = abs((pyaw - tyaw + 180.0) % 360.0 - 180.0)
            return dist <= tol and yaw_err <= yaw_tol_deg
        except Exception:
            return False

    def log_message(self, msg: str, **kwargs):
        self.get_logger().info(msg)
        self.log_messages.append(f'[dashboard] {msg}')

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    # --- Route/waypoint data mgmt ---
    def set_route_data(self, data: Dict):
        # Keep UI format (XY). Also build GPS mirrors when origin known.
        settings = (data or {}).get('settings', {})
        data['settings'] = {
            'proximity': float(settings.get('proximity', 0.2)),
            'recordDensity': float(settings.get('recordDensity', 0.1)),
        }
        self.route_data = data
        self._rebuild_gps_mirrors()
        self.log_message(f'Updated route data: {len(data.get("waypoints",[]))} waypoints, {len(data.get("routes",{}))} routes.')

    def _rebuild_gps_mirrors(self):
        self._waypoints_gps.clear()
        self._routes_gps.clear()
        # Waypoints: prefer existing lat/lon on entries; otherwise derive if origin known
        for wp in self.route_data.get('waypoints', []):
            lat = wp.get('lat'); lon = wp.get('lon')
            if lat is None or lon is None:
                # Only compute from XY if accurate service is available
                if not self.geo_ready:
                    continue
                ll = self.xy_to_ll(wp.get('x', 0.0), wp.get('y', 0.0))
                if ll is None:
                    continue
                lat, lon = ll
            if lat is None:
                continue
            # Store GPS mirror and backfill lat/lon into route_data for UI/export
            self._waypoints_gps[str(wp['id'])] = {
                'id': wp['id'], 'name': wp.get('name', ''),
                'lat': float(lat), 'lon': float(lon),
                'yaw_deg': float(wp.get('yaw_deg', 0.0))
            }
            wp['lat'] = float(lat); wp['lon'] = float(lon)
        # Routes: prefer lat/lon on points; otherwise derive if origin known
        for key, path in self.route_data.get('routes', {}).items():
            gps_path = []
            for p in path:
                plat = p.get('lat'); plon = p.get('lon')
                if plat is None or plon is None:
                    if not self.geo_ready:
                        continue
                    ll = self.xy_to_ll(p.get('x', 0.0), p.get('y', 0.0))
                    if ll is None:
                        continue
                    plat, plon = ll
                gps_path.append({'lat': float(plat), 'lon': float(plon), 'yaw_deg': float(p.get('yaw_deg', 0.0))})
                # Backfill into UI mirror
                p['lat'] = float(plat); p['lon'] = float(plon)
            self._routes_gps[key] = gps_path

    def start_recording(self, from_wp_id):
        if self.mission_mode != 'idle':
            return
        self.is_recording = True
        self._record_from_id = from_wp_id

        # Seed recorded path with the chosen start waypoint (GPS)
        # If we only have XY, convert to GPS; otherwise use current pose GPS
        start_gps = None
        if from_wp_id is not None:
            # Try GPS mirror first
            start_gps = self._waypoints_gps.get(str(from_wp_id))
            if start_gps is None:
                # Convert from XY to GPS if origin exists
                wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == from_wp_id), None)
                if wp is not None:
                    lat, lon = self.xy_to_ll(wp.get('x', 0.0), wp.get('y', 0.0))
                    if lat is not None:
                        start_gps = {'lat': lat, 'lon': lon, 'yaw_deg': float(wp.get('yaw_deg', 0.0))}

        if start_gps is not None:
            self._recorded_path_gps = [{'lat': float(start_gps['lat']), 'lon': float(start_gps['lon']), 'yaw_deg': float(start_gps.get('yaw_deg', 0.0)), 'id': from_wp_id}]
            # Only mirror to XY if accurate conversion is available
            self.recorded_path = []
            x_y = self.ll_to_xy(start_gps['lat'], start_gps['lon'])
            if x_y is not None:
                x, y = x_y
                self.recorded_path.append({'x': x, 'y': y, 'lat': float(start_gps['lat']), 'lon': float(start_gps['lon']), 'yaw_deg': float(start_gps.get('yaw_deg', 0.0))})
        else:
            # Start at current pose
            lat = self.pose.get('lat')
            lon = self.pose.get('lon')
            yaw_deg = self.pose.get('yaw_deg', 0.0)
            if lat is not None and lon is not None:
                self._recorded_path_gps = [{'lat': float(lat), 'lon': float(lon), 'yaw_deg': float(yaw_deg), 'id': from_wp_id}]
                # Only add XY if accurate conversion previously set pose.x/y
                pr = {'lat': float(lat), 'lon': float(lon), 'yaw_deg': float(yaw_deg)}
                if isinstance(self.pose.get('x', None), (int, float)) and isinstance(self.pose.get('y', None), (int, float)):
                    pr['x'] = float(self.pose['x']); pr['y'] = float(self.pose['y'])
                self.recorded_path = [pr]
            else:
                self._recorded_path_gps = []
                self.recorded_path = []
        self.log_message(f'Route recording started from {from_wp_id}.')

    def stop_recording(self, to_wp_id):
        if not self.is_recording:
            return
        from_wp_id = self._recorded_path_gps[0].get('id') if self._recorded_path_gps else self._record_from_id
        if from_wp_id and to_wp_id:
            # Build final GPS path, optionally interpolate to destination waypoint
            to_wp_gps = self._waypoints_gps.get(str(to_wp_id))
            gps_path = list(self._recorded_path_gps)
            if to_wp_gps is not None:
                if not gps_path:
                    gps_path = [{'lat': float(to_wp_gps['lat']), 'lon': float(to_wp_gps['lon']), 'yaw_deg': float(to_wp_gps.get('yaw_deg', 0.0))}]
                else:
                    last = gps_path[-1]
                    step = float(self.route_data.get('settings', {}).get('recordDensity', 0.1))
                    dist = self._gps_distance_m(last['lat'], last['lon'], to_wp_gps['lat'], to_wp_gps['lon'])
                    if dist > step * 0.5:
                        # interpolate linearly in XY space for simplicity
                        lx, ly = self.ll_to_xy(last['lat'], last['lon'])
                        tx, ty = self.ll_to_xy(to_wp_gps['lat'], to_wp_gps['lon'])
                        dx, dy = (tx - lx), (ty - ly)
                        steps_n = max(1, int(dist / step))
                        for i in range(1, steps_n + 1):
                            t = i / steps_n
                            x = lx + dx * t
                            y = ly + dy * t
                            ilat, ilon = self.xy_to_ll(x, y)
                            yaw_deg = clamp_angle_deg(math.degrees(math.atan2(dy, dx)))
                            gps_path.append({'lat': ilat, 'lon': ilon, 'yaw_deg': yaw_deg})
            # Normalize headings; force final yaw from target if available
            final_yaw = float(to_wp_gps.get('yaw_deg', 0.0)) if to_wp_gps else None
            gps_path = self._apply_headings_gps(gps_path, final_yaw_deg=final_yaw)

            # Save GPS route mirror
            route_key = f"{from_wp_id}-{to_wp_id}"
            self._routes_gps[route_key] = gps_path

            # Also create reverse route in GPS
            rev_key = f"{to_wp_id}-{from_wp_id}"
            rev_path = list(reversed(gps_path))
            # For reverse, set final yaw to from_wp
            from_wp_gps = self._waypoints_gps.get(str(from_wp_id))
            rev_final_yaw = float(from_wp_gps.get('yaw_deg', 0.0)) if from_wp_gps else None
            rev_path = self._apply_headings_gps(rev_path, final_yaw_deg=rev_final_yaw)
            self._routes_gps[rev_key] = rev_path

            # Build UI XY routes from GPS path
            # Update UI mirror only if accurate conversion is available
            if self.geo_ready:
                xy_path = [self._gps_pose_to_xy_dict(p) for p in gps_path]
                self.route_data['routes'][route_key] = xy_path
                self.route_data['routes'][rev_key] = [self._gps_pose_to_xy_dict(p) for p in rev_path]

            self.log_message(f'Route {route_key} saved with {len(gps_path)} points. Reverse saved as {rev_key}.')

        # Reset recording state
        self.is_recording = False
        self.recorded_path = []
        self._recorded_path_gps = []
        self._record_from_id = None

    # --- Mission control ---
    def start_mission(self):
        self.mission_mode = 'active'
        self.is_moving = False
        self.is_paused = False
        self.log_message('Mission mode started.')

    def stop_mission(self):
        self.mission_mode = 'idle'
        self.log_message('Mission mode stopped.')
        self.cancel_current_goal()

    def _get_closest_waypoint_xy(self):
        waypoints = self.route_data.get('waypoints', [])
        if not waypoints:
            return None
        return min(waypoints, key=lambda wp: (self.pose['x'] - wp['x'])**2 + (self.pose['y'] - wp['y'])**2)

    def go_to_waypoint(self, target_wp_id: str):
        if self.mission_mode != 'active' or self.is_moving:
            return

        start_wp_xy = self._get_closest_waypoint_xy()
        target_wp_xy = next((w for w in self.route_data['waypoints'] if w['id'] == target_wp_id), None)
        if not start_wp_xy or not target_wp_xy:
            self.log_message('Start or target waypoint not found.')
            return

        self.is_paused = False
        self.is_moving = True
        self.current_goal_info = {
            'start_id': start_wp_xy['id'],
            'target_id': target_wp_xy['id'],
            'name': target_wp_xy.get('name', str(target_wp_id)),
            'resume_index': 0
        }

        # Prefer chained GPS routes if available, else direct target
        path_sequence = self._find_path(start_wp_xy['id'], target_wp_id)
        if path_sequence:
            full_gps_path = self._stitch_paths_gps(path_sequence)
            self.current_goal_info['through_path'] = [self._gps_pose_to_xy_dict(p) for p in full_gps_path]
            self.log_message(f"Following chained GPS route (through poses): {' -> '.join(path_sequence)}")
            self._execute_through_poses_gps(full_gps_path)
        else:
            self.log_message(f"No route found. Navigating directly to '{target_wp_xy.get('name')}' via GPS...")
            # Build a single GPS waypoint goal (prefer stored lat/lon if present)
            if target_wp_xy.get('lat') is not None and target_wp_xy.get('lon') is not None:
                lat, lon = float(target_wp_xy['lat']), float(target_wp_xy['lon'])
            else:
                lat, lon = self.xy_to_ll(target_wp_xy['x'], target_wp_xy['y'])
            yaw_deg = float(target_wp_xy.get('yaw_deg', 0.0))
            if lat is None:
                self.log_message('No GPS origin set yet; cannot navigate to target.')
                self.is_moving = False
                return
            self._execute_navigate_to_pose_gps({'lat': lat, 'lon': lon, 'yaw_deg': yaw_deg})

    def _find_path(self, start_id, end_id):
        if f"{start_id}-{end_id}" in self._routes_gps:
            return [start_id, end_id]
        q = deque([[start_id]])
        visited = {start_id}
        all_ids = [str(w['id']) for w in self.route_data.get('waypoints', [])]
        while q:
            path = q.popleft()
            node = path[-1]
            if str(node) == str(end_id):
                return path
            for neighbor_id in all_ids:
                if f"{node}-{neighbor_id}" in self._routes_gps and neighbor_id not in visited:
                    visited.add(neighbor_id)
                    new_path = list(path)
                    new_path.append(neighbor_id)
                    q.append(new_path)
        return None

    def _stitch_paths_gps(self, wp_sequence: List[str]) -> List[Dict]:
        full = []
        for i in range(len(wp_sequence) - 1):
            key = f"{wp_sequence[i]}-{wp_sequence[i+1]}"
            seg = self._routes_gps.get(key, [])
            full.extend(seg)
        # Ensure final pose matches exact target waypoint with desired yaw
        if wp_sequence:
            target_id = str(wp_sequence[-1])
            target_wp = self._waypoints_gps.get(target_id)
            if target_wp:
                if not full or (self._gps_distance_m(full[-1]['lat'], full[-1]['lon'], target_wp['lat'], target_wp['lon']) > 0.01):
                    full.append({'lat': float(target_wp['lat']), 'lon': float(target_wp['lon']), 'yaw_deg': float(target_wp.get('yaw_deg', 0.0))})
                else:
                    full[-1]['yaw_deg'] = float(target_wp.get('yaw_deg', full[-1].get('yaw_deg', 0.0)))
        return full

    def _apply_headings_gps(self, path, final_yaw_deg: float = None):
        if not path:
            return []
        out = []
        for i in range(len(path) - 1):
            p = path[i]
            n = path[i + 1]
            px, py = self.ll_to_xy(float(p['lat']), float(p['lon']))
            nx, ny = self.ll_to_xy(float(n['lat']), float(n['lon']))
            yaw_deg = clamp_angle_deg(math.degrees(math.atan2(ny - py, nx - px)))
            out.append({'lat': float(p['lat']), 'lon': float(p['lon']), 'yaw_deg': yaw_deg})
        last = path[-1]
        if len(path) >= 2 and final_yaw_deg is None:
            prev = path[-2]
            lx, ly = self.ll_to_xy(float(last['lat']), float(last['lon']))
            px, py = self.ll_to_xy(float(prev['lat']), float(prev['lon']))
            yaw_deg = clamp_angle_deg(math.degrees(math.atan2(ly - py, lx - px)))
        elif final_yaw_deg is None:
            yaw_deg = float(last.get('yaw_deg', 0.0))
        else:
            yaw_deg = float(final_yaw_deg)
        out.append({'lat': float(last['lat']), 'lon': float(last['lon']), 'yaw_deg': yaw_deg})
        return out

    # --- Nav2 execution (GPS -> XY PoseStamped) ---
    def _gps_pose_to_xy_dict(self, p: Dict) -> Dict:
        x, y = self.ll_to_xy(p['lat'], p['lon'])
        return {'x': x, 'y': y, 'lat': float(p['lat']), 'lon': float(p['lon']), 'yaw_deg': float(p.get('yaw_deg', 0.0))}

    def _create_pose_stamped_xy(self, pose_xy: Dict) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        try:
            p.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        p.pose.position.x = float(pose_xy['x'])
        p.pose.position.y = float(pose_xy['y'])
        yaw_rad = math.radians(float(pose_xy.get('yaw_deg', 0.0)))
        o = euler_to_quaternion(0, 0, yaw_rad)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = o
        return p

    def _execute_navigate_to_pose_gps(self, wp_gps: Dict):
        if not self._nav_to_pose_client.server_is_ready():
            self.log_message('NavigateToPose server not ready.')
            self.is_moving = False
            return
        x_y = self.ll_to_xy(wp_gps['lat'], wp_gps['lon'])
        if x_y is None:
            self.log_message('fromLL service not ready; cannot send NavigateToPose.')
            self.is_moving = False
            return
        x, y = x_y
        goal_pose = self._create_pose_stamped_xy({'x': x, 'y': y, 'yaw_deg': float(wp_gps.get('yaw_deg', 0.0))})
        self.log_message(f"Sending NavigateToPose: frame={goal_pose.header.frame_id} x={goal_pose.pose.position.x:.2f} y={goal_pose.pose.position.y:.2f} yaw={float(wp_gps.get('yaw_deg', 0.0)):.1f}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self._last_goal_type = 'nav_to_pose'
        self._nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _execute_through_poses_gps(self, gps_path: List[Dict]):
        if not self._through_poses_client.server_is_ready():
            self.log_message('NavigateThroughPoses server not ready.')
            self.is_moving = False
            return
        goal_msg = NavigateThroughPoses.Goal()
        # Accurate conversion required
        if not self.geo_ready:
            self.log_message('fromLL service not ready; cannot send NavigateThroughPoses.')
            self.is_moving = False
            return
        xy_path = [self._gps_pose_to_xy_dict(p) for p in gps_path]
        goal_msg.poses = [self._create_pose_stamped_xy(p) for p in xy_path]
        if goal_msg.poses:
            p0 = goal_msg.poses[0]
            pN = goal_msg.poses[-1]
            self.log_message(f"Sending ThroughPoses: N={len(goal_msg.poses)} start=({p0.pose.position.x:.2f},{p0.pose.position.y:.2f}) end=({pN.pose.position.x:.2f},{pN.pose.position.y:.2f})")
        self._last_goal_type = 'through_poses'
        self._through_poses_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    # Keep FollowPath variant for parity (unused by default)
    def _execute_follow_path_xy(self, path_xy: List[Dict]):
        if not self._follow_path_client.server_is_ready():
            self.log_message('FollowPath server not ready.')
            self.is_moving = False
            return
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        try:
            path_msg.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        path_msg.poses = [self._create_pose_stamped_xy(p) for p in path_xy]
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        self._last_goal_type = 'follow_path'
        self._follow_path_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.log_message('Goal rejected by server.')
            self.is_moving = False
            return
        self._goal_handle.get_result_async().add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        if self.is_paused:
            self.log_message('Navigation paused successfully.')
            self.is_moving = False
            return
        status = future.result().status
        status_text = self.status_map.get(status, f'UNKNOWN ({status})')
        self.log_message(f"Navigation to '{self.current_goal_info.get('name')}' finished with status: {status_text}")
        # If aborted/canceled near target, treat as success to allow new inputs
        if self.mission_mode == 'active' and status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            target_id = self.current_goal_info.get('target_id')
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp and self._within_goal_tolerance(target_wp):
                self.log_message('Within goal tolerance; marking success.')
                self.is_moving = False
                self.is_paused = False
                return
        # If a through-poses route aborted (e.g., obstacle), retry from current progress
        if self.mission_mode == 'active' and status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED) and self._last_goal_type == 'through_poses':
            route = self.current_goal_info.get('through_path', [])  # XY path for UI/progress
            if route:
                self.log_message('Route aborted; retrying from current progress in 2s...')
                def retry():
                    if self.mission_mode == 'active' and not self.is_paused:
                        self._resume_route_from_progress_xy()
                threading.Timer(2.0, retry).start()
                return
        self.is_moving = False
        self.is_paused = False

    def pause_mission(self):
        if not self.is_moving or self.is_paused:
            return
        self.is_paused = True
        self.log_message('Pausing current navigation goal...')
        self.cancel_current_goal(paused_cancel=True)

    def resume_mission(self):
        if not self.is_paused:
            return
        self.is_paused = False
        self.log_message(f"Resuming navigation to '{self.current_goal_info.get('name')}'...")
        # Try to resume along the saved route from nearest remaining point; fallback to direct goal
        if not self._resume_route_from_progress_xy():
            target_id = self.current_goal_info.get('target_id')
            target_wp_xy = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp_xy:
                # Fallback: single GPS goal from XY
                lat, lon = self.xy_to_ll(target_wp_xy['x'], target_wp_xy['y'])
                if lat is None:
                    self.log_message('No GPS origin; cannot resume to target.')
                    return
                self.is_moving = True
                self._execute_navigate_to_pose_gps({'lat': lat, 'lon': lon, 'yaw_deg': float(target_wp_xy.get('yaw_deg', 0.0))})

    def cancel_current_goal(self, paused_cancel=False):
        if not paused_cancel:
            self.is_moving = False
        self.is_paused = paused_cancel
        if self._goal_handle and self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self._goal_handle.cancel_goal_async()

    def _resume_route_from_progress_xy(self) -> bool:
        """Resume a through-poses route (XY mirror) from the closest remaining point.
        Returns True if a command was dispatched.
        """
        route_xy = self.current_goal_info.get('through_path', [])
        if not route_xy:
            return False
        try:
            px, py = float(self.pose['x']), float(self.pose['y'])
            # Find closest index along the route
            best_i = min(range(len(route_xy)), key=lambda i: (float(route_xy[i]['x'])-px)**2 + (float(route_xy[i]['y'])-py)**2)
            prox = float(self.route_data.get('settings', {}).get('proximity', 0.2))
            d = math.hypot(px - float(route_xy[best_i]['x']), py - float(route_xy[best_i]['y']))
            # Do not go backwards: honor previously progressed resume_index
            resume_index = int(self.current_goal_info.get('resume_index', 0))
            candidate = min(best_i + 1, len(route_xy) - 1) if d <= prox else best_i
            start_i = max(candidate, resume_index)
            subpath_xy = route_xy[start_i:]
            if len(subpath_xy) >= 2:
                # Convert XY subpath to GPS and execute through poses
                subpath_gps = []
                for p in subpath_xy:
                    ll = self.xy_to_ll(p['x'], p['y'])
                    if ll is None:
                        self.log_message('toLL service not ready; cannot resume route.')
                        return False
                    lat, lon = ll
                    subpath_gps.append({'lat': lat, 'lon': lon, 'yaw_deg': float(p.get('yaw_deg', 0.0))})
                if len(subpath_gps) >= 2:
                    self.is_moving = True
                    self._execute_through_poses_gps(subpath_gps)
                    self.current_goal_info['resume_index'] = start_i
                    return True
        except Exception as e:
            self.log_message(f"Resume from progress failed: {e}")
        return False


# --- Math helpers (quaternion/euler) ---
def euler_from_quaternion(q):
    x, y, z, w = q
    _, _, yaw = euler_from_quaternion_explicit(x, y, z, w)
    return _, _, yaw


def euler_to_quaternion(r, p, y):
    cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5); sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5); sr = math.sin(r * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def euler_from_quaternion_explicit(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    rx = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else -1.0 if t2 < -1.0 else t2
    py = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yz = math.atan2(t3, t4)
    return rx, py, yz


# --- Flask app glue ---
app = Flask(__name__, static_url_path="/static")
node: Optional[GPSNavCommander] = None


@app.route('/')
def index():
    return send_from_directory('.', 'index.html')


@app.route('/api/status')
def status():
    return jsonify({
        'pose': node.pose,
        'mission_mode': node.mission_mode,
        'is_moving': node.is_moving,
        'is_paused': node.is_paused,
        'is_recording': node.is_recording,
        'nav2_path': node.nav2_path,
        'geo_ready': getattr(node, 'geo_ready', False),
        'logs': list(node.log_messages),
        'route_data': node.route_data,
        'recorded_path': node.recorded_path,
    })


@app.route('/api/set_route_data', methods=['POST'])
def set_route_data():
    node.set_route_data(request.get_json(force=True))
    return jsonify({'ok': True})


@app.route('/api/start_recording', methods=['POST'])
def start_recording():
    node.start_recording(request.get_json(force=True).get('from_wp_id'))
    return jsonify({'ok': True})


@app.route('/api/stop_recording', methods=['POST'])
def stop_recording():
    node.stop_recording(request.get_json(force=True).get('to_wp_id'))
    return jsonify({'ok': True})


@app.route('/api/start_mission', methods=['POST'])
def start_mission():
    node.start_mission()
    return jsonify({'ok': True})


@app.route('/api/stop_mission', methods=['POST'])
def stop_mission():
    node.stop_mission()
    return jsonify({'ok': True})


@app.route('/api/go_to_waypoint', methods=['POST'])
def go_to_waypoint():
    node.go_to_waypoint(request.get_json(force=True).get('target_wp_id'))
    return jsonify({'ok': True})


# Optional: send a direct GPS goal (lat/lon/yaw_deg)
@app.route('/api/go_to_ll', methods=['POST'])
def go_to_ll():
    data = request.get_json(force=True)
    try:
        lat = float(data.get('lat'))
        lon = float(data.get('lon'))
    except Exception:
        return jsonify({'ok': False, 'error': 'lat/lon required'}), 400
    yaw_deg = float(data.get('yaw_deg', 0.0))
    node.is_moving = True
    node.is_paused = False
    node.current_goal_info = {'start_id': None, 'target_id': None, 'name': f'LL({lat:.6f},{lon:.6f})', 'resume_index': 0}
    node._execute_navigate_to_pose_gps({'lat': lat, 'lon': lon, 'yaw_deg': yaw_deg})
    return jsonify({'ok': True})


@app.route('/api/pause', methods=['POST'])
def pause():
    node.pause_mission()
    return jsonify({'ok': True})


@app.route('/api/resume', methods=['POST'])
def resume():
    node.resume_mission()
    return jsonify({'ok': True})


@app.route('/api/teleop', methods=['POST'])
def teleop():
    data = request.get_json(force=True)
    node.publish_cmd_vel(float(data.get('linear', {}).get('x', 0.0)), float(data.get('angular', {}).get('z', 0.0)))
    return jsonify({'ok': True})


# Optional export/import routes for convenience (kept XY format for UI)
@app.route('/api/export_route_data', methods=['GET'])
def export_route_data():
    from flask import Response
    import json
    data = json.dumps(node.route_data, indent=2)
    resp = Response(data, mimetype='application/json')
    resp.headers['Content-Disposition'] = 'attachment; filename=routes.json'
    return resp


@app.route('/api/import_route_data', methods=['POST'])
def import_route_data():
    data = request.get_json(force=True)
    node.set_route_data(data)
    return jsonify({'ok': True})


def ros_spin():
    rclpy.spin(node)


def main():
    global node
    rclpy.init()
    node = GPSNavCommander()
    threading.Thread(target=ros_spin, daemon=True).start()
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    print("Serving GPS dashboard on http://0.0.0.0:8090")
    app.run(host='0.0.0.0', port=8091, debug=False, threaded=True)


if __name__ == '__main__':
    main()
