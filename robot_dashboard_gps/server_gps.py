#!/usr/bin/env python3
import logging
import math
import threading
import time
import os
from typing import Dict
from collections import deque

from flask import Flask, jsonify, request, send_from_directory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from sensor_msgs.msg import NavSatFix, Imu
from nav2_msgs.action import NavigateToPose, FollowPath, NavigateThroughPoses
try:
    from robot_localization.srv import FromLL, ToLL
    HAVE_RL_LL = True
except Exception:
    HAVE_RL_LL = False

try:
    from nav2_simple_commander.basic_navigator import BasicNavigator
    from nav2_simple_commander.robot_navigator import TaskResult
    HAVE_SIMPLE_NAV = True
except Exception:
    HAVE_SIMPLE_NAV = False


def clamp_heading_deg(h):
    return (float(h) + 360.0) % 360.0


class NavCommanderGPS(Node):
    def __init__(self):
        super().__init__('nav_dashboard_commander_gps')
        self.log_node_filter = ['bt_navigator', 'nav2_controller', 'nav2_planner', 'nav_dashboard_commander_gps']
        self.status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }
        # Map frame pose (meters)
        self.pose_map = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0}
        # GPS pose
        self.pose_gps = {'lat': 0.0, 'lon': 0.0, 'heading_deg': 0.0}
        # GPS reference for local tangent plane projection
        self.gps_ref = {
            'lat0': self._env_float('GPS_REF_LAT'),
            'lon0': self._env_float('GPS_REF_LON'),
            'm_per_deg_lat': None,
            'm_per_deg_lon': None,
        }
        self._update_meters_per_degree()

        self.nav2_path_gps = []  # list of {lat, lon}
        self.log_messages = deque(maxlen=100)
        # All route data stored as GPS lat/lon + heading
        self.route_data = { 'waypoints': [], 'routes': {}, 'settings': { 'proximity': 0.5, 'recordDensity': 0.2 } }

        self.mission_mode = 'idle'
        self.is_moving = False
        self.is_paused = False
        self.is_recording = False
        self.recorded_path = []  # [{lat, lon, yaw_deg}]
        self.current_goal_info = {}
        self._record_from_id = None

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Nav2 control stack
        self._navigator = BasicNavigator() if HAVE_SIMPLE_NAV else None
        self._nav_to_pose_client = None
        self._follow_path_client = None
        self._through_poses_client = None
        if not HAVE_SIMPLE_NAV:
            self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self._follow_path_client = ActionClient(self, FollowPath, 'follow_path')
            self._through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._last_goal_type = None
        self._monitor_thread = None
        # Local affine calibration for fast XY<->LL conversion (derived from ToLL)
        self._ll_cal = None  # {'x0':..,'y0':..,'lat0':..,'lon0':..,'J':[[dlat_dx,dlat_dy],[dlon_dx,dlon_dy]], 'J_inv':[[...]], 't':time.time()}
        self._ll_cal_inflight = False

        # Topics
        # Use globally fused odometry; prefer raw GPS fixes for smoother UI movement
        odom_topic = os.getenv('ODOM_TOPIC', '/odometry/filtered/global')
        gps_topic = os.getenv('GPS_TOPIC', '/gps/fix')
        imu_topic = os.getenv('IMU_TOPIC', '/imu')
        # Subscribe with Best Effort and Reliable for compatibility
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Path, '/plan_smoothed', self._plan_cb, 10)
        self.create_subscription(Path, '/received_global_plan', self._plan_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 10)
        self.log_message('GPS Dashboard node started and ready.')
        self._has_imu = False
        self._has_gps = False
        # robot_localization conversion services (optional but preferred)
        fromll_name = os.getenv('FROMLL_SERVICE', '/fromLL')
        toll_name = os.getenv('TOLL_SERVICE', '/toLL')
        self._fromll_cli = self.create_client(FromLL, fromll_name) if HAVE_RL_LL else None
        self._toll_cli = self.create_client(ToLL, toll_name) if HAVE_RL_LL else None
        self.log_message(f"FromLL service: {fromll_name}, ToLL service: {toll_name}")

    def _env_float(self, key):
        try:
            v = os.getenv(key)
            return float(v) if v is not None and v != '' else None
        except Exception:
            return None

    def _update_meters_per_degree(self):
        lat0 = self.gps_ref.get('lat0')
        if lat0 is None:
            # leave as None until we have a reference
            return
        lat_rad = math.radians(lat0)
        # Approx WGS84 meters per degree
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2*lat_rad) + 1.175 * math.cos(4*lat_rad)
        m_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3*lat_rad)
        self.gps_ref['m_per_deg_lat'] = m_per_deg_lat
        self.gps_ref['m_per_deg_lon'] = m_per_deg_lon

    def _latlon_to_xy(self, lat, lon):
        # Prefer robot_localization service for consistent conversion across restarts
        if HAVE_RL_LL and self._fromll_cli is not None and self._fromll_cli.wait_for_service(timeout_sec=0.05):
            try:
                req = FromLL.Request()
                # Support both interface variants
                try:
                    req.latitude = float(lat); req.longitude = float(lon); req.altitude = 0.0
                except AttributeError:
                    from geometry_msgs.msg import Point
                    req.ll_point = Point(x=float(lat), y=float(lon), z=0.0)
                fut = self._fromll_cli.call_async(req)
                t0 = time.time()
                while not fut.done() and (time.time() - t0) < 1.0:
                    time.sleep(0.01)
                if fut.done():
                    res = fut.result()
                    mp = getattr(res, 'map_point', None)
                    if mp is not None:
                        return float(mp.x), float(mp.y)
                    x = getattr(res, 'x', None); y = getattr(res, 'y', None)
                    if x is not None and y is not None:
                        return float(x), float(y)
            except Exception:
                pass
        # Fallback: local tangent-plane approximation
        lat0 = self.gps_ref.get('lat0'); lon0 = self.gps_ref.get('lon0')
        if lat0 is None or lon0 is None or self.gps_ref.get('m_per_deg_lat') is None:
            return 0.0, 0.0
        dx = (float(lon) - float(lon0)) * self.gps_ref['m_per_deg_lon']
        dy = (float(lat) - float(lat0)) * self.gps_ref['m_per_deg_lat']
        return dx, dy

    def _xy_to_latlon(self, x, y):
        # Prefer robot_localization service for consistent conversion across restarts
        if HAVE_RL_LL and self._toll_cli is not None and self._toll_cli.wait_for_service(timeout_sec=0.05):
            try:
                req = ToLL.Request()
                # Support both interface variants
                try:
                    # Some versions use geometry_msgs/Point
                    from geometry_msgs.msg import Point
                    req.map_point = Point(x=float(x), y=float(y), z=0.0)
                except Exception:
                    try:
                        req.x = float(x); req.y = float(y); req.z = 0.0
                    except Exception:
                        pass
                fut = self._toll_cli.call_async(req)
                t0 = time.time()
                while not fut.done() and (time.time() - t0) < 1.0:
                    time.sleep(0.01)
                if fut.done():
                    res = fut.result()
                    ll = getattr(res, 'll_point', None)
                    if ll is not None:
                        return float(ll.x), float(ll.y)
                    lat = getattr(res, 'latitude', None); lon = getattr(res, 'longitude', None)
                    if lat is not None and lon is not None:
                        return float(lat), float(lon)
            except Exception:
                pass
        # Fallback: local tangent-plane approximation
        lat0 = self.gps_ref.get('lat0'); lon0 = self.gps_ref.get('lon0')
        if lat0 is None or lon0 is None or self.gps_ref.get('m_per_deg_lat') is None:
            return 0.0, 0.0
        lat = float(lat0) + float(y) / self.gps_ref['m_per_deg_lat']
        lon = float(lon0) + float(x) / self.gps_ref['m_per_deg_lon']
        return lat, lon

    def _xy_to_latlon_fast(self, x, y):
        """Fast, non-blocking XY->LL for UI drawing only.
        Avoids calling /toLL inside high-frequency callbacks like /plan.
        """
        lat0 = self.gps_ref.get('lat0'); lon0 = self.gps_ref.get('lon0')
        if lat0 is None or lon0 is None or self.gps_ref.get('m_per_deg_lat') is None:
            return 0.0, 0.0
        lat = float(lat0) + float(y) / self.gps_ref['m_per_deg_lat']
        lon = float(lon0) + float(x) / self.gps_ref['m_per_deg_lon']
        return lat, lon

    def _ensure_ll_cal(self):
        if self._ll_cal_inflight or not HAVE_RL_LL or self._toll_cli is None:
            return
        # Calibrate around current map pose for best local accuracy
        x0 = float(self.pose_map.get('x', 0.0))
        y0 = float(self.pose_map.get('y', 0.0))
        step = 10.0  # meters
        def tol_req(px, py):
            req = ToLL.Request()
            try:
                from geometry_msgs.msg import Point
                req.map_point = Point(x=float(px), y=float(py), z=0.0)
            except Exception:
                try:
                    req.x = float(px); req.y = float(py); req.z = 0.0
                except Exception:
                    pass
            return req
        if not self._toll_cli.wait_for_service(timeout_sec=0.2):
            return
        self._ll_cal_inflight = True
        def _cal():
            try:
                f0 = self._toll_cli.call_async(tol_req(x0, y0))
                f1 = self._toll_cli.call_async(tol_req(x0 + step, y0))
                f2 = self._toll_cli.call_async(tol_req(x0, y0 + step))
                t0 = time.time()
                while (not f0.done() or not f1.done() or not f2.done()) and (time.time() - t0) < 2.0:
                    time.sleep(0.01)
                if not (f0.done() and f1.done() and f2.done()):
                    return
                def res_to_ll(fut):
                    res = fut.result()
                    ll = getattr(res, 'll_point', None)
                    if ll is not None:
                        return float(ll.x), float(ll.y)
                    lat = getattr(res, 'latitude', None); lon = getattr(res, 'longitude', None)
                    if lat is not None and lon is not None:
                        return float(lat), float(lon)
                    return None, None
                lat0, lon0 = res_to_ll(f0)
                latx, lonx = res_to_ll(f1)
                laty, lony = res_to_ll(f2)
                if None in (lat0, lon0, latx, lonx, laty, lony):
                    return
                # Compute Jacobian in deg/m
                dlat_dx = (latx - lat0) / step
                dlat_dy = (laty - lat0) / step
                dlon_dx = (lonx - lon0) / step
                dlon_dy = (lony - lon0) / step
                det = dlat_dx * dlon_dy - dlat_dy * dlon_dx
                if abs(det) < 1e-12:
                    return
                inv = [[ dlon_dy / det, -dlat_dy / det],
                       [-dlon_dx / det,  dlat_dx / det]]
                self._ll_cal = {
                    'x0': x0, 'y0': y0,
                    'lat0': lat0, 'lon0': lon0,
                    'J': [[dlat_dx, dlat_dy], [dlon_dx, dlon_dy]],
                    'J_inv': inv,
                    't': time.time()
                }
            finally:
                self._ll_cal_inflight = False
        threading.Thread(target=_cal, daemon=True).start()

    def _xy_to_latlon_affine(self, x, y):
        cal = self._ll_cal
        if cal is None:
            return self._xy_to_latlon_fast(x, y)
        dx = float(x) - cal['x0']
        dy = float(y) - cal['y0']
        dlat = cal['J'][0][0] * dx + cal['J'][0][1] * dy
        dlon = cal['J'][1][0] * dx + cal['J'][1][1] * dy
        return cal['lat0'] + dlat, cal['lon0'] + dlon

    def _latlon_to_xy_affine(self, lat, lon):
        cal = self._ll_cal
        if cal is None:
            return self._latlon_to_xy(float(lat), float(lon))
        dlat = float(lat) - cal['lat0']
        dlon = float(lon) - cal['lon0']
        dx = cal['J_inv'][0][0] * dlat + cal['J_inv'][0][1] * dlon
        dy = cal['J_inv'][1][0] * dlat + cal['J_inv'][1][1] * dlon
        return cal['x0'] + dx, cal['y0'] + dy

    def _latlon_to_xy_robust(self, lat, lon):
        if HAVE_RL_LL and self._fromll_cli is not None and self._fromll_cli.wait_for_service(timeout_sec=0.5):
            try:
                req = FromLL.Request()
                try:
                    req.latitude = float(lat); req.longitude = float(lon); req.altitude = 0.0
                except AttributeError:
                    from geometry_msgs.msg import Point
                    req.ll_point = Point(x=float(lat), y=float(lon), z=0.0)
                fut = self._fromll_cli.call_async(req)
                t0 = time.time()
                while not fut.done() and (time.time() - t0) < 1.5:
                    time.sleep(0.01)
                if fut.done():
                    res = fut.result()
                    mp = getattr(res, 'map_point', None)
                    if mp is not None:
                        return float(mp.x), float(mp.y)
                    x = getattr(res, 'x', None); y = getattr(res, 'y', None)
                    if x is not None and y is not None:
                        return float(x), float(y)
            except Exception:
                pass
        if self._ll_cal is not None:
            try:
                return self._latlon_to_xy_affine(lat, lon)
            except Exception:
                pass
        return self._latlon_to_xy(lat, lon)

    def _latlon_to_xy_strict(self, lat, lon, timeout_sec: float = 1.5):
        """Strict GPS->map using robot_localization FromLL only. Returns (x,y) or None."""
        if not (HAVE_RL_LL and self._fromll_cli is not None and self._fromll_cli.wait_for_service(timeout_sec=timeout_sec)):
            return None
        try:
            req = FromLL.Request()
            try:
                req.latitude = float(lat); req.longitude = float(lon); req.altitude = 0.0
            except AttributeError:
                from geometry_msgs.msg import Point
                req.ll_point = Point(x=float(lat), y=float(lon), z=0.0)
            fut = self._fromll_cli.call_async(req)
            t0 = time.time()
            while not fut.done() and (time.time() - t0) < timeout_sec:
                time.sleep(0.01)
            if fut.done():
                res = fut.result()
                mp = getattr(res, 'map_point', None)
                if mp is not None:
                    return float(mp.x), float(mp.y)
                x = getattr(res, 'x', None); y = getattr(res, 'y', None)
                if x is not None and y is not None:
                    return float(x), float(y)
        except Exception:
            pass
        return None

    def _dist_m(self, lat1, lon1, lat2, lon2):
        x1, y1 = self._latlon_to_xy(lat1, lon1)
        x2, y2 = self._latlon_to_xy(lat2, lon2)
        return math.hypot(x2 - x1, y2 - y1)

    def _within_goal_tolerance(self, target_wp):
        try:
            lat, lon = float(self.pose_gps['lat']), float(self.pose_gps['lon'])
            tlat, tlon = float(target_wp['lat']), float(target_wp['lon'])
            dist = self._dist_m(lat, lon, tlat, tlon)
            tol = float(self.route_data.get('settings', {}).get('proximity', 0.5))
            yaw_tol_deg = 30.0
            tyaw = float(target_wp.get('yaw_deg', 0.0))
            pyaw = float(self.pose_gps.get('heading_deg', 0.0))
            yaw_err = abs((pyaw - tyaw + 180.0) % 360.0 - 180.0)
            return dist <= tol and yaw_err <= yaw_tol_deg
        except Exception:
            return False

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        twist = Twist(); twist.linear.x = linear_x; twist.angular.z = angular_z; self._cmd_vel_pub.publish(twist)

    def _odom_cb(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        yaw_deg = (math.degrees(yaw) + 360.0) % 360.0
        self.pose_map = {'x': x, 'y': y, 'yaw': yaw, 'yaw_deg': yaw_deg}
        # If no GPS fix yet, fall back to projecting odom into GPS frame
        if (not getattr(self, '_has_gps', False)) and self.gps_ref.get('lat0') is not None and self.gps_ref.get('lon0') is not None:
            lat, lon = self._xy_to_latlon(x, y)
            self.pose_gps['lat'] = lat
            self.pose_gps['lon'] = lon
        # Use IMU heading when available; otherwise derive from odom
        if not self._has_imu:
            self.pose_gps['heading_deg'] = yaw_deg
        # Recording in GPS coordinates
        if self.is_recording:
            last_pt = self.recorded_path[-1] if self.recorded_path else None
            lat, lon = self.pose_gps['lat'], self.pose_gps['lon']
            dist = self._dist_m(lat, lon, last_pt['lat'], last_pt['lon']) if last_pt else 999
            if dist > (self.route_data.get('settings', {}).get('recordDensity', 0.2)):
                self.recorded_path.append({'lat': lat, 'lon': lon, 'yaw_deg': self.pose_gps['heading_deg']})

    def _gps_cb(self, msg: NavSatFix):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            # Initialize reference on first good fix if not provided
            if self.gps_ref.get('lat0') is None or self.gps_ref.get('lon0') is None:
                self.gps_ref['lat0'] = float(msg.latitude)
                self.gps_ref['lon0'] = float(msg.longitude)
                self._update_meters_per_degree()
                self.log_message(f"GPS ref set to lat={self.gps_ref['lat0']}, lon={self.gps_ref['lon0']}")
            self.pose_gps['lat'] = float(msg.latitude)
            self.pose_gps['lon'] = float(msg.longitude)
            self._has_gps = True

    def _imu_cb(self, msg: Imu):
        # Compute yaw from IMU orientation quaternion
        o = msg.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.pose_gps['heading_deg'] = (math.degrees(yaw) + 360.0) % 360.0
        self._has_imu = True

    def set_route_data(self, data: Dict):
        settings = (data or {}).get('settings', {})
        data['settings'] = {
            'proximity': float(settings.get('proximity', 0.5)),
            'recordDensity': float(settings.get('recordDensity', 0.2)),
        }
        # Ensure GPS keys exist
        for w in data.get('waypoints', []):
            w['lat'] = float(w.get('lat', 0.0)); w['lon'] = float(w.get('lon', 0.0)); w['yaw_deg'] = float(w.get('yaw_deg', 0.0))
        for key, path in (data.get('routes', {}) or {}).items():
            for p in path:
                p['lat'] = float(p.get('lat', 0.0)); p['lon'] = float(p.get('lon', 0.0)); p['yaw_deg'] = float(p.get('yaw_deg', 0.0))
        self.route_data = data
        self.log_message(f"Updated GPS route data: {len(data.get('waypoints',[]))} waypoints, {len(data.get('routes',{}))} routes.")

    def start_recording(self, from_wp_id):
        if self.mission_mode != 'idle': return
        self.is_recording = True
        self._record_from_id = from_wp_id
        wp = next((w for w in self.route_data['waypoints'] if w['id'] == from_wp_id), None)
        if wp:
            self.recorded_path = [{'lat': float(wp['lat']), 'lon': float(wp['lon']), 'yaw_deg': float(wp.get('yaw_deg', 0.0)), 'id': from_wp_id}]
        else:
            p = {'lat': self.pose_gps['lat'], 'lon': self.pose_gps['lon'], 'yaw_deg': self.pose_gps['heading_deg'], 'id': from_wp_id}
            self.recorded_path = [p]
        self.log_message(f'GPS route recording started from {from_wp_id}.')

    def stop_recording(self, to_wp_id):
        if not self.is_recording: return
        from_wp_id = self.recorded_path[0].get('id') if self.recorded_path else self._record_from_id
        if from_wp_id and to_wp_id:
            to_wp = next((w for w in self.route_data['waypoints'] if w['id'] == to_wp_id), None)
            path = list(self.recorded_path)
            if to_wp:
                if not path:
                    path = [{'lat': float(to_wp['lat']), 'lon': float(to_wp['lon']), 'yaw_deg': float(to_wp.get('yaw_deg', 0.0))}]
                else:
                    last = path[-1]
                    # Interpolate in local XY, then convert back to GPS
                    lx, ly = self._latlon_to_xy(last['lat'], last['lon'])
                    tx, ty = self._latlon_to_xy(to_wp['lat'], to_wp['lon'])
                    dx, dy = tx - lx, ty - ly
                    dist = math.hypot(dx, dy)
                    step = float(self.route_data.get('settings', {}).get('recordDensity', 0.2))
                    if dist > step * 0.5:
                        steps_n = max(1, int(dist / step))
                        for i in range(1, steps_n + 1):
                            t = i / steps_n
                            x = lx + dx * t; y = ly + dy * t
                            lat_i, lon_i = self._xy_to_latlon(x, y)
                            yaw_deg = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
                            path.append({'lat': lat_i, 'lon': lon_i, 'yaw_deg': yaw_deg})
            # Normalize headings along the path; force final yaw to target if provided
            final_yaw = float(to_wp.get('yaw_deg', 0.0)) if to_wp else None
            path = self._apply_headings_gps(path, final_yaw_deg=final_yaw)
            route_key = f"{from_wp_id}-{to_wp_id}"
            self.route_data['routes'][route_key] = path
            # Also create reverse
            rev_key = f"{to_wp_id}-{from_wp_id}"
            rev_path = list(reversed(path))
            from_wp = next((w for w in self.route_data['waypoints'] if w['id'] == from_wp_id), None)
            rev_final_yaw = float(from_wp.get('yaw_deg', 0.0)) if from_wp else None
            rev_path = self._apply_headings_gps(rev_path, final_yaw_deg=rev_final_yaw)
            self.route_data['routes'][rev_key] = rev_path
            self.log_message(f'GPS Route {route_key} saved with {len(path)} points. Reverse saved as {rev_key}.')
        self.is_recording = False; self.recorded_path = []; self._record_from_id = None

    def _apply_headings_gps(self, path, final_yaw_deg: float = None):
        if not path: return []
        out = []
        for i in range(len(path) - 1):
            p = path[i]; n = path[i + 1]
            px, py = self._latlon_to_xy(p['lat'], p['lon'])
            nx, ny = self._latlon_to_xy(n['lat'], n['lon'])
            yaw_deg = (math.degrees(math.atan2(ny - py, nx - px)) + 360.0) % 360.0
            out.append({'lat': float(p['lat']), 'lon': float(p['lon']), 'yaw_deg': yaw_deg})
        last = path[-1]
        if len(path) >= 2 and final_yaw_deg is None:
            pprev = path[-2]
            px, py = self._latlon_to_xy(pprev['lat'], pprev['lon'])
            lx, ly = self._latlon_to_xy(last['lat'], last['lon'])
            yaw_deg = (math.degrees(math.atan2(ly - py, lx - px)) + 360.0) % 360.0
        elif final_yaw_deg is None:
            yaw_deg = float(last.get('yaw_deg', 0.0))
        else:
            yaw_deg = float(final_yaw_deg)
        out.append({'lat': float(last['lat']), 'lon': float(last['lon']), 'yaw_deg': yaw_deg})
        return out

    def start_mission(self):
        self.mission_mode = 'active'; self.is_moving = False; self.is_paused = False; self.log_message('Mission mode started.')

    def stop_mission(self):
        self.mission_mode = 'idle'; self.log_message('Mission mode stopped.'); self.cancel_current_goal()

    def _get_closest_waypoint(self):
        waypoints = self.route_data.get('waypoints', [])
        if not waypoints: return None
        lat, lon = self.pose_gps['lat'], self.pose_gps['lon']
        return min(waypoints, key=lambda wp: self._dist_m(lat, lon, wp['lat'], wp['lon']))

    def go_to_waypoint(self, target_wp_id: str):
        if self.mission_mode != 'active' or self.is_moving: return
        start_wp = self._get_closest_waypoint()
        target_wp = next((w for w in self.route_data['waypoints'] if w['id'] == target_wp_id), None)
        if not start_wp or not target_wp: self.log_message('Start or target waypoint not found.'); return
        self.is_paused = False; self.is_moving = True
        self.current_goal_info = {'start_id': start_wp['id'], 'target_id': target_wp['id'], 'name': target_wp['name'], 'resume_index': 0}

        path_sequence = self._find_path(start_wp['id'], target_wp_id)
        if path_sequence:
            full_path = self._stitch_paths(path_sequence)
            self.current_goal_info['through_path'] = list(full_path)
            self.log_message(f"Following GPS chained route (through poses): {' -> '.join(path_sequence)}")
            self._execute_through_poses(full_path)
        else:
            self.log_message(f"No GPS route found. Navigating directly to '{target_wp['name']}'...")
            self._execute_navigate_to_pose(target_wp)

    def _find_path(self, start_id, end_id):
        if f"{start_id}-{end_id}" in self.route_data['routes']: return [start_id, end_id]
        from collections import deque
        q = deque([[start_id]])
        visited = {start_id}
        while q:
            path = q.popleft()
            node = path[-1]
            if node == end_id: return path
            for neighbor_id in [w['id'] for w in self.route_data['waypoints']]:
                if f"{node}-{neighbor_id}" in self.route_data['routes'] and neighbor_id not in visited:
                    visited.add(neighbor_id); new_path = list(path); new_path.append(neighbor_id); q.append(new_path)
        return None

    def _stitch_paths(self, wp_sequence):
        full_path = []
        for i in range(len(wp_sequence) - 1):
            route_key = f"{wp_sequence[i]}-{wp_sequence[i+1]}"
            path_segment = self.route_data['routes'].get(route_key, [])
            full_path.extend(path_segment)
        if wp_sequence:
            target_id = wp_sequence[-1]
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp:
                if not full_path or (abs(full_path[-1]['lat'] - float(target_wp['lat'])) > 1e-7 or abs(full_path[-1]['lon'] - float(target_wp['lon'])) > 1e-7):
                    full_path.append({'lat': float(target_wp['lat']), 'lon': float(target_wp['lon']), 'yaw_deg': float(target_wp.get('yaw_deg', 0.0))})
                else:
                    full_path[-1]['yaw_deg'] = float(target_wp.get('yaw_deg', full_path[-1].get('yaw_deg', 0.0)))
        return full_path

    def _execute_navigate_to_pose(self, wp_data):
        goal_pose = self._create_pose_stamped_gps(wp_data)
        if goal_pose is None:
            self.is_moving = False
            self.log_message('Goal conversion failed; not sending to Nav2.')
            return
        self._last_goal_type = 'nav_to_pose'
        if HAVE_SIMPLE_NAV and self._navigator is not None:
            self._navigator.goToPose(goal_pose)
            self._start_nav_monitor()
        else:
            if not self._nav_to_pose_client.server_is_ready():
                self.log_message('NavigateToPose server not ready.'); self.is_moving = False; return
            goal_msg = NavigateToPose.Goal(); goal_msg.pose = goal_pose
            self._nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _execute_follow_path(self, path_data):
        if HAVE_SIMPLE_NAV and self._navigator is not None:
            poses = []
            for p in path_data:
                ps = self._create_pose_stamped_gps(p)
                if ps is None:
                    self.log_message('Skipping path point: conversion failed.')
                    continue
                poses.append(ps)
            if not poses:
                self.log_message('No valid path points; aborting follow_path.')
                self.is_moving = False
                return
            try:
                self._last_goal_type = 'through_poses'
                self._navigator.goThroughPoses(poses)
            except AttributeError:
                self._last_goal_type = 'follow_path'
                self._navigator.followWaypoints(poses)
            self._start_nav_monitor(); return
        if not self._follow_path_client or not self._follow_path_client.server_is_ready():
            self.log_message('FollowPath server not ready.'); self.is_moving = False; return
        path_msg = Path(); path_msg.header.frame_id = 'map'
        poses = []
        for p in path_data:
            ps = self._create_pose_stamped_gps(p)
            if ps is not None:
                poses.append(ps)
        if not poses:
            self.log_message('No valid path points; aborting FollowPath action.'); self.is_moving = False; return
        goal_msg = FollowPath.Goal(); path_msg.poses = poses; goal_msg.path = path_msg
        self._last_goal_type = 'follow_path'
        self._follow_path_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _execute_through_poses(self, path_data):
        poses = []
        for p in path_data:
            ps = self._create_pose_stamped_gps(p)
            if ps is None:
                self.log_message('Skipping through-poses point: conversion failed.')
                continue
            poses.append(ps)
        if not poses:
            self.log_message('No valid through-poses; aborting.'); self.is_moving = False; return
        if HAVE_SIMPLE_NAV and self._navigator is not None:
            self._last_goal_type = 'through_poses'
            try:
                self._navigator.goThroughPoses(poses)
            except AttributeError:
                self._navigator.followWaypoints(poses); self._last_goal_type = 'follow_path'
            self._start_nav_monitor(); return
        if not self._through_poses_client or not self._through_poses_client.server_is_ready():
            self.log_message('NavigateThroughPoses server not ready.'); self.is_moving = False; return
        goal_msg = NavigateThroughPoses.Goal(); goal_msg.poses = poses
        self._last_goal_type = 'through_poses'
        self._through_poses_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted: self.log_message('Goal rejected by server.'); self.is_moving = False; return
        self._goal_handle.get_result_async().add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        if self.is_paused: self.log_message('Navigation paused successfully.'); self.is_moving = False; return
        status = future.result().status; status_text = self.status_map.get(status, f'UNKNOWN ({status})')
        self.log_message(f"Navigation to '{self.current_goal_info.get('name')}' finished with status: {status_text}")
        if self.mission_mode == 'active' and status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            target_id = self.current_goal_info.get('target_id')
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp and self._within_goal_tolerance(target_wp):
                self.log_message('Within goal tolerance; marking success.'); self.is_moving = False; self.is_paused = False; return
        if self.mission_mode == 'active' and status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED) and self._last_goal_type == 'through_poses':
            route = self.current_goal_info.get('through_path', [])
            if route:
                self.log_message("Route aborted; retrying from current progress in 2s...")
                def retry():
                    if self.mission_mode == 'active' and not self.is_paused:
                        self._resume_route_from_progress()
                threading.Timer(2.0, retry).start(); return
        self.is_moving = False; self.is_paused = False

    def _start_nav_monitor(self):
        self.is_moving = True
        def _monitor():
            try:
                check_count = 0
                while not self.is_paused and not self._navigator.isTaskComplete():
                    fb = self._navigator.getFeedback()
                    if fb and (check_count % 10 == 0):
                        try:
                            eta = getattr(getattr(fb, 'estimated_time_remaining', None), 'sec', None)
                            if eta is not None:
                                self.log_message(f'ETA: ~{eta}s')
                        except Exception:
                            pass
                    check_count += 1
                    time.sleep(0.1)
                if self.is_paused:
                    self.log_message('Navigation paused successfully.'); self.is_moving = False; return
                status_text = 'UNKNOWN'
                try:
                    result = self._navigator.getResult()
                    if result == TaskResult.SUCCEEDED: status_text = 'SUCCEEDED'
                    elif result == TaskResult.CANCELED: status_text = 'CANCELED'
                    elif result == TaskResult.FAILED: status_text = 'FAILED'
                except Exception as e:
                    status_text = f'UNKNOWN ({e})'
                self.log_message(f"Navigation to '{self.current_goal_info.get('name')}' finished with status: {status_text}")
                if self.mission_mode == 'active' and status_text in ('FAILED', 'CANCELED') and self._last_goal_type == 'through_poses':
                    route = self.current_goal_info.get('through_path', [])
                    if route:
                        self.log_message('Route aborted; retrying from current progress in 2s...')
                        threading.Timer(2.0, lambda: (not self.is_paused) and self._resume_route_from_progress()).start(); return
            finally:
                self.is_moving = False; self.is_paused = False
        self._monitor_thread = threading.Thread(target=_monitor, daemon=True)
        self._monitor_thread.start()

    def pause_mission(self):
        if not self.is_moving or self.is_paused: return
        self.is_paused = True
        self.log_message('Pausing current navigation goal...')
        if HAVE_SIMPLE_NAV and self._navigator is not None:
            try: self._navigator.cancelTask()
            except Exception: pass
        else:
            self.cancel_current_goal(paused_cancel=True)

    def resume_mission(self):
        if not self.is_paused: return
        self.is_paused = False
        self.log_message(f"Resuming navigation to '{self.current_goal_info.get('name')}'...")
        if not self._resume_route_from_progress():
            target_id = self.current_goal_info.get('target_id')
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp:
                self.is_moving = True
                self._execute_navigate_to_pose(target_wp)

    def cancel_current_goal(self, paused_cancel=False):
        if not paused_cancel: self.is_moving = False
        self.is_paused = paused_cancel
        if HAVE_SIMPLE_NAV and self._navigator is not None:
            try: self._navigator.cancelTask()
            except Exception: pass
        else:
            if getattr(self, '_goal_handle', None) and self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
                self._goal_handle.cancel_goal_async()

    def _resume_route_from_progress(self) -> bool:
        route = self.current_goal_info.get('through_path', [])
        if not route: return False
        try:
            plat, plon = float(self.pose_gps['lat']), float(self.pose_gps['lon'])
            best_i = min(range(len(route)), key=lambda i: self._dist_m(plat, plon, float(route[i]['lat']), float(route[i]['lon'])))
            prox = float(self.route_data.get('settings', {}).get('proximity', 0.5))
            d = self._dist_m(plat, plon, float(route[best_i]['lat']), float(route[best_i]['lon']))
            resume_index = int(self.current_goal_info.get('resume_index', 0))
            candidate = min(best_i + 1, len(route) - 1) if d <= prox else best_i
            start_i = max(candidate, resume_index)
            subpath = route[start_i:]
            if len(subpath) >= 2:
                self.is_moving = True
                self._execute_through_poses(subpath)
                self.current_goal_info['resume_index'] = start_i
                return True
        except Exception as e:
            self.log_message(f"Resume from progress failed: {e}")
        return False

    def _create_pose_stamped_gps(self, pose_dict):
        # Convert GPS lat/lon to local map XY using strict FromLL for goals
        xy = self._latlon_to_xy_strict(pose_dict['lat'], pose_dict['lon'])
        if xy is None:
            self.log_message(f"FromLL unavailable or failed for goal lat={pose_dict['lat']}, lon={pose_dict['lon']}")
            return None
        x, y = xy
        p = PoseStamped(); p.header.frame_id = os.getenv('WORLD_FRAME', 'map'); p.pose.position.x = float(x); p.pose.position.y = float(y)
        yaw_rad = math.radians(float(pose_dict.get('yaw_deg', 0.0))); o = euler_to_quaternion(0, 0, yaw_rad)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = o
        return p

    def _plan_cb(self, msg: Path):
        # Temporarily disable path overlay to focus on marker only
        self.nav2_path_gps = []

    def _rosout_cb(self, msg: Log):
        if msg.name in self.log_node_filter and msg.level >= Log.INFO[0]: self.log_messages.append(f'[{msg.name}] {msg.msg}')

    def log_message(self, msg: str, **kwargs):
        self.get_logger().info(msg); self.log_messages.append(f'[dashboard-gps] {msg}')


def euler_from_quaternion(q):
    x, y, z, w = q
    _, _, yaw = euler_from_quaternion_explicit(x, y, z, w)
    return _, _, yaw


def euler_to_quaternion(r, p, y):
    cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)
    cp = math.cos(p * 0.5); sp = math.sin(p * 0.5)
    cr = math.cos(r * 0.5); sr = math.sin(r * 0.5)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


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


app = Flask(__name__, static_url_path="/static"); node = None


@app.route('/')
def index(): return send_from_directory('.', 'index_gps.html')


@app.route('/api/status')
def status():
    # Compose a status payload using GPS structures
    pose = getattr(node, 'pose_map', {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0})
    gps_ref = node.gps_ref.copy()
    resp = jsonify({
        'pose': pose,
        'pose_gps': node.pose_gps,
        'gps_ref': gps_ref,
        'mission_mode': node.mission_mode,
        'is_moving': node.is_moving,
        'is_paused': node.is_paused,
        'is_recording': node.is_recording,
        'nav2_path': node.nav2_path_gps,  # now GPS list
        'logs': list(node.log_messages),
        'route_data': node.route_data,
        'recorded_path': node.recorded_path,
    })
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    return resp


@app.route('/api/set_route_data', methods=['POST'])
def set_route_data(): node.set_route_data(request.get_json(force=True)); return jsonify({'ok': True})


@app.route('/api/start_recording', methods=['POST'])
def start_recording(): node.start_recording(request.get_json(force=True).get('from_wp_id')); return jsonify({'ok': True})


@app.route('/api/stop_recording', methods=['POST'])
def stop_recording(): node.stop_recording(request.get_json(force=True).get('to_wp_id')); return jsonify({'ok': True})


@app.route('/api/start_mission', methods=['POST'])
def start_mission(): node.start_mission(); return jsonify({'ok': True})


@app.route('/api/stop_mission', methods=['POST'])
def stop_mission(): node.stop_mission(); return jsonify({'ok': True})


@app.route('/api/go_to_waypoint', methods=['POST'])
def go_to_waypoint(): node.go_to_waypoint(request.get_json(force=True).get('target_wp_id')); return jsonify({'ok': True})


@app.route('/api/pause', methods=['POST'])
def pause(): node.pause_mission(); return jsonify({'ok': True})


@app.route('/api/resume', methods=['POST'])
def resume(): node.resume_mission(); return jsonify({'ok': True})


@app.route('/api/teleop', methods=['POST'])
def teleop(): data = request.get_json(force=True); node.publish_cmd_vel(float(data.get('linear', {}).get('x', 0.0)), float(data.get('angular', {}).get('z', 0.0))); return jsonify({'ok': True})


def ros_spin():
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
    except Exception:
        executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if HAVE_SIMPLE_NAV and getattr(node, '_navigator', None) is not None:
            nav_node = getattr(node._navigator, 'node', None) or getattr(node._navigator, '_node', None)
            if nav_node is not None:
                executor.add_node(nav_node)
    except Exception:
        pass
    executor.spin()


def main():
    global node; rclpy.init(); node = NavCommanderGPS(); threading.Thread(target=ros_spin, daemon=True).start()
    log = logging.getLogger('werkzeug'); log.setLevel(logging.ERROR)
    port = int(os.getenv('GPS_DASHBOARD_PORT', '8091'))
    print(f"Serving GPS dashboard on http://0.0.0.0:{port}")
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)


if __name__ == '__main__': main()
