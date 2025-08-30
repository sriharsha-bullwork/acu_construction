#!/usr/bin/env python3
import logging
import math
import threading
import time
from typing import List, Dict
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
from nav2_msgs.action import NavigateToPose, FollowPath, NavigateThroughPoses

class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_dashboard_commander')
        self.log_node_filter = ['bt_navigator', 'nav2_controller', 'nav2_planner', 'nav_dashboard_commander']
        self.status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0}
        self.nav2_path = []
        self.log_messages = deque(maxlen=100)
        self.route_data = {'waypoints': [], 'routes': {}, 'settings': {'proximity': 0.2, 'recordDensity': 0.1}}
        
        self.mission_mode = 'idle'
        self.is_moving = False
        self.is_paused = False
        self.is_recording = False
        self.recorded_path = []
        self.current_goal_info = {}
        self._record_from_id = None
        
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self._through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._last_goal_type = None  # 'nav_to_pose' | 'follow_path' | 'through_poses'
        
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 10)
        self.log_message('Dashboard node started and ready.')

    def _within_goal_tolerance(self, target_wp):
        try:
            px, py = float(self.pose['x']), float(self.pose['y'])
            tx, ty = float(target_wp['x']), float(target_wp['y'])
            dist = math.hypot(px - tx, py - ty)
            tol = float(self.route_data.get('settings', {}).get('proximity', 0.2))
            # Yaw tolerance is intentionally loose; goal checkers also enforce
            yaw_tol_deg = 30.0
            tyaw = float(target_wp.get('yaw_deg', 0.0))
            pyaw = float(self.pose.get('yaw_deg', 0.0))
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
        self.pose = {'x': x, 'y': y, 'yaw': yaw, 'yaw_deg': (math.degrees(yaw) + 360.0) % 360.0}
        if self.is_recording:
            last_pt = self.recorded_path[-1] if self.recorded_path else None
            dist_sq = (x - last_pt['x'])**2 + (y - last_pt['y'])**2 if last_pt else 999
            if dist_sq > (self.route_data.get('settings', {}).get('recordDensity', 0.1))**2:
                # append a copy to avoid mutating past points when pose updates
                self.recorded_path.append({'x': self.pose['x'], 'y': self.pose['y'], 'yaw_deg': self.pose['yaw_deg']})

    def set_route_data(self, data: Dict):
        # Ensure settings always exist with sane defaults
        settings = (data or {}).get('settings', {})
        data['settings'] = {
            'proximity': float(settings.get('proximity', 0.2)),
            'recordDensity': float(settings.get('recordDensity', 0.1)),
        }
        self.route_data = data
        self.log_message(f'Updated route data: {len(data.get("waypoints",[]))} waypoints, {len(data.get("routes",{}))} routes.')

    def start_recording(self, from_wp_id):
        if self.mission_mode != 'idle': return
        self.is_recording = True
        self._record_from_id = from_wp_id
        wp = next((w for w in self.route_data['waypoints'] if w['id'] == from_wp_id), None)
        if wp:
            # Start at the waypoint with its orientation and keep the id for saving
            self.recorded_path = [{'x': float(wp['x']), 'y': float(wp['y']), 'yaw_deg': float(wp.get('yaw_deg', 0.0)), 'id': from_wp_id}]
        else:
            # Start at current pose, still remember the intended from id
            p = {'x': self.pose['x'], 'y': self.pose['y'], 'yaw_deg': self.pose['yaw_deg'], 'id': from_wp_id}
            self.recorded_path = [p]
        self.log_message(f'Route recording started from {from_wp_id}.')

    def stop_recording(self, to_wp_id):
        if not self.is_recording: return
        from_wp_id = self.recorded_path[0].get('id') if self.recorded_path else self._record_from_id
        if from_wp_id and to_wp_id:
            # Build final path, optionally interpolate straight to the destination waypoint
            to_wp = next((w for w in self.route_data['waypoints'] if w['id'] == to_wp_id), None)
            path = list(self.recorded_path)
            if to_wp:
                if not path:
                    path = [
                        {'x': float(to_wp['x']), 'y': float(to_wp['y']), 'yaw_deg': float(to_wp.get('yaw_deg', 0.0))}
                    ]
                else:
                    last = path[-1]
                    dx = float(to_wp['x']) - float(last['x'])
                    dy = float(to_wp['y']) - float(last['y'])
                    dist = math.hypot(dx, dy)
                    step = float(self.route_data.get('settings', {}).get('recordDensity', 0.1))
                    if dist > step * 0.5:
                        # interpolate points every 'step' meters
                        steps_n = max(1, int(dist / step))
                        for i in range(1, steps_n + 1):
                            t = i / steps_n
                            x = float(last['x']) + dx * t
                            y = float(last['y']) + dy * t
                            yaw_deg = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
                            path.append({'x': x, 'y': y, 'yaw_deg': yaw_deg})
            # Normalize headings along the path, force final yaw to target waypoint yaw if available
            final_yaw = float(to_wp.get('yaw_deg', 0.0)) if to_wp else None
            path = self._apply_headings(path, final_yaw_deg=final_yaw)
            route_key = f"{from_wp_id}-{to_wp_id}"
            self.route_data['routes'][route_key] = path
            # Also create the reverse route
            rev_key = f"{to_wp_id}-{from_wp_id}"
            rev_path = list(reversed(path))
            # For reverse, set final yaw to from_wp's yaw if available
            from_wp = next((w for w in self.route_data['waypoints'] if w['id'] == from_wp_id), None)
            rev_final_yaw = float(from_wp.get('yaw_deg', 0.0)) if from_wp else None
            rev_path = self._apply_headings(rev_path, final_yaw_deg=rev_final_yaw)
            self.route_data['routes'][rev_key] = rev_path
            self.log_message(f'Route {route_key} saved with {len(path)} points. Reverse saved as {rev_key}.')
        self.is_recording = False; self.recorded_path = []; self._record_from_id = None

    def start_mission(self):
        self.mission_mode = 'active'; self.is_moving = False; self.is_paused = False; self.log_message('Mission mode started.')
    def stop_mission(self):
        self.mission_mode = 'idle'; self.log_message('Mission mode stopped.'); self.cancel_current_goal()

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
            self.log_message(f"Following chained route (through poses): {' -> '.join(path_sequence)}")
            self._execute_through_poses(full_path)
        else:
            self.log_message(f"No route found. Navigating directly to '{target_wp['name']}'...")
            self._execute_navigate_to_pose(target_wp)

    def _find_path(self, start_id, end_id):
        if f"{start_id}-{end_id}" in self.route_data['routes']: return [start_id, end_id]
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
        # Ensure final pose matches exact target waypoint with desired yaw
        if wp_sequence:
            target_id = wp_sequence[-1]
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp:
                if not full_path or (abs(full_path[-1]['x'] - float(target_wp['x'])) > 1e-3 or abs(full_path[-1]['y'] - float(target_wp['y'])) > 1e-3):
                    full_path.append({'x': float(target_wp['x']), 'y': float(target_wp['y']), 'yaw_deg': float(target_wp.get('yaw_deg', 0.0))})
                else:
                    # Adjust final yaw to waypoint's yaw to avoid spinning at goal
                    full_path[-1]['yaw_deg'] = float(target_wp.get('yaw_deg', full_path[-1].get('yaw_deg', 0.0)))
        return full_path

    def _get_closest_waypoint(self):
        waypoints = self.route_data.get('waypoints', [])
        if not waypoints: return None
        return min(waypoints, key=lambda wp: (self.pose['x'] - wp['x'])**2 + (self.pose['y'] - wp['y'])**2)

    def _apply_headings(self, path, final_yaw_deg: float = None):
        if not path:
            return []
        out = []
        for i in range(len(path) - 1):
            p = path[i]
            n = path[i + 1]
            dx = float(n['x']) - float(p['x'])
            dy = float(n['y']) - float(p['y'])
            yaw_deg = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
            out.append({'x': float(p['x']), 'y': float(p['y']), 'yaw_deg': yaw_deg})
        last = path[-1]
        if len(path) >= 2 and final_yaw_deg is None:
            prev = path[-2]
            dx = float(last['x']) - float(prev['x'])
            dy = float(last['y']) - float(prev['y'])
            yaw_deg = (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0
        elif final_yaw_deg is None:
            yaw_deg = float(last.get('yaw_deg', 0.0))
        else:
            yaw_deg = float(final_yaw_deg)
        out.append({'x': float(last['x']), 'y': float(last['y']), 'yaw_deg': yaw_deg})
        return out

    def _execute_navigate_to_pose(self, wp_data):
        if not self._nav_to_pose_client.server_is_ready(): self.log_message('NavigateToPose server not ready.'); self.is_moving = False; return
        goal_pose = self._create_pose_stamped(wp_data); goal_msg = NavigateToPose.Goal(); goal_msg.pose = goal_pose
        self._last_goal_type = 'nav_to_pose'
        self._nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _execute_follow_path(self, path_data):
        if not self._follow_path_client.server_is_ready(): self.log_message('FollowPath server not ready.'); self.is_moving = False; return
        path_msg = Path(); path_msg.header.frame_id = 'map'
        path_msg.poses = [self._create_pose_stamped(p) for p in path_data]; goal_msg = FollowPath.Goal(); goal_msg.path = path_msg
        self._last_goal_type = 'follow_path'
        self._follow_path_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)

    def _execute_through_poses(self, path_data):
        if not self._through_poses_client.server_is_ready(): self.log_message('NavigateThroughPoses server not ready.'); self.is_moving = False; return
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self._create_pose_stamped(p) for p in path_data]
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
        # If aborted/canceled near target, treat as success to allow new inputs
        if self.mission_mode == 'active' and status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            target_id = self.current_goal_info.get('target_id')
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp and self._within_goal_tolerance(target_wp):
                self.log_message('Within goal tolerance; marking success.')
                self.is_moving = False; self.is_paused = False; return
        # If a through-poses route aborted (e.g., obstacle), retry from current progress
        if self.mission_mode == 'active' and status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED) and self._last_goal_type == 'through_poses':
            route = self.current_goal_info.get('through_path', [])
            if route:
                self.log_message("Route aborted; retrying from current progress in 2s...")
                def retry():
                    if self.mission_mode == 'active' and not self.is_paused:
                        self._resume_route_from_progress()
                threading.Timer(2.0, retry).start()
                return
        self.is_moving = False; self.is_paused = False

    def pause_mission(self):
        if not self.is_moving or self.is_paused: return
        self.is_paused = True; self.log_message('Pausing current navigation goal...'); self.cancel_current_goal(paused_cancel=True)

    def resume_mission(self):
        if not self.is_paused: return
        self.is_paused = False
        self.log_message(f"Resuming navigation to '{self.current_goal_info.get('name')}'...")
        # Try to resume along the saved route from nearest remaining point; fallback to direct goal
        if not self._resume_route_from_progress():
            target_id = self.current_goal_info.get('target_id')
            target_wp = next((w for w in self.route_data.get('waypoints', []) if w['id'] == target_id), None)
            if target_wp:
                self.is_moving = True
                self._execute_navigate_to_pose(target_wp)

    def cancel_current_goal(self, paused_cancel=False):
        if not paused_cancel: self.is_moving = False
        self.is_paused = paused_cancel
        if self._goal_handle and self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self._goal_handle.cancel_goal_async()

    def _resume_route_from_progress(self) -> bool:
        """Resume a through-poses route from the closest remaining point to the current pose.
        Returns True if a command was dispatched.
        """
        route = self.current_goal_info.get('through_path', [])
        if not route:
            return False
        try:
            px, py = float(self.pose['x']), float(self.pose['y'])
            # Find closest index along the route
            best_i = min(range(len(route)), key=lambda i: (float(route[i]['x'])-px)**2 + (float(route[i]['y'])-py)**2)
            prox = float(self.route_data.get('settings', {}).get('proximity', 0.2))
            d = math.hypot(px - float(route[best_i]['x']), py - float(route[best_i]['y']))
            # Do not go backwards: honor previously progressed resume_index
            resume_index = int(self.current_goal_info.get('resume_index', 0))
            candidate = min(best_i + 1, len(route) - 1) if d <= prox else best_i
            start_i = max(candidate, resume_index)
            subpath = route[start_i:]
            if len(subpath) >= 2:
                self.is_moving = True
                self._execute_through_poses(subpath)
                # Update progress index so subsequent resumes don't jump back
                self.current_goal_info['resume_index'] = start_i
                return True
        except Exception as e:
            self.log_message(f"Resume from progress failed: {e}")
        return False
    
    def _create_pose_stamped(self, pose_dict):
        p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = float(pose_dict['x']); p.pose.position.y = float(pose_dict['y'])
        yaw_rad = math.radians(float(pose_dict.get('yaw_deg', 0.0))); o = euler_to_quaternion(0, 0, yaw_rad)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = o
        return p
    def _plan_cb(self, msg: Path): self.nav2_path = [{'x': p.pose.position.x, 'y': p.pose.position.y} for p in msg.poses]
    def _rosout_cb(self, msg: Log):
        if msg.name in self.log_node_filter and msg.level >= Log.INFO[0]: self.log_messages.append(f'[{msg.name}] {msg.msg}')
    def log_message(self, msg: str, **kwargs): self.get_logger().info(msg); self.log_messages.append(f'[dashboard] {msg}')
def euler_from_quaternion(q): x, y, z, w = q; _, _, yaw = euler_from_quaternion_explicit(x, y, z, w); return _, _, yaw
def euler_to_quaternion(r,p,y):cy=math.cos(y*0.5);sy=math.sin(y*0.5);cp=math.cos(p*0.5);sp=math.sin(p*0.5);cr=math.cos(r*0.5);sr=math.sin(r*0.5);return[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
def euler_from_quaternion_explicit(x,y,z,w):t0=+2.0*(w*x+y*z);t1=+1.0-2.0*(x*x+y*y);rx=math.atan2(t0,t1);t2=+2.0*(w*y-z*x);t2=+1.0 if t2>+1.0 else -1.0 if t2<-1.0 else t2;py=math.asin(t2);t3=+2.0*(w*z+x*y);t4=+1.0-2.0*(y*y+z*z);yz=math.atan2(t3,t4);return rx,py,yz
app = Flask(__name__, static_url_path="/static"); node = None
@app.route('/')
def index(): return send_from_directory('.', 'index.html')
@app.route('/api/status')
def status(): return jsonify({'pose': node.pose, 'mission_mode': node.mission_mode, 'is_moving': node.is_moving, 'is_paused': node.is_paused, 'is_recording': node.is_recording, 'nav2_path': node.nav2_path, 'logs': list(node.log_messages), 'route_data': node.route_data, 'recorded_path': node.recorded_path})
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

# Optional export/import routes for convenience
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
def ros_spin(): rclpy.spin(node)
def main():
    global node; rclpy.init(); node = NavCommander(); threading.Thread(target=ros_spin, daemon=True).start()
    log = logging.getLogger('werkzeug'); log.setLevel(logging.ERROR)
    print("Serving dashboard on http://0.0.0.0:8090")
    app.run(host='0.0.0.0', port=8090, debug=False, threaded=True)
if __name__ == '__main__': main()