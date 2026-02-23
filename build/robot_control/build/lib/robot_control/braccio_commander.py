#!/usr/bin/env python3
"""
braccio_commander.py
====================
ROS2 Humble port of target_object_sim.py (originally ROS1 / Python 2).

Provides an interactive CLI to operate the Braccio arm in Gazebo via MoveIt2:
  c = calibrate  – scan points and derive the gazebo→robot homography
  l = load       – load calibration.json
  t = target     – pick red box and drop in bowl
  m = manual     – enter x/y coordinates manually
  b = bowl       – run pre-programmed bowl drop sequence
  r = reset      – reset block and bowl positions
  e = evaluate   – run N automated trials and save stats
  u = up         – go to neutral up pose
  q = quit

Usage:
  ros2 run robot_control braccio_commander
"""

import sys
import json
import time
import threading

import numpy as np
import scipy.optimize

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print('[WARN] cv2 not found – calibration/transform disabled')

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_PY_AVAILABLE = True
except ImportError:
    MOVEIT_PY_AVAILABLE = False
    print('[WARN] moveit.planning not found – using trajectory action client fallback')

# ── Constants (same as original) ──────────────────────────────────────────────
THETA_EXT = 0.27
THETA_RET = np.pi / 4
L_FUDGE = 0.08

Z_MAX_SIDE = -0.03
Z_MAX_DOWN = 0.0
Z_MIN = -0.045
CLOSE_ENOUGH = 0.02
DEFAULT_ROT = 0.0

S_SIDE_MAX = 0.4
S_SIDE_MIN = 0.161
S_TOP_MAX = 0.29


# ── Geometry helpers ──────────────────────────────────────────────────────────
def cart2pol(x, y):
    """Cartesian → polar."""
    return np.sqrt(x**2 + y**2), np.arctan2(y, x)


def pol2cart(rho, phi):
    """Polar → Cartesian."""
    return rho * np.cos(phi), rho * np.sin(phi)


def get_other_angles(theta_shoulder):
    theta_wrist = theta_shoulder + np.pi / 2
    theta_elbow = np.pi / 2 - 2 * theta_shoulder
    return theta_wrist, theta_elbow


# ── Inverse kinematics (unchanged from original) ──────────────────────────────
class Arm3Link:
    """
    Simple IK solver for a shoulder-elbow-wrist robot arm.
    Credit: https://github.com/studywolf/blog/tree/master/InvKin
    """

    def __init__(self, L=None):
        self.q = [0.0, 0.0, 0.0]
        self.L = np.array([1.0, 1.0, 0.8]) if L is None else np.array(L)
        self.max_y = 1.0
        self.min_y = 0.0
        self.end_angle_tol = 0.05
        self.end_angle = -np.pi / 2
        self.max_angles = np.array([1.6, np.pi / 2, np.pi / 2])
        self.min_angles = np.array([0.27, -np.pi / 2, -np.pi / 2])

    def get_xy(self, q=None):
        if q is None:
            q = self.q
        x = (self.L[0] * np.cos(q[0]) +
             self.L[1] * np.cos(q[0] + q[1]) +
             self.L[2] * np.cos(np.sum(q)))
        y = (self.L[0] * np.sin(q[0]) +
             self.L[1] * np.sin(q[0] + q[1]) +
             self.L[2] * np.sin(np.sum(q)))
        return [x, y]

    def inv_kin(self, x, min_y, max_y, end_angle):
        def distance_to_default(q, x):
            xc = (self.L[0] * np.cos(q[0]) + self.L[1] * np.cos(q[0] + q[1]) +
                  self.L[2] * np.cos(np.sum(q))) - x
            return xc ** 2

        def y_upper(q, *_):
            y = (self.L[0] * np.sin(q[0]) + self.L[1] * np.sin(q[0] + q[1]) +
                 self.L[2] * np.sin(np.sum(q)))
            return self.max_y - y

        def y_lower(q, *_):
            y = (self.L[0] * np.sin(q[0]) + self.L[1] * np.sin(q[0] + q[1]) +
                 self.L[2] * np.sin(np.sum(q)))
            return y - self.min_y

        def jt_upper(q, *_):
            return self.max_angles - q

        def jt_lower(q, *_):
            return q - self.min_angles

        def orientation(q, *_):
            return self.end_angle_tol - abs(np.sum(q) - self.end_angle)

        self.min_y = min_y
        self.max_y = max_y
        if end_angle is not None:
            self.end_angle = end_angle

        q = scipy.optimize.fmin_slsqp(
            func=distance_to_default, x0=self.q, args=(x,), iprint=0,
            ieqcons=[orientation, jt_upper, jt_lower, y_upper, y_lower])
        self.q = q
        return self.q


# ── Motion interface ──────────────────────────────────────────────────────────
class BraccioMoveItInterface:
    """Thin wrapper around MoveItPy (or stub when unavailable)."""

    def __init__(self, node: Node):
        self._node = node
        if MOVEIT_PY_AVAILABLE:
            self._moveit = MoveItPy(node_name='braccio_commander')
            self._arm = self._moveit.get_planning_component('braccio_arm')
            self._gripper = self._moveit.get_planning_component('braccio_gripper')
        else:
            self._moveit = None
            node.get_logger().warn('MoveItPy unavailable – motion calls are no-ops')

    # ── joint motion ──────────────────────────────────────────────────────────
    def go_to_joint(self, j0=None, j1=None, j2=None, j3=None, j4=None):
        if not MOVEIT_PY_AVAILABLE:
            return
        with self._arm.get_planning_scene_monitor().read_write() as scene:
            state = scene.current_state
        cur = {
            'base_joint': state.get_joint_positions('base_joint')[0],
            'shoulder_joint': state.get_joint_positions('shoulder_joint')[0],
            'elbow_joint': state.get_joint_positions('elbow_joint')[0],
            'wrist_pitch_joint': state.get_joint_positions('wrist_pitch_joint')[0],
            'wrist_roll_joint': state.get_joint_positions('wrist_roll_joint')[0],
        }
        overrides = {}
        if j0 is not None: overrides['base_joint'] = float(j0)
        if j1 is not None: overrides['shoulder_joint'] = float(j1)
        if j2 is not None: overrides['elbow_joint'] = float(j2)
        if j3 is not None: overrides['wrist_pitch_joint'] = float(j3)
        if j4 is not None: overrides['wrist_roll_joint'] = float(j4)
        cur.update(overrides)
        joints = list(cur.values())
        self._arm.set_goal_state(joint_positions=joints)
        plan = self._arm.plan()
        if plan:
            self._moveit.execute(plan.trajectory, controllers=[])

    def go_gripper(self, val: float):
        if not MOVEIT_PY_AVAILABLE:
            return
        self._gripper.set_goal_state(
            joint_positions=[float(val), float(val)])
        plan = self._gripper.plan()
        if plan:
            self._moveit.execute(plan.trajectory, controllers=[])

    # ── named motions ─────────────────────────────────────────────────────────
    def gripper_open(self):   self.go_gripper(0.2)
    def gripper_close(self):  self.go_gripper(1.2)
    def gripper_middle(self): self.go_gripper(0.5)

    def go_to_raise(self):
        self.go_to_joint(j1=1.15, j2=0.13, j3=2.29)

    def go_to_up(self):
        self.go_to_joint(j0=1.5708, j1=1.5708, j2=1.5708, j3=1.5708)

    def go_to_home(self):
        self.go_to_raise()
        self.go_to_joint(j0=3.14)
        self.gripper_open()

    def go_to_bowl(self):
        self.go_to_up()
        self.gripper_open()
        self.go_to_joint(j0=0.45, j1=1.57, j2=3.14, j3=3.14)
        self.go_to_joint(j1=2.76, j2=2.82, j3=0.76)
        self.gripper_middle()
        self.go_to_joint(j1=2.87, j2=2.52, j3=0.83)
        self.go_to_joint(j1=2.5,  j2=2.52, j3=0.83)
        self.go_to_joint(j0=0.9)
        self.go_to_joint(j1=2.87, j2=2.52, j3=0.83)
        self.gripper_open()
        self.go_to_joint(j1=2.76, j2=2.82, j3=0.76)
        self.gripper_open()
        self.go_to_joint(j1=1.57, j2=3.14, j3=3.14)
        self.go_to_up()

    def go_to_pull(self, phi):
        self.go_to_raise()
        self.gripper_close()
        if phi is not None:
            self.go_to_joint(j0=float(phi))
        self.go_to_joint(j1=0.3, j2=1.8, j3=1.8)
        self.go_to_joint(j1=0.3, j2=1.8, j3=0.1)
        self.go_to_joint(j1=1.3, j2=0.4, j3=0.01)
        self.go_to_joint(j1=1.5, j2=0.4, j3=0.1)
        self.go_to_joint(j1=0.3, j2=1.8, j3=1.8)

    def go_to_push(self, phi):
        self.go_to_raise()
        self.gripper_close()
        if phi is not None:
            self.go_to_joint(j0=float(phi))
        self.go_to_joint(j1=2.7,  j2=0.01, j3=0.01)
        self.go_to_joint(j1=1.6,  j2=0.01, j3=0.01)
        self.go_to_joint(j1=0.3,  j2=1.8,  j3=0.1)
        self.go_to_joint(j1=2.1,  j2=0.01, j3=0.01)
        self.go_to_joint(j1=2.7,  j2=0.01, j3=0.01)

    def go_to_xy(self, kinematics: Arm3Link, x, y, how):
        if how == 'top':
            s, phi = cart2pol(x, y)
            q = kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -np.pi / 2)
            xy = kinematics.get_xy(q)
            joint_targets = [phi, q[0], q[1] + np.pi / 2, q[2] + np.pi / 2]
            if joint_targets[0] < 0 or joint_targets[0] > 3.14:
                print('Not in reachable area, aborting')
                return -1
            if abs(xy[0] - s) > CLOSE_ENOUGH:
                if s < S_TOP_MAX and s > S_SIDE_MIN:
                    print('Too far out, pulling backwards')
                    self.go_to_pull(joint_targets[0])
                    return 1
                print('No IK solution, aborting')
                return -1
        elif how == 'side':
            s, phi = cart2pol(x, y)
            q = kinematics.inv_kin(s, Z_MIN, Z_MAX_SIDE, 0)
            xy = kinematics.get_xy(q)
            joint_targets = [phi, q[0], q[1] + np.pi / 2, q[2] + np.pi / 2]
            if joint_targets[0] < 0 or joint_targets[0] > 3.14:
                print('Not in reachable area, aborting')
                return -1
            if abs(xy[0] - s) > CLOSE_ENOUGH:
                if s < S_SIDE_MAX and s > S_SIDE_MIN:
                    print('Too close, pushing backwards')
                    self.go_to_push(joint_targets[0])
                    return 1
                print('No IK solution, aborting')
                return -1

        self.go_to_raise()
        self.gripper_open()
        self.go_to_joint(j0=float(joint_targets[0]))
        self.go_to_joint(j1=float(joint_targets[1]),
                         j2=float(joint_targets[2]),
                         j3=float(joint_targets[3]))
        self.gripper_close()
        if how == 'top' and joint_targets[2] < 3:
            self.go_to_joint(j2=float(joint_targets[2]) + 0.1)
        self.go_to_home()
        return 0


# ── Main node ─────────────────────────────────────────────────────────────────
class BraccioCommanderNode(Node):

    def __init__(self):
        super().__init__('braccio_commander')

        self.linkstate_data = None
        self.homography = None
        self.kinematics = Arm3Link()
        self.L = None
        self.l = None

        self.motion = BraccioMoveItInterface(self)

        self._link_sub = self.create_subscription(
            LinkStates, '/gazebo/link_states',
            self._linkstate_cb, 10)

        self._set_state_cli = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        self.get_logger().info('BraccioCommander ready.')

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _linkstate_cb(self, msg: LinkStates):
        self.linkstate_data = msg

    # ── Gazebo helpers ────────────────────────────────────────────────────────
    def reset_link(self, name, x, y, z):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.w = 1.0
        if self._set_state_cli.wait_for_service(timeout_sec=2.0):
            future = self._set_state_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        else:
            self.get_logger().warn(f'set_entity_state service unavailable for {name}')

    def get_link_position(self, link_names):
        if self.linkstate_data is None:
            return 0.0, 0.0, DEFAULT_ROT
        xs, ys, n = 0.0, 0.0, 0
        for lname in link_names:
            try:
                idx = self.linkstate_data.name.index(lname)
                xs += self.linkstate_data.pose[idx].position.x
                ys += self.linkstate_data.pose[idx].position.y
                n += 1
            except ValueError:
                pass
        return (xs / n if n else 0.0), (ys / n if n else 0.0), DEFAULT_ROT

    # ── Calibration ──────────────────────────────────────────────────────────
    def calibrate(self):
        if not CV2_AVAILABLE:
            print('[ERROR] cv2 required for calibration')
            return
        src_pts, dst_angs = [], []
        mx, my, _ = self.get_link_position(['braccio::base_link'])
        src_pts.append([mx, my])
        self.motion.gripper_middle()
        N = 8
        phi_min, phi_max = np.pi / 6, np.pi - np.pi / 6
        for i in range(2, N):
            self.motion.go_to_raise()
            rand_phi = phi_min + i * (phi_max - phi_min) / N
            theta_shoulder = THETA_RET if i % 2 == 0 else THETA_EXT
            theta_wrist, theta_elbow = get_other_angles(theta_shoulder)
            rand_targ = [rand_phi, theta_shoulder, theta_elbow, theta_wrist]
            self.motion.go_to_joint(
                j0=rand_phi, j1=theta_shoulder, j2=theta_elbow, j3=theta_wrist)
            mx, my, _ = self.get_link_position(
                ['braccio::left_gripper_link', 'braccio::right_gripper_link'])
            src_pts.append([mx, my])
            dst_angs.append(rand_targ)
        with open('calibration.json', 'w') as f:
            json.dump({'src_pts': src_pts, 'dst_angs': dst_angs}, f)
        self.load_calibrate()
        self.motion.go_to_up()

    def load_calibrate(self):
        if not CV2_AVAILABLE:
            print('[ERROR] cv2 required for calibration loading')
            return
        try:
            with open('calibration.json', 'r') as f:
                calib = json.load(f)
            src_pts = calib['src_pts']
            dst_angs = calib['dst_angs']

            s_ret = src_pts[1::2]
            s_ext = src_pts[2::2]
            arr = np.array(s_ret) - np.array(s_ext)
            self.L = np.sqrt((arr * arr).sum(axis=1)).mean() / (
                np.cos(THETA_EXT) - np.cos(THETA_RET))
            arr1 = np.array(s_ret) - np.array(src_pts[0])
            l1 = np.sqrt((arr1 * arr1).sum(axis=1)).mean() - self.L * np.cos(THETA_RET)
            arr2 = np.array(s_ext) - np.array(src_pts[0])
            l2 = np.sqrt((arr2 * arr2).sum(axis=1)).mean() - self.L * np.cos(THETA_EXT)
            self.l = (l1 + l2) / 2

            dst_pts = [[0.0, 0.0]]
            for ang in dst_angs:
                phi = ang[0]
                rho = self.L * np.cos(ang[1]) + self.l
                x, y = pol2cart(rho, phi)
                dst_pts.append([x, y])

            src_arr = np.array(src_pts, dtype='float32')
            dst_arr = np.array(dst_pts, dtype='float32')
            h, _ = cv2.findHomography(src_arr, dst_arr)
            self.homography = h
            self.kinematics = Arm3Link(
                L=[self.L / 2, self.L / 2, self.l + L_FUDGE])
            print(f'Calibration loaded. l={self.l:.4f}  L={self.L:.4f}')
        except Exception as e:
            print(f'[WARN] Could not load calibration.json: {e}')

    def transform(self, x1, y1):
        if self.homography is None:
            raise ValueError('Run calibration first (c) or load it (l)')
        if not CV2_AVAILABLE:
            raise RuntimeError('cv2 unavailable')
        a = np.array([[x1, y1]], dtype='float32')
        res = cv2.perspectiveTransform(a[None, :, :], self.homography)[0][0]
        return float(res[0]), float(res[1]), DEFAULT_ROT

    def get_box_position(self):
        x, y, r = self.get_link_position(['unit_box_0::link'])
        return self.transform(x, y)

    # ── Pick-and-place helpers ────────────────────────────────────────────────
    def go_to_target(self, how):
        x, y, r = self.get_box_position()
        print(f'Box at x={x:.3f} y={y:.3f}')
        return self.motion.go_to_xy(self.kinematics, x, y, how)

    def go_to_manual(self, how):
        x = float(input('pos x? '))
        y = float(input('pos y? '))
        return self.motion.go_to_xy(self.kinematics, x, y, how)

    def reset_target_position(self):
        x = input('reset block x= ')
        y = input('reset block y= ')
        z = input('reset block z= ')
        self.reset_link('unit_box_0', x, y, z)
        self.reset_link('my_mesh', -0.15, -0.325, 0)

    def run_eval(self):
        N = int(input('How many trials? '))
        evl_data = []
        for i in range(N):
            print(f'Trial {i}')
            how = 'side' if np.random.uniform() < 0.5 else 'top'
            extent = 0.5 if how == 'side' else S_TOP_MAX
            x = np.random.uniform() * extent
            y = -extent + 2 * extent * np.random.uniform()
            self.reset_link('unit_box_0', x, y, 0)
            self.reset_link('my_mesh', -0.15, -0.325, 0)
            time.sleep(1)
            record = {'target': [x, y], 'how': how}
            state, results = 1, []
            for _ in range(3):
                x_, y_, _ = self.get_link_position(['unit_box_0::link'])
                results.append([x_, y_, state])
                if state < 1:
                    break
                state = self.go_to_target(how)
            if state == 0:
                self.motion.go_to_bowl()
                x_, y_, _ = self.get_link_position(['unit_box_0::link'])
                results.append([x_, y_, state])
            record['box_results'] = results
            bx, by, _ = self.get_link_position(['my_mesh::body'])
            record['bowl_result'] = [bx, by]
            evl_data.append(record)
            with open('eval_results.json', 'w') as f:
                json.dump(evl_data, f)
            print(f'  Saved to eval_results.json')


# ── CLI ──────────────────────────────────────────────────────────────────────
def print_instructions():
    print("""
==================== Braccio Commander (ROS2) ====================
  c = calibrate     – scan and generate gazebo→robot homography
  l = load cal      – load calibration.json
  t = target        – pick box (top/side) and drop in bowl
  m = manual        – enter x/y for pickup
  b = bowl          – run bowl drop sequence
  r = reset         – reposition block and bowl
  e = evaluate      – run N automated trials
  u = up            – go to neutral up pose
  q = quit
==================================================================
Command: """, end='', flush=True)


def ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = BraccioCommanderNode()

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    node.load_calibrate()
    print('\n== Arduino Braccio Pick+Drop Simulator (ROS2 Humble) ==\n')

    while True:
        print_instructions()
        inp = input().strip().lower()
        if inp == 'q':
            break
        elif inp == 'c':
            node.calibrate()
        elif inp == 'l':
            node.load_calibrate()
        elif inp == 't':
            how = input('top (t) or side (s)? ').strip()
            if how in ('t', 'top'):
                node.go_to_target('top')
            elif how in ('s', 'side'):
                node.go_to_target('side')
        elif inp == 'm':
            how = input('top (t) or side (s)? ').strip()
            if how in ('t', 'top'):
                node.go_to_manual('top')
            elif how in ('s', 'side'):
                node.go_to_manual('side')
        elif inp == 'b':
            node.motion.go_to_bowl()
        elif inp == 'r':
            node.reset_target_position()
        elif inp == 'e':
            node.run_eval()
        elif inp == 'u':
            node.motion.go_to_up()
        else:
            print(f'Unknown command: {inp!r}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
