#!/usr/bin/env python3
"""
cv_hand_control.py
==================
ROS2 Humble node for controlling the Braccio robotic arm using computer vision.
Uses MediaPipe to track hand gestures from the webcam.

- Left Hand: Controls which joint to move based on the number of fingers held up.
- Right Hand: Controls the angle of the selected joint based on its vertical position.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import cv2
import mediapipe as mp
import threading

class CVHandControlNode(Node):
    def __init__(self):
        super().__init__('cv_hand_control')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Joint names matching the URDF
        self.joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]
        
        # Joint limits as defined in the URDF (min, max) in radians
        self.joint_limits = {
            'base_joint': (0.0, 3.1416),
            'shoulder_joint': (0.2618, 2.8798),
            'elbow_joint': (0.0, 3.1416),
            'wrist_pitch_joint': (0.0, 3.1416),
            'wrist_roll_joint': (0.0, 3.1416),
            'gripper_joint': (0.1750, 1.2741)
        }
        
        # Current joint positions (initialize to middle of their range)
        self.joint_positions = {
            name: (limits[0] + limits[1]) / 2.0 
            for name, limits in self.joint_limits.items()
        }
        
        # Setup MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=2
        )
        
        self.active_joint_idx = 0 # Default to base_joint
        self.running = True
        
        # Start vision processing in a separate thread so ROS can spin
        self.vision_thread = threading.Thread(target=self.process_video)
        self.vision_thread.start()
        
        # Publish timer (e.g., 20Hz)
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        self.get_logger().info('CV Hand Control Node started.')

    def count_fingers(self, hand_landmarks):
        """Returns the number of fingers held up (0-5)."""
        fingers = 0
        tips = [
            self.mp_hands.HandLandmark.THUMB_TIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]
        mcp = [
            self.mp_hands.HandLandmark.THUMB_IP, # Thumb uses IP instead of MCP for easier detection 
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]
        
        # Thumb: compare x coordinates (assuming palm facing camera)
        # Note: This is a simple heuristic and might need adjustment based on handedness
        if hand_landmarks.landmark[tips[0]].x > hand_landmarks.landmark[mcp[0]].x:
            fingers += 1
            
        # Other fingers: compare y coordinates
        for i in range(1, 5):
            if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[mcp[i]].y:
                fingers += 1
                
        return fingers

    def process_video(self):
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            return

        while self.running and cap.isOpened():
            success, image = cap.read()
            if not success:
                self.get_logger().warn("Ignoring empty camera frame.")
                continue

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            
            # Process the image and find hands
            image.flags.writeable = False
            results = self.hands.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            left_hand_detected = False
            right_hand_detected = False
            
            if results.multi_hand_landmarks and results.multi_handedness:
                for idx, hand_handedness in enumerate(results.multi_handedness):
                    hand_label = hand_handedness.classification[0].label # "Left" or "Right"
                    hand_landmarks = results.multi_hand_landmarks[idx]
                    
                    self.mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )
                    
                    # Process Left Hand for Joint Selection
                    if hand_label == "Left":
                        left_hand_detected = True
                        fingers = self.count_fingers(hand_landmarks)
                        
                        # Map fingers to joint index (0-5)
                        if fingers == 1:
                            self.active_joint_idx = 0 # base
                        elif fingers == 2:
                            self.active_joint_idx = 1 # shoulder
                        elif fingers == 3:
                            self.active_joint_idx = 2 # elbow
                        elif fingers == 4:
                            self.active_joint_idx = 3 # wrist pitch
                        elif fingers == 5:
                            self.active_joint_idx = 4 # wrist roll
                        elif fingers == 0:
                            self.active_joint_idx = 5 # gripper
                                    
                        # Process Right Hand for Angle Control
                    elif hand_label == "Right":
                        right_hand_detected = True
                        
                        # Count fingers to determine if clutching (strictly 0 fingers = grabbing/fist)
                        right_fingers = self.count_fingers(hand_landmarks)
                        is_grabbing = (right_fingers == 0)
                        
                        # Use the y position of the wrist or middle finger mcp to control angle
                        # mapped between 0.0 (top) and 1.0 (bottom)
                        y_pos = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
                        
                        # INCREASED SENSITIVITY: Narrower control band so smaller up/down movements
                        # cover the whole joint range. 
                        # Constrain y_pos between 0.3 and 0.7 instead of 0.1 and 0.9
                        lower_bound = 0.3
                        upper_bound = 0.7
                        y_pos = max(lower_bound, min(upper_bound, y_pos))
                        # Normalize to 0.0 - 1.0
                        norm_y = (y_pos - lower_bound) / (upper_bound - lower_bound)
                        
                        # Map normalized y to joint limits (invert so up is max angle, down is min angle)
                        joint_name = self.joint_names[self.active_joint_idx]
                        min_limit, max_limit = self.joint_limits[joint_name]
                        target_angle = min_limit + (1.0 - norm_y) * (max_limit - min_limit)
                        
                        # Apply low-pass filter for smooth movement ONLY IF GRABBING
                        current_angle = self.joint_positions[joint_name]
                        if is_grabbing:
                            self.joint_positions[joint_name] = current_angle + 0.4 * (target_angle - current_angle)
                        
                        # Professional Visual feedback for right hand level
                        h, w, c = image.shape
                        y_pixel = int(y_pos * h)
                        
                        # Dynamic HUD Target Crosshair
                        crosshair_size = 20
                        # Color changes based on grab state
                        color_xhair = (0, 0, 255) if is_grabbing else (0, 255, 100)
                        cv2.line(image, (w//2 - crosshair_size, y_pixel), (w//2 - crosshair_size + 10, y_pixel), color_xhair, 2)
                        cv2.line(image, (w//2 + crosshair_size, y_pixel), (w//2 + crosshair_size - 10, y_pixel), color_xhair, 2)
                        cv2.line(image, (w//2, y_pixel - crosshair_size), (w//2, y_pixel - crosshair_size + 10), color_xhair, 2)
                        cv2.line(image, (w//2, y_pixel + crosshair_size), (w//2, y_pixel + crosshair_size - 10), color_xhair, 2)
                        cv2.circle(image, (w//2, y_pixel), 3, color_xhair, -1)
                        if not is_grabbing:
                            cv2.putText(image, "IDLE", (w//2 + 25, y_pixel - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_xhair, 1)
                        else:
                            cv2.putText(image, "LOCKED", (w//2 + 25, y_pixel - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_xhair, 2)
                        
                        # High-Tech Vertical Slider Bar
                        bar_x1 = w - 45
                        bar_x2 = w - 25
                        bar_y_top = int(lower_bound * h)
                        bar_y_bot = int(upper_bound * h)
                        
                        # Slider background
                        overlay_slider = image.copy()
                        cv2.rectangle(overlay_slider, (bar_x1, bar_y_top), (bar_x2, bar_y_bot), (20, 20, 20), -1)
                        cv2.addWeighted(overlay_slider, 0.7, image, 0.3, 0, image)
                        
                        # Slider fill (dynamic color based on position)
                        fill_color = (0, 255, 0) if norm_y > 0.5 else (0, 165, 255)
                        if norm_y < 0.15 or norm_y > 0.85: fill_color = (0, 0, 255)
                        cv2.rectangle(image, (bar_x1, y_pixel), (bar_x2, bar_y_bot), fill_color, -1)
                        cv2.rectangle(image, (bar_x1, bar_y_top), (bar_x2, bar_y_bot), (200, 200, 200), 1)

            # --- Global HUD Overlay ---
            overlay_hud = image.copy()
            h, w, c = image.shape
            
            # Corner Targeting Brackets
            cl = 30
            th = 2
            c_color = (200, 200, 200)
            cv2.line(overlay_hud, (20, 20), (20 + cl, 20), c_color, th)
            cv2.line(overlay_hud, (20, 20), (20, 20 + cl), c_color, th)
            cv2.line(overlay_hud, (w - 20, 20), (w - 20 - cl, 20), c_color, th)
            cv2.line(overlay_hud, (w - 20, 20), (w - 20, 20 + cl), c_color, th)
            cv2.line(overlay_hud, (20, h - 20), (20 + cl, h - 20), c_color, th)
            cv2.line(overlay_hud, (20, h - 20), (20, h - 20 - cl), c_color, th)
            cv2.line(overlay_hud, (w - 20, h - 20), (w - 20 - cl, h - 20), c_color, th)
            cv2.line(overlay_hud, (w - 20, h - 20), (w - 20, h - 20 - cl), c_color, th)
            
            # Bottom Header Bar
            cv2.rectangle(overlay_hud, (0, h - 45), (w, h), (10, 10, 10), -1)
            cv2.addWeighted(overlay_hud, 0.6, image, 0.4, 0, image)
            
            # System Status Indicator
            timer_msec = self.get_clock().now().nanoseconds / 1e6
            if int(timer_msec / 500) % 2 == 0:
                cv2.circle(image, (35, 35), 5, (0, 0, 255), -1)
            cv2.putText(image, "SYS: LIVE", (45, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Draw Joint Telemetry List (Left Side)
            cv2.putText(image, "[ JOINT TELEMETRY ]", (25, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
            y_offset = 95
            for i, j_name in enumerate(self.joint_names):
                j_val = self.joint_positions[j_name]
                j_min, j_max = self.joint_limits[j_name]
                if j_max > j_min:
                    j_norm = (j_val - j_min) / (j_max - j_min)
                else:
                    j_norm = 0
                
                is_active = (i == self.active_joint_idx)
                text_color = (0, 255, 0) if is_active else (120, 120, 120)
                prefix = ">" if is_active else " "
                
                disp_name = j_name.replace('_joint', '').upper()
                cv2.putText(image, f"{prefix} {disp_name[:5]}: {j_val:.2f}", (25, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)
                
                bar_pw = 60
                bar_px = 120
                cv2.rectangle(image, (bar_px, y_offset - 8), (bar_px + bar_pw, y_offset + 2), (40, 40, 40), -1)
                cv2.rectangle(image, (bar_px, y_offset - 8), (bar_px + int(bar_pw * j_norm), y_offset + 2), text_color, -1)
                
                y_offset += 20
          
            # Bottom Banner Status Text
            cv2.putText(image, "L-HAND ID:", (25, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            if left_hand_detected:
                joint_name = self.joint_names[self.active_joint_idx].replace('_', ' ').upper()
                cv2.putText(image, f"ACQUIRED [{joint_name}]", (105, h - 18), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255), 1)
            else:
                cv2.putText(image, "SEARCHING...", (105, h - 18), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1)
                
            rx_offset = w // 2 + 30
            cv2.putText(image, "R-HAND VAL:", (rx_offset, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            if right_hand_detected:
                joint_name = self.joint_names[self.active_joint_idx]
                val = self.joint_positions[joint_name]
                cv2.putText(image, f"TRACKING [{val:.3f}]", (rx_offset + 95, h - 18), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0), 1)
            else:
                cv2.putText(image, "NO TARGET", (rx_offset + 95, h - 18), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255), 1)

            cv2.imshow('Braccio Advanced Kinematics HUD', image)
            if cv2.waitKey(5) & 0xFF == 27:
                self.running = False
                break
                
        cap.release()
        cv2.destroyAllWindows()
        # Ensure ROS node exits when OpenCV window is closed
        rclpy.shutdown()

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # The joint_state_republisher also needs sub_gripper_joint since the robot state publisher expects it
        msg.name = self.joint_names + ['sub_gripper_joint']
        
        positions = []
        for name in self.joint_names:
            positions.append(self.joint_positions[name])
            
        # The sub_gripper_joint is a mimic joint for the gripper_joint in gazebo, 
        # but displaying it in RViz might need it explicitly published 
        # if the joint_state_publisher isn't echoing the mimic.
        # It typically mirrors the primary gripper joint.
        positions.append(self.joint_positions['gripper_joint'])
        
        msg.position = positions
        
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.vision_thread.is_alive():
            self.vision_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Professional Terminal Output
    print("\n" + "="*60)
    print("      __ )  _ \    \    __ | __| _)  _ \  ")
    print("      __ \    /   _ \  (     |    | (   | ")
    print("      ___/ _|_\ _/  _\ \___|\__|_|  \___/ ")
    print("="*60)
    print("\n[INFO] Starting MediaPipe CV Control Node...")
    print("[INFO] AI Hand Tracking Initialized.")
    print("       - Left Hand: Select Joint (Fingers 0-5)")
    print("       - Right Hand: Control Angle (Vertical Position)\n")
    print("  Author: Dilip Kumar")
    print("\nPress 'ESC' in the video window or Ctrl+C here to exit.\n")
    
    cv_node = CVHandControlNode()
    
    try:
        rclpy.spin(cv_node)
    except KeyboardInterrupt:
        print("\n\n[INFO] Shutting down CV Control Node gracefully...")
    finally:
        # Avoid destroying repeatedly if already handled
        if rclpy.ok():
            cv_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
