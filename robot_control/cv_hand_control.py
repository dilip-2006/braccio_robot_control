#!/usr/bin/env python3
"""
cv_hand_control.py
==================
ROS2 Humble node for controlling the Braccio robotic arm using computer vision.
Uses MediaPipe to track hand gestures from the webcam.

BRACCIO ADVANCED KINEMATICS HUD  —  Professional Edition
---------------------------------------------------------
- Left Hand : Select joint  (finger count  0-5)
- Right Hand: Control angle (vertical position + fist to lock)

Author: Dilip Kumar
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import cv2
import mediapipe as mp
import numpy as np
import threading

# ─── Colour Palette ────────────────────────────────────────────────────────────
C_BG        = (8,   12,  20)     # near-black panel background
C_ACCENT    = (0,   210, 255)    # cyan accent
C_GREEN     = (0,   230, 120)    # healthy / active
C_ORANGE    = (0,   165, 255)    # mid-range (OpenCV BGR)
C_RED       = (30,  30,  220)    # danger / searching
C_WHITE     = (230, 235, 240)
C_DIM       = (80,  90,  100)
C_GOLD      = (0,   200, 220)
C_PANEL     = (15,  22,  35)

# Joint display metadata
JOINT_META = [
    {"short": "BASE",      "icon": "⬤", "color": (0, 210, 255)},
    {"short": "SHOULDER",  "icon": "⬤", "color": (0, 230, 120)},
    {"short": "ELBOW",     "icon": "⬤", "color": (80, 180, 255)},
    {"short": "W.PITCH",   "icon": "⬤", "color": (0, 165, 255)},
    {"short": "W.ROLL",    "icon": "⬤", "color": (160, 80, 255)},
    {"short": "GRIPPER",   "icon": "⬤", "color": (0, 220, 180)},
]


# ─── Helper drawing utilities ──────────────────────────────────────────────────

def blend_rect(img, x1, y1, x2, y2, color, alpha=0.55):
    """Draw a semi-transparent filled rectangle."""
    overlay = img.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_corner_brackets(img, x1, y1, x2, y2, color, length=22, thickness=2, pulse=1.0):
    """Draw animated corner brackets around a rectangle."""
    l = int(length * pulse)
    pts = [
        # TL
        ((x1, y1), (x1 + l, y1)), ((x1, y1), (x1, y1 + l)),
        # TR
        ((x2, y1), (x2 - l, y1)), ((x2, y1), (x2, y1 + l)),
        # BL
        ((x1, y2), (x1 + l, y2)), ((x1, y2), (x1, y2 - l)),
        # BR
        ((x2, y2), (x2 - l, y2)), ((x2, y2), (x2, y2 - l)),
    ]
    for p1, p2 in pts:
        cv2.line(img, p1, p2, color, thickness, cv2.LINE_AA)


def draw_arc_gauge(img, cx, cy, radius, value, vmin, vmax, color, thickness=4, bg_color=None):
    """Draw a circular arc gauge from -210° to +30° (240° sweep)."""
    start_a = 150          # degrees, OpenCV convention
    sweep   = 240
    if bg_color:
        cv2.ellipse(img, (cx, cy), (radius, radius), 0,
                    start_a, start_a + sweep, bg_color, thickness, cv2.LINE_AA)
    if vmax > vmin:
        fraction = max(0.0, min(1.0, (value - vmin) / (vmax - vmin)))
    else:
        fraction = 0.0
    end_a = start_a + fraction * sweep
    if fraction > 0:
        cv2.ellipse(img, (cx, cy), (radius, radius), 0,
                    start_a, end_a, color, thickness, cv2.LINE_AA)
    # Needle dot
    needle_angle = math.radians(end_a)
    nx = int(cx + (radius - 2) * math.cos(needle_angle))
    ny = int(cy + (radius - 2) * math.sin(needle_angle))
    cv2.circle(img, (nx, ny), 4, color, -1, cv2.LINE_AA)


def put_text_centered(img, text, cx, cy, font, scale, color, thickness=1):
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    cv2.putText(img, text, (cx - tw // 2, cy + th // 2),
                font, scale, color, thickness, cv2.LINE_AA)


def put_text(img, text, x, y, font=cv2.FONT_HERSHEY_SIMPLEX,
             scale=0.4, color=C_WHITE, thickness=1):
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def draw_scan_line(img, t, color=(0, 210, 255), alpha=0.08):
    """Scrolling horizontal scan line."""
    h, w = img.shape[:2]
    y = int((t * 60) % h)
    overlay = img.copy()
    cv2.line(overlay, (0, y), (w, y), color, 1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


# ─── Main Node ─────────────────────────────────────────────────────────────────

class CVHandControlNode(Node):
    def __init__(self):
        super().__init__('cv_hand_control')

        # Publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_names = [
            'base_joint', 'shoulder_joint', 'elbow_joint',
            'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint'
        ]

        self.joint_limits = {
            'base_joint':        (0.0,    3.1416),
            'shoulder_joint':    (0.2618, 2.8798),
            'elbow_joint':       (0.0,    3.1416),
            'wrist_pitch_joint': (0.0,    3.1416),
            'wrist_roll_joint':  (0.0,    3.1416),
            'gripper_joint':     (0.1750, 1.2741),
        }

        self.joint_positions = {
            name: (lim[0] + lim[1]) / 2.0
            for name, lim in self.joint_limits.items()
        }

        # MediaPipe
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=2
        )

        self.active_joint_idx = 0
        self.running = True

        # Runtime state
        self._fps        = 0.0
        self._frame_t    = time.time()
        self._start_time = time.time()
        self._left_fingers  = 0
        self._right_mode    = "IDLE"    # IDLE | TRACKING | LOCKED
        self._target_angle  = {}        # smooth target per joint
        for name, lim in self.joint_limits.items():
            self._target_angle[name] = (lim[0] + lim[1]) / 2.0

        self.vision_thread = threading.Thread(target=self.process_video, daemon=True)
        self.vision_thread.start()

        self.timer = self.create_timer(0.05, self.publish_joint_states)
        self.get_logger().info('CV Hand Control Node started.')

    # ── Finger count ──────────────────────────────────────────────────────────
    def count_fingers(self, hand_landmarks):
        fingers = 0
        tips = [
            self.mp_hands.HandLandmark.THUMB_TIP,
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP,
        ]
        mcp = [
            self.mp_hands.HandLandmark.THUMB_IP,
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP,
        ]
        if hand_landmarks.landmark[tips[0]].x > hand_landmarks.landmark[mcp[0]].x:
            fingers += 1
        for i in range(1, 5):
            if hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[mcp[i]].y:
                fingers += 1
        return fingers

    # ── Main video loop ───────────────────────────────────────────────────────
    def process_video(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            return

        while self.running and cap.isOpened():
            success, raw = cap.read()
            if not success:
                continue

            # FPS
            now = time.time()
            self._fps = 1.0 / max(now - self._frame_t, 1e-6)
            self._frame_t = now

            # Pre-process
            image = cv2.cvtColor(cv2.flip(raw, 1), cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = self.hands.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            h, w = image.shape[:2]
            left_detected = right_detected = False
            norm_y_val = 0.5

            if results.multi_hand_landmarks and results.multi_handedness:
                for idx, handedness in enumerate(results.multi_handedness):
                    label = handedness.classification[0].label
                    lm    = results.multi_hand_landmarks[idx]

                    # Draw landmarks with custom style
                    self.mp_drawing.draw_landmarks(
                        image, lm,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )

                    if label == "Left":
                        left_detected = True
                        fingers = self.count_fingers(lm)
                        self._left_fingers = fingers
                        joint_map = {0: 5, 1: 0, 2: 1, 3: 2, 4: 3, 5: 4}
                        self.active_joint_idx = joint_map.get(fingers, 0)

                    elif label == "Right":
                        right_detected = True
                        rf = self.count_fingers(lm)
                        is_fist = (rf == 0)

                        y_pos = lm.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y
                        lower, upper = 0.3, 0.7
                        y_pos = max(lower, min(upper, y_pos))
                        norm_y_val = (y_pos - lower) / (upper - lower)

                        jname = self.joint_names[self.active_joint_idx]
                        mn, mx = self.joint_limits[jname]
                        target = mn + (1.0 - norm_y_val) * (mx - mn)

                        if is_fist:
                            self._right_mode = "LOCKED"
                            cur = self.joint_positions[jname]
                            self.joint_positions[jname] = cur + 0.35 * (target - cur)
                        else:
                            self._right_mode = "TRACKING"

            if not right_detected:
                self._right_mode = "IDLE"
            if not left_detected:
                self._left_fingers = 0

            # ── Draw HUD ──────────────────────────────────────────────────────
            self._draw_hud(image, h, w, left_detected, right_detected, norm_y_val, now)

            cv2.imshow('BRACCIO  |  Advanced Kinematics HUD', image)
            if cv2.waitKey(5) & 0xFF == 27:
                self.running = False
                break

        cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

    # ── HUD Renderer ─────────────────────────────────────────────────────────
    def _draw_hud(self, img, h, w, left_det, right_det, norm_y, t):
        elapsed = t - self._start_time
        pulse   = 0.85 + 0.15 * math.sin(elapsed * 3.0)   # 0.85–1.0 pulsing factor

        # ── 1. Scanline ──────────────────────────────────────────────────────
        draw_scan_line(img, elapsed)

        # ── 2. Top header bar ────────────────────────────────────────────────
        blend_rect(img, 0, 0, w, 52, C_PANEL, alpha=0.78)
        cv2.line(img, (0, 52), (w, 52), C_ACCENT, 1, cv2.LINE_AA)

        # Title
        put_text(img, "BRACCIO", 14, 20,
                 cv2.FONT_HERSHEY_DUPLEX, 0.65, C_ACCENT, 1)
        put_text(img, "ADVANCED KINEMATICS HUD", 100, 20,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_WHITE, 1)
        put_text(img, "v2.0  |  DILIP KUMAR", 100, 40,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.32, C_DIM, 1)

        # FPS counter (top-right)
        fps_color = C_GREEN if self._fps > 24 else C_ORANGE
        put_text(img, f"FPS  {self._fps:4.1f}", w - 110, 22,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.42, fps_color, 1)

        # System blink indicator
        blink_on = int(elapsed * 2) % 2 == 0
        dot_col  = C_GREEN if blink_on else C_DIM
        cv2.circle(img, (w - 22, 16), 5, dot_col, -1, cv2.LINE_AA)
        put_text(img, "SYS LIVE" if blink_on else "SYS LIVE",
                 w - 118, 42, cv2.FONT_HERSHEY_SIMPLEX, 0.30, dot_col, 1)

        # Uptime
        m, s = divmod(int(elapsed), 60)
        put_text(img, f"UP  {m:02d}:{s:02d}", w - 110, 42,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.30, C_DIM, 1)

        # ── 3. Corner brackets (full-frame) ──────────────────────────────────
        draw_corner_brackets(img, 6, 6, w - 6, h - 6,
                             C_ACCENT, length=28, thickness=2, pulse=pulse)

        # ── 4. Left panel  — Joint Telemetry ──────────────────────────────────
        panel_x1, panel_y1 = 10, 60
        panel_x2, panel_y2 = 210, 60 + 6 * 52 + 14
        blend_rect(img, panel_x1, panel_y1, panel_x2, panel_y2, C_PANEL, 0.70)
        cv2.rectangle(img, (panel_x1, panel_y1), (panel_x2, panel_y2),
                      C_DIM, 1, cv2.LINE_AA)
        draw_corner_brackets(img, panel_x1, panel_y1, panel_x2, panel_y2,
                             C_ACCENT, length=10, thickness=1)
        put_text(img, "[ JOINT TELEMETRY ]", panel_x1 + 8, panel_y1 + 14,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.36, C_ACCENT, 1)

        for i, jname in enumerate(self.joint_names):
            val  = self.joint_positions[jname]
            mn, mx = self.joint_limits[jname]
            meta = JOINT_META[i]
            jcol = meta["color"]
            is_active = (i == self.active_joint_idx)
            row_y = panel_y1 + 26 + i * 52

            # Row background highlight if active
            if is_active:
                blend_rect(img, panel_x1 + 2, row_y - 4,
                           panel_x2 - 2, row_y + 44, jcol, 0.10)
                cv2.rectangle(img, (panel_x1 + 2, row_y - 4),
                              (panel_x2 - 2, row_y + 44), jcol, 1, cv2.LINE_AA)

            act_col = jcol if is_active else C_DIM
            prefix  = "▶" if is_active else " "

            # Joint name + arrow
            deg = math.degrees(val)
            put_text(img, f"{prefix} {meta['short']}", panel_x1 + 8, row_y + 10,
                     cv2.FONT_HERSHEY_SIMPLEX, 0.38, act_col, 1)

            # Degree value (large)
            put_text(img, f"{deg:6.1f}°", panel_x1 + 108, row_y + 10,
                     cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_WHITE if is_active else C_DIM, 1)

            # Flat progress bar
            bar_x  = panel_x1 + 8
            bar_y  = row_y + 18
            bar_w  = panel_x2 - panel_x1 - 16
            bar_h  = 8
            norm   = (val - mn) / (mx - mn) if mx > mn else 0
            cv2.rectangle(img, (bar_x, bar_y),
                          (bar_x + bar_w, bar_y + bar_h), (30, 35, 45), -1)
            if norm > 0:
                cv2.rectangle(img, (bar_x, bar_y),
                              (bar_x + int(bar_w * norm), bar_y + bar_h),
                              act_col, -1, cv2.LINE_AA)
            cv2.rectangle(img, (bar_x, bar_y),
                          (bar_x + bar_w, bar_y + bar_h), C_DIM, 1)

            # Min / Max labels
            mn_deg = math.degrees(mn)
            mx_deg = math.degrees(mx)
            put_text(img, f"{mn_deg:.0f}", bar_x, row_y + 40,
                     cv2.FONT_HERSHEY_SIMPLEX, 0.28, C_DIM, 1)
            put_text(img, f"{mx_deg:.0f}°", bar_x + bar_w - 28, row_y + 40,
                     cv2.FONT_HERSHEY_SIMPLEX, 0.28, C_DIM, 1)

        # ── 5. Right panel  — Arc gauge for active joint ─────────────────────
        rp_w   = 160
        rp_x1  = w - rp_w - 10
        rp_y1  = 60
        rp_x2  = w - 10
        rp_y2  = rp_y1 + 200
        blend_rect(img, rp_x1, rp_y1, rp_x2, rp_y2, C_PANEL, 0.72)
        cv2.rectangle(img, (rp_x1, rp_y1), (rp_x2, rp_y2), C_DIM, 1, cv2.LINE_AA)
        draw_corner_brackets(img, rp_x1, rp_y1, rp_x2, rp_y2,
                             C_ACCENT, length=10, thickness=1)

        jname  = self.joint_names[self.active_joint_idx]
        jval   = self.joint_positions[jname]
        mn, mx = self.joint_limits[jname]
        jcol_a = JOINT_META[self.active_joint_idx]["color"]

        put_text(img, "ACTIVE JOINT", rp_x1 + 14, rp_y1 + 16,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.35, C_ACCENT, 1)
        put_text(img, JOINT_META[self.active_joint_idx]["short"],
                 rp_x1 + 14, rp_y1 + 34,
                 cv2.FONT_HERSHEY_DUPLEX, 0.55, jcol_a, 1)

        # Arc gauge
        gcx = rp_x1 + rp_w // 2
        gcy = rp_y1 + 120
        draw_arc_gauge(img, gcx, gcy, 55, jval, mn, mx,
                       jcol_a, thickness=6, bg_color=(35, 40, 50))
        # Center value
        deg_val = math.degrees(jval)
        put_text_centered(img, f"{deg_val:.1f}", gcx, gcy,
                          cv2.FONT_HERSHEY_DUPLEX, 0.60, C_WHITE, 1)
        put_text_centered(img, "deg", gcx, gcy + 18,
                          cv2.FONT_HERSHEY_SIMPLEX, 0.30, C_DIM, 1)

        # Range labels
        put_text(img, f"{math.degrees(mn):.0f}°", rp_x1 + 8, rp_y1 + 178,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.30, C_DIM, 1)
        put_text(img, f"{math.degrees(mx):.0f}°", rp_x2 - 36, rp_y1 + 178,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.30, C_DIM, 1)

        # ── 6. Right-side vertical slider (hand position) ────────────────────
        if right_det:
            sb_x1 = w - 30
            sb_x2 = w - 14
            sb_yt  = int(0.30 * h)
            sb_yb  = int(0.70 * h)
            sb_yp  = int(sb_yt + norm_y * (sb_yb - sb_yt))

            blend_rect(img, sb_x1, sb_yt, sb_x2, sb_yb, (20, 25, 35), 0.75)
            mode = self._right_mode
            scol = C_RED if mode == "LOCKED" else C_GREEN
            cv2.rectangle(img, (sb_x1, sb_yp), (sb_x2, sb_yb), scol, -1, cv2.LINE_AA)
            cv2.rectangle(img, (sb_x1, sb_yt), (sb_x2, sb_yb), C_DIM, 1)
            # Notch
            cv2.line(img, (sb_x1 - 4, sb_yp), (sb_x2 + 4, sb_yp), C_WHITE, 2, cv2.LINE_AA)

        # ── 7. Mode indicator (center-top below header) ──────────────────────
        mode_str = self._right_mode
        mode_colors = {"IDLE": C_DIM, "TRACKING": C_GREEN, "LOCKED": C_RED}
        m_col = mode_colors.get(mode_str, C_DIM)
        mx_mode = w // 2
        blend_rect(img, mx_mode - 65, 58, mx_mode + 65, 82, C_PANEL, 0.75)
        cv2.rectangle(img, (mx_mode - 65, 58), (mx_mode + 65, 82), m_col, 1, cv2.LINE_AA)
        put_text_centered(img, f"● {mode_str}", mx_mode, 73,
                          cv2.FONT_HERSHEY_SIMPLEX, 0.40, m_col, 1)

        # ── 8. Bottom footer bar ─────────────────────────────────────────────
        blend_rect(img, 0, h - 50, w, h, C_PANEL, 0.78)
        cv2.line(img, (0, h - 50), (w, h - 50), C_ACCENT, 1, cv2.LINE_AA)

        # Left hand status
        l_col  = C_ACCENT if left_det else C_RED
        l_text = (f"L ◀ {JOINT_META[self.active_joint_idx]['short']}  "
                  f"[{self._left_fingers} fingers]") if left_det else "L ◀ SEARCHING..."
        put_text(img, l_text, 14, h - 28,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.40, l_col, 1)

        # Right hand status
        r_col  = C_GREEN if right_det else C_RED
        r_val  = math.degrees(self.joint_positions[self.joint_names[self.active_joint_idx]])
        r_text = (f"R ▶ {r_val:.1f}°  [{mode_str}]") if right_det else "R ▶ NO TARGET"
        r_tw, _ = cv2.getTextSize(r_text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
        put_text(img, r_text, w - r_tw[0] - 14, h - 28,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.40, r_col, 1)

        # Bottom center: gesture guide
        guide = "L-HAND: fingers=joint  |  R-HAND: height=angle  |  FIST=lock"
        gt, _ = cv2.getTextSize(guide, cv2.FONT_HERSHEY_SIMPLEX, 0.30, 1)
        put_text(img, guide, w // 2 - gt[0] // 2, h - 10,
                 cv2.FONT_HERSHEY_SIMPLEX, 0.30, C_DIM, 1)

        # ── 9. Crosshair on right hand position ─────────────────────────────
        if right_det:
            cy_px = int(0.30 * h + norm_y * 0.40 * h)
            xh_x  = w // 2
            col_x = C_RED if self._right_mode == "LOCKED" else C_GREEN
            cs = 18
            cv2.line(img, (xh_x - cs, cy_px), (xh_x - 6, cy_px), col_x, 1, cv2.LINE_AA)
            cv2.line(img, (xh_x + 6, cy_px), (xh_x + cs, cy_px), col_x, 1, cv2.LINE_AA)
            cv2.line(img, (xh_x, cy_px - cs), (xh_x, cy_px - 6), col_x, 1, cv2.LINE_AA)
            cv2.line(img, (xh_x, cy_px + 6), (xh_x, cy_px + cs), col_x, 1, cv2.LINE_AA)
            cv2.circle(img, (xh_x, cy_px), 3, col_x, -1, cv2.LINE_AA)
            put_text(img, self._right_mode, xh_x + 24, cy_px - 6,
                     cv2.FONT_HERSHEY_SIMPLEX, 0.35, col_x, 1)

    # ── Publisher ─────────────────────────────────────────────────────────────
    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names + ['sub_gripper_joint']
        positions = [self.joint_positions[n] for n in self.joint_names]
        positions.append(self.joint_positions['gripper_joint'])
        msg.position = positions
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.vision_thread.is_alive():
            self.vision_thread.join(timeout=2.0)
        super().destroy_node()


# ─── Entry Point ───────────────────────────────────────────────────────────────
def main(args=None):
    print("\n" + "=" * 62)
    print("  ____                      _")
    print(" | __ ) _ __ __ _  ___ ___ (_) ___")
    print(" |  _ \\| '__/ _` |/ __/ __|| |/ _ \\")
    print(" | |_) | | | (_| | (_| (__ | | (_) |")
    print(" |____/|_|  \\__,_|\\___\\___|_|\\___/")
    print("=" * 62)
    print("  ADVANCED KINEMATICS HUD  —  Professional Edition")
    print("=" * 62)
    print("\n[INFO] Starting MediaPipe CV Control Node...")
    print("[INFO] AI Dual-Hand Tracking  →  Initialized")
    print("")
    print("  LEFT  HAND : Finger count selects the active joint")
    print("               0=Gripper  1=Base  2=Shoulder  3=Elbow")
    print("               4=W.Pitch  5=W.Roll")
    print("")
    print("  RIGHT HAND : Vertical position sets the angle")
    print("               Make a FIST to lock & apply the angle")
    print("")
    print("  ESC  in the video window  or  Ctrl+C  to exit")
    print("  Author: Dilip Kumar")
    print("=" * 62 + "\n")

    rclpy.init(args=args)
    node = CVHandControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down gracefully...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
