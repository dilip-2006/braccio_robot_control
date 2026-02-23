<div align="center">

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=30&duration=3000&pause=1000&color=00D4FF&center=true&vCenter=true&width=700&lines=🦾+Braccio+Robotic+Arm+Control;ROS2+%2B+MoveIt2+%2B+Computer+Vision;Gesture-Powered+6DOF+Manipulation" alt="Typing SVG" />

<br/>

<p>
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?style=for-the-badge&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/MediaPipe-CV%20Control-brightgreen?style=for-the-badge&logo=google&logoColor=white" />
  <img src="https://img.shields.io/badge/MoveIt2-Motion%20Planning-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Gazebo-Simulation-red?style=for-the-badge" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-lightgrey?style=for-the-badge" />
</p>

<p>
  <img src="https://img.shields.io/badge/Build-Passing-success?style=flat-square" />
  <img src="https://img.shields.io/badge/RViz2-Visualization-purple?style=flat-square" />
  <img src="https://img.shields.io/badge/6DOF-Manipulator-cyan?style=flat-square" />
  <img src="https://img.shields.io/badge/Maintained-Yes-brightgreen?style=flat-square" />
</p>

</div>

---

```
 ____                      _
| __ ) _ __ __ _  ___ ___ (_) ___
|  _ \| '_ / _` |/ __/ __|| |/ _ \
| |_) | | | (_| | (_| (__ | | (_) |
|____/|_|  \__,_|\___\___||_|\___/

  Starting Manual Control Interface
  Author: Dilip Kumar
  ROS2 Humble | MoveIt2 | MediaPipe CV
```

---

## 🤖 Project Overview

**Braccio Robotic Arm Control** is a fully integrated ROS 2 Humble package for simulating, visualizing, and controlling the **Arduino Braccio 6DOF robotic arm** using a cutting-edge combination of:

- 🖐️ **Real-time hand gesture control** via [MediaPipe](https://google.github.io/mediapipe/) computer vision
- 🧠 **MoveIt 2** for motion planning and trajectory execution
- 🌍 **Gazebo** for physics-based simulation
- 👁️ **RViz 2** for real-time 3D visualization
- 🔧 **ros2_control** for hardware abstraction and controller management

> 💡 **Control the arm with your bare hands** — left hand selects the joint, right hand sets the angle. No joystick. No keyboard. Just gestures.

---

## ✨ Key Features

| Feature | Description |
|---|---|
| 👁️ **Computer Vision Control** | MediaPipe dual-hand tracking for intuitive joint control |
| 🦾 **6DOF Manipulation** | Full articulation: base, shoulder, elbow, wrist pitch, wrist roll, gripper |
| 🧠 **MoveIt 2 Integration** | OMPL-powered motion planning with collision avoidance |
| 🌍 **Gazebo Simulation** | Physics simulation with ros2_control plugin |
| 📊 **Real-time HUD** | Live joint telemetry dashboard overlaid on the webcam feed |
| 🔄 **Joint State Publishing** | 20 Hz joint state broadcasting compatible with RViz 2 |
| 🎯 **Geometric IK Solver** | Custom pick-and-place inverse kinematics |
| 🛡️ **Joint Limit Enforcement** | Per-joint angle limits strictly enforced in all control modes |

---

## 🏗️ Package Structure

```
robot_control/
├── 📁 config/
│   ├── braccio.srdf               # Semantic robot description (MoveIt2)
│   ├── braccio_display.rviz       # Pre-configured RViz2 layout
│   ├── joint_limits.yaml          # Per-joint velocity & position limits
│   ├── kinematics.yaml            # IK solver configuration (KDL)
│   ├── moveit_controllers.yaml    # MoveIt2 ↔ ros2_control bridge
│   └── ros2_controllers.yaml      # Joint trajectory & state broadcaster
│
├── 📁 launch/
│   ├── display.launch.py          # Standard RViz2 visualization launch
│   └── cv_display.launch.py       # Computer vision control launch
│
├── 📁 robot_control/              # Python ROS2 nodes
│   ├── braccio_commander.py       # Pick-and-place commander (MoveIt2 API)
│   ├── cv_hand_control.py         # MediaPipe gesture control node ⭐
│   └── joint_state_republisher.py # Mimic joint state relay
│
├── 📁 urdf/                       # Robot description (Xacro/URDF)
├── 📁 stl/                        # 3D mesh files for each arm segment
├── 📁 worlds/                     # Gazebo world files
├── package.xml
└── setup.py
```

---

## 🖐️ Computer Vision Control — How It Works

The `cv_hand_control.py` node streams your webcam and processes **two hands simultaneously**:

```
┌─────────────────────────────────────────────────────────────┐
│                    GESTURE CONTROL MAP                       │
├─────────────────┬───────────────────────────────────────────┤
│   LEFT HAND     │         Selects Active Joint               │
│   (Fingers Up)  │                                           │
│                 │  ✊ 0 fingers → Gripper                   │
│                 │  ☝️  1 finger  → Base                     │
│                 │  ✌️  2 fingers → Shoulder                 │
│                 │  🤟 3 fingers → Elbow                     │
│                 │  🖖 4 fingers → Wrist Pitch               │
│                 │  🖐️  5 fingers → Wrist Roll               │
├─────────────────┼───────────────────────────────────────────┤
│   RIGHT HAND    │         Controls Joint Angle               │
│   (Position)    │                                           │
│                 │  ↑ Move Hand UP   → Increase angle        │
│                 │  ↓ Move Hand DOWN → Decrease angle        │
│                 │  ✊ Make a FIST   → Lock & Apply angle     │
└─────────────────┴───────────────────────────────────────────┘
```

> 🔒 **Smart Clutch**: The angle only updates when you make a **fist** with your right hand, preventing accidental movement.

---

## 🦾 Robot Joint Specifications

| Joint | Name | Range (rad) | Range (deg) |
|---|---|---|---|
| 🔄 | `base_joint` | 0.0 → 3.14 | 0° → 180° |
| 💪 | `shoulder_joint` | 0.26 → 2.88 | 15° → 165° |
| 🦵 | `elbow_joint` | 0.0 → 3.14 | 0° → 180° |
| 🤝 | `wrist_pitch_joint` | 0.0 → 3.14 | 0° → 180° |
| 🌀 | `wrist_roll_joint` | 0.0 → 3.14 | 0° → 180° |
| 🤌 | `gripper_joint` | 0.175 → 1.274 | 10° → 73° |

---

## ⚙️ Prerequisites

Make sure you have the following installed on **Ubuntu 22.04**:

- **ROS 2 Humble** — [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **MoveIt 2** for Humble
- **Gazebo Classic** (with `gazebo_ros`)
- **Python 3.10+**
- **MediaPipe** + **OpenCV**

```bash
# Install Python dependencies
pip3 install mediapipe opencv-python

# Install ROS2 MoveIt & controllers (if not already installed)
sudo apt install ros-humble-moveit \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-joint-state-publisher-gui
```

---

## 🚀 Installation & Build

```bash
# 1. Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/your-username/robot_control.git

# 2. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build the package
colcon build --packages-select robot_control

# 4. Source the workspace
source install/setup.bash
```

---

## ▶️ Running the Project

### 🖥️ Option 1 — Standard Visualization (RViz2 + Sliders)

```bash
ros2 launch robot_control display.launch.py
```

Launches **RViz2** with `joint_state_publisher_gui` for manual joint control via sliders.

---

### 🖐️ Option 2 — Computer Vision Gesture Control

```bash
ros2 launch robot_control cv_display.launch.py
```

Launches:
- **RViz2** for 3D visualization
- **CV Hand Control Node** with webcam stream + live HUD
- **Joint State Republisher** for synchronized display

> ⚠️ **Webcam Required**: Ensure your webcam is connected and accessible at `/dev/video0`.

---

## 📡 ROS 2 Topics

| Topic | Message Type | Direction | Description |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Publish | Current angles for all 6 joints |
| `/robot_description` | `std_msgs/String` | Subscribe | URDF model from robot_state_publisher |

---

## 🧱 Tech Stack

<div align="center">

|  |  |  |
|---|---|---|
| <img src="https://img.shields.io/badge/ROS2-Humble-0A0A0A?style=for-the-badge&logo=ros" /> | <img src="https://img.shields.io/badge/MoveIt2-Planning-orange?style=for-the-badge" /> | <img src="https://img.shields.io/badge/Gazebo-Simulation-red?style=for-the-badge" /> |
| <img src="https://img.shields.io/badge/MediaPipe-Hand%20Tracking-00C853?style=for-the-badge&logo=google" /> | <img src="https://img.shields.io/badge/OpenCV-Vision-5C3EE8?style=for-the-badge&logo=opencv" /> | <img src="https://img.shields.io/badge/Python-3.10-FFD43B?style=for-the-badge&logo=python" /> |
| <img src="https://img.shields.io/badge/URDF%2FXacro-Robot%20Model-blue?style=for-the-badge" /> | <img src="https://img.shields.io/badge/ros2__control-Hardware%20Abstraction-blueviolet?style=for-the-badge" /> | <img src="https://img.shields.io/badge/RViz2-Visualization-purple?style=for-the-badge" /> |

</div>

---

## 🗺️ Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                     robot_control package                         │
│                                                                   │
│   ┌─────────────┐     /joint_states      ┌──────────────────┐   │
│   │ cv_hand_    │ ──────────────────────▶ │ robot_state_     │   │
│   │ control.py  │                         │ publisher        │   │
│   │ (MediaPipe) │                         └────────┬─────────┘   │
│   └─────────────┘                                  │             │
│                                            /robot_description    │
│   ┌─────────────┐                                  ▼             │
│   │ braccio_    │     MoveIt2 API       ┌──────────────────┐    │
│   │ commander   │ ─────────────────────▶│    RViz2         │    │
│   │ (Pick&Place)│                       │  (3D Preview)    │    │
│   └─────────────┘                       └──────────────────┘    │
│                                                                   │
│   ┌──────────────────────────────────────────────────────────┐   │
│   │                    ros2_control                           │   │
│   │  joint_state_broadcaster  +  joint_trajectory_controller │   │
│   └──────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 📂 Configuration Files

<details>
<summary><b>📋 kinematics.yaml</b> — IK Solver</summary>

```yaml
braccio_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_attempts: 3
  kinematics_solver_timeout: 0.05
```
</details>

<details>
<summary><b>📋 ros2_controllers.yaml</b> — Controller Setup</summary>

Configures `joint_state_broadcaster` and `braccio_arm_controller` (JointTrajectoryController) for all 6 joints.
</details>

<details>
<summary><b>📋 joint_limits.yaml</b> — Safety Limits</summary>

Per-joint position, velocity, and acceleration limits enforced by MoveIt 2 during planning.
</details>

---

## 🔮 Roadmap

- [x] URDF/Xacro robot model with accurate joint limits
- [x] RViz 2 visualization with pre-configured layout
- [x] MediaPipe dual-hand gesture control (6DOF)
- [x] Real-time HUD telemetry overlay
- [x] MoveIt 2 integration with OMPL planning
- [x] Gazebo physics simulation
- [ ] 🔧 Arduino hardware serial bridge for real arm control
- [ ] 📷 Object detection for autonomous pick-and-place
- [ ] 🤖 Reinforcement learning for optimized trajectories
- [ ] 🌐 Web dashboard for remote monitoring

---

## 📄 License

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

<div align="center">

**Dilip Kumar**

[![Email](https://img.shields.io/badge/Email-letsmaildilip%40gmail.com-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:letsmaildilip@gmail.com)
[![GitHub](https://img.shields.io/badge/GitHub-Follow-181717?style=for-the-badge&logo=github)](https://github.com/your-username)

*Robotics Engineer | ROS 2 Developer | Computer Vision Enthusiast*

</div>

---

<div align="center">

**⭐ If you found this project useful, please give it a star! ⭐**

<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=14&duration=4000&pause=1000&color=00D4FF&center=true&vCenter=true&width=500&lines=Built+with+❤️+using+ROS2+%2B+Python+%2B+MediaPipe;Braccio+Arm+%7C+6DOF+%7C+Gesture+Control" alt="Footer" />

</div>
