<h1 align="center">Face Following Drone (ROS2 + Tello)</h1>

<p align="center">
  <img src="https://se-cdn.djiits.com/tpc/uploads/sku/cover/9e4b5fd8-325d-47b2-80ee-f47542134048@ultra.png" alt="Face Following Drone" width="420"/>
</p>

<p align="center">
  <a href="https://www.python.org/downloads/release/python-3100/">
    <img src="https://img.shields.io/badge/Python-≥3.10-blue.svg?logo=python&logoColor=white" alt="Python Version">
  </a>
  <a href="https://opencv.org/">
    <img src="https://img.shields.io/badge/OpenCV-≥4.8.0-green.svg?logo=opencv&logoColor=white" alt="OpenCV Version">
  </a>
  <a href="https://docs.ros.org/en/humble/">
    <img src="https://img.shields.io/badge/ROS2-Humble-ff69b4.svg?logo=ros&logoColor=white" alt="ROS2 Humble">
  </a>
</p>


---

## Overview

This project implements a **ROS2-based autonomous drone** capable of:

- Detecting and following human faces using computer vision  
- Debugging through voice feedback while flying  
- Running on the **DJI Tello** with full PID control and tracking modules  

Designed as a fun experimentation platform for **vision-based control**, the system integrates multiple ROS2 nodes for real-time image analysis, PID tuning, and coordinated drone behavior.

Inspired by this video: https://www.youtube.com/watch?v=esw88_gKOpA

---

## Architecture

```
Face_Following_Drone/
├── custom_services/        # Custom ROS2 services (speech, control)
├── drone_launcher/         # Launch files for the system
├── drone_pid_controller/   # PID control logic
├── face_tracking/          # Face & ArUco tag detection/tracking
├── speech_engine/          # Speech-based debugging and feedback
├── tello_controller/       # Drone velocity and pose control node
├── tello_driver/           # ROS2 driver for DJI Tello (PedroS235/tello_ros2_driver)
├── tello_msgs/             # Custom ROS2 message definitions
└── webcam_bridge/          # Camera bridge for local webcam testing
```

---

## Features

- **Face Tracking:** Detects and follows targets using OpenCV Haar cascades or ArUco markers  
- **PID Controller:** Stabilizes the drone’s orientation and position relative to the target  
- **Speech Engine:** Provides voice feedback about what the drone detects in real time  
- **ROS2 Integration:** Built on top of ROS2 Humble (Python/C++)  

---

## Dependencies

| Component | Version / Repo |
|------------|----------------|
| ROS2 | Humble (Ubuntu 22.04) |
| Python | ≥ 3.10 |
| OpenCV | ≥ 4.8 |
| `tello_ros2_driver` | [PedroS235/tello_ros2_driver](https://github.com/PedroS235/tello_ros2_driver) |
| `cv_bridge`, `image_transport`, `rclpy`, `geometry_msgs`, etc. | Standard ROS2 packages |

---

## Installation

```bash
# Clone the repository
git clone https://github.com/rantaluca/Face_Following_Drone.git
cd Face_Following_Drone

# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build
source install/setup.bash
```

---

## Launch

### Local Webcam Simulation

```bash
ros2 launch drone_launcher face_following.launch.py
```

### Or Real Tello Drone

```bash
ros2 launch drone_launcher drone.launch.py
```


---

## Author

**Robert Antaluca**  
ÉTS Montréal / Université de Technologie de Compiègne  
Website: [antaluca.com](http://antaluca.com)

---

## License

MIT License — feel free to modify and build upon this project for research or learning purposes.

---

## Acknowledgements

- [PedroS235](https://github.com/PedroS235) for the Tello ROS2 driver  
- @Jabrils for the inpiration
- OpenCV and ROS2 communities for their open-source contributions  
- DJI for providing the Tello SDK
