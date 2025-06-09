# 14-Day ROS + AI Learning Roadmap

This repository follows the 14-day ROS (Robot Operating System) + AI learning plan. Each day focuses on a specific topic, ranging from ROS fundamentals to AI integration with robots. It is designed for students, robotics enthusiasts, and AI developers who want to build intelligent robotic systems from scratch.

---

## Prerequisites

- Ubuntu 20.04 or 22.04
- ROS 1 (Noetic) or ROS 2 (Foxy/Galactic)
- Python 3.8+
- AI libraries such as OpenCV, PyTorch, or TensorFlow
- ROS tools: `rosdep`, `catkin`, `colcon` (depending on ROS version)

---

## Installation

### ROS Installation (ROS Noetic Example)

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```
### Dependency Installation
```bash
sudo apt install python3-opencv python3-pip
pip3 install torch torchvision rospkg
```

## Project Structure
```bash
ros_ai_ws/
└── src/
    ├── day01_ros_basics/
    ├── day02_package_structure/
    ├── day03_pub_sub/
    ├── day04_services/
    ├── day05_parameters/
    ├── day06_sensors/
    ├── day07_gazebo_simulation/
    ├── day08_opencv_integration/
    ├── day09_pytorch_node/
    ├── day10_pretrained_models/
    ├── day11_realtime_processing/
    ├── day12_control_logic/
    ├── day13_full_demo/
    └── day14_benchmarking/
```
