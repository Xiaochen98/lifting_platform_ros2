# Lifting Platform ROS2

## Introduction
Lifting Platform ROS2 is an automation project designed to control and manage lifting platforms using Robot Operating System 2 (ROS2). This solution is tailored for industrial environments, focusing on safety, precision, and efficiency in automated lifting operations.

## Features
- **ROS2-based Control:** Implements modular and scalable control architecture using ROS2.
- **Multi-language Support:** Combines Python and C++ for both high-level logic and real-time performance.
- **Easy Integration:** Designed to be integrated with various lifting platforms and industrial systems.
- **Safety & Reliability:** Prioritizes robust operation in critical industrial scenarios.

## Installation

### Prerequisites
- ROS2 (Humble or later recommended)
- Python 3.8+
- C++17 compatible compiler
- CMake

### Steps
1. Clone the repository:
   ```bash
   git clone https://github.com/Xiaochen98/lifting_platform_ros2.git
   ```
2. Navigate to the project folder:
   ```bash
   cd lifting_platform_ros2
   ```
3. Install required Python packages:
   ```bash
   pip install -r requirements.txt
   ```
4. Build the ROS2 workspace:
   ```bash
   colcon build
   ```
5. Source the setup script:
   ```bash
   source install/setup.bash
   ```

## Usage
After installation, you can launch the main platform controller node using ROS2 launch files:
```bash
ros2 launch lifting_platform_ros2 platform_controller.launch.py
```
Refer to the documentation and example configuration files for platform-specific setup.

## Technologies Used
- **Python** (≈48%): For scripting, automation, and high-level control.
- **C++** (≈45%): For real-time and performance-critical modules.
- **CMake** (≈7%): Project build and configuration.
- **ROS2:** Robotics middleware for distributed control.
- **Git:** Version control and collaboration.

## Contact
For more information about the project or the candidate’s experience, please reach out via GitHub or email.

- GitHub: [Xiaochen98](https://github.com/Xiaochen98)
- Email: [your.email@example.com] (please replace with your actual email)

---

Thank you for your interest in the Lifting Platform ROS2 project!
