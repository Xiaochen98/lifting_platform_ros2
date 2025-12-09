# Lifting Platform ROS2

## Introduction

**Lifting Platform ROS2** is a ROS2-based automation framework designed to control and manage electric lifting platforms. It allows users to command the platform to move to target heights, automate lifting sequences, and integrate platform motion into a larger robotics or industrial control system.

This project is intended for developers, researchers, and system integrators who require a reliable, adaptable lifting mechanism within a ROS2 environment.

<img width="450" height="800" alt="image" src="https://github.com/user-attachments/assets/626fa2f3-a65c-4ff2-889d-cbf82edef618" />



## Supported Hardware

This implementation is specifically customized for the **Haorang electric lifting column**.

An external **USB-to-RS232 adapter** is required for communication. If you are using lifting columns from other vendors, changes to the communication interface or protocol may be required based on the manufacturer’s specifications.



## Communication Interface Notes

For reliable serial communication, it is **strongly recommended** to use a USB-to-RS232 adapter based on the **FT232RL** chipset.

❌ **Avoid PL2303-based adapters.**  
Jetson devices do not provide native driver support for PL2303. Using it requires recompiling the Linux kernel, which adds unnecessary deployment complexity.



## System Architecture

The system consists of two major components:

1. **Core Control Layer (C++)**  
   Responsible for real-time serial communication with the lifting platform.  
   This layer processes low-level control commands and ensures deterministic behavior when sending and receiving data through the RS232 interface.

2. **ROS2 Integration Layer (Python)**  
   Provides user-facing ROS2 nodes, services, and interfaces that expose control functions to the ROS ecosystem.  
   This layer interprets high-level commands, sends corresponding instructions to the core controller, and handles platform configuration and automation logic.

This hybrid architecture combines the execution efficiency of C++ with the development flexibility of Python, ensuring both performance and scalability.



## Installation

### Prerequisites

- ROS2 **Humble**
- **Python 3.8+**
- **C++17-compatible** compiler
- **CMake**
- USB-to-RS232 adapter (**FT232RL recommended**)

### Installation Steps


#### Clone the repository
```bash
git clone https://github.com/Xiaochen98/lifting_platform_ros2.git
```

#### Navigate into the project directory
```bash
cd lifting_platform_ros2
```

#### Install Python dependencies
```bash
pip install -r requirements.txt
```

#### Build the ROS2 workspace
```bash
colcon build
```

#### Source the workspace
```bash
source install/setup.bash
```

## Usage

To start the lifting platform controller:
```bash
ros2 launch lifting_platform_ros2 platform_controller.launch.py
```

Before controlling the lifting platform, verify:

Correct serial device (e.g., /dev/ttyUSB0)

Matching baud rate with the lifting platform

Properly configured height limits and motion parameters

Configuration files in the repository can be adjusted to match your specific hardware setup.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For questions or collaboration, please open an issue or contact myself via github
