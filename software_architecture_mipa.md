# MiPA-X Hardware Architecture

**Version**: 3.1
**Last Updated**: 2026-01-12
**Project**: MiPA-X Robotic Platform

---

## Overview

The MiPA-X architecture is centered around high-compute modules for vision and AI, interconnected via a high-speed 1Gbps Ethernet network. The system utilizes a dual-processor approach for high-level tasks with hierarchical control layers for real-time operation.

---

## 1. Core Compute & Vision Processing

### Primary Processing Units

#### Dual NVIDIA Jetson Orin Modules
- **Function**: Primary "brains" for the robot
- **Location**: Upper Body
- **Responsibilities**:
  - Vision processing from RGBD cameras
  - AI inference and decision making
  - Video streaming at 720p (1280x720) @ 30FPS
  - Audio processing with microphone arrays and echo cancellation

#### Verdin iMX8M Plus
- **Type**: ARM-based System-on-Module (SoM)
- **Location**: Upper Body
- **Responsibilities**:
  - Dual 7-DOF robotic arm control interface
  - Local control bridge between Jetson and arm actuators
  - Real-time motion planning and trajectory execution
  - Dual-CAN bus management (can0 for left arm, can1 for right arm)

> **📘 Dual-Arm Control**: For detailed information about the T-Motor based dual-arm system, see [DUAL_ARM_INTEGRATION.md](DUAL_ARM_INTEGRATION.md) and [mipax_arms/README.md](mipax/mipax_arms/README.md).

#### Industrial PC (IPC)
- **Platform**: Running ROS 2 (Robot Operating System)
- **Location**: Chassis/Base
- **Responsibilities**:
  - Central node for chassis navigation
  - IMU and ultrasonic sensor data coordination
  - Base mobility control
  - SLAM and mapping

#### STM32H7 Microcontroller
- **Interface**: CAN bus @ 1 Mbps
- **Location**: Chassis - MCU + Power Board
- **Responsibilities**:
  - Real-time control loop (1ms / 1000Hz)
  - Low-level hardware interfacing
  - Power distribution management
  - Emergency stop monitoring

---

## 2. Structural Segmentation

### Upper Body (上半身)

**Components**:
- Dual NVIDIA Jetson Orin modules
- Verdin iMX8M Plus SoM
- Power distribution board
- Robotic arm interface
- 13.3-inch touchscreen (flush-mounted)
- Audio subsystem (microphone arrays, echo cancellation)

**Pending Improvements (v3.71)**:
- Optimization of module placement
- Improved cable management with cable trays
- Touchscreen flush integration with housing

### Chassis/Base (底盘)

**Components**:
- Industrial PC (IPC) running ROS 2
- 48V Battery system with display
- Emergency Stop Button (急停按钮)
- Physical safety switches
- Indicator lights
- Motor controllers

**Critical Issues**:
- Chassis exceeds original design dimensions (requires size optimization)
- Battery accessibility improvement needed for user-swappable design
- Motor failure incident (front-right drive motor) indicates need for easier maintenance access

**Pending Improvements (v3.71)**:
- Structural optimization for easier maintenance
- Battery position redesign for autonomous swapping
- Chassis size reduction

---

## 3. Sensors and Connectivity

### Vision Sensors

#### Wide-Angle RGBD Camera
- **FOV**: 190° Horizontal / 114° Vertical
- **Resolution**: 1280x720 (720p)
- **Frame Rate**: 30 Hz
- **Location**: Upper Body
- **Purpose**: Primary spatial awareness and environment understanding

#### Secondary RGBD Camera
- **FOV**: 86° Horizontal / 58° Vertical
- **Resolution**: 640x480
- **Frame Rate**: 30 Hz
- **Location**: Upper Body
- **Purpose**: Localized manipulation and robotic arm feedback

#### Fish-eye Cameras (2x)
- **FOV**: 120° Horizontal
- **Resolution**: 1920x1080
- **Frame Rate**: 30 Hz
- **Location**: Front & Rear of upper body
- **Purpose**: Wide-area surround view

#### 335LG Precision Sensor
- **FOV**: 18° Vertical
- **Location**: Height-specific mounting (moved downward in v3.1 due to space constraints)
- **Purpose**: Precision depth/ranging for specific tasks

### Navigation Sensors

#### 3D LiDAR
- **FOV**: 360° Horizontal coverage
- **Vertical Layers**: 16-32 layers
- **Vertical FOV**: ±15°
- **Range**: 0.3m to 100m
- **Update Rate**: 10 Hz
- **Location**: Center-mounted on base
- **Purpose**: SLAM and environmental mapping

#### IMU (Inertial Measurement Unit)
- **Update Rate**: 50-100 Hz
- **Location**: Chassis
- **Data**: Acceleration, gyroscope, orientation
- **Integration**: EKF fusion with wheel odometry @ 50Hz

#### Ultrasonic Sensor Array
- **Count**: 8 sensors
- **Sensing Angle**: 75° per sensor
- **Positions**: Distributed around base (Front-Left, Front-Right, Right-Front, Right-Rear, Rear-Right, Rear-Left, Left-Rear, Left-Front)
- **Range**: 0.02m to 4.0m
- **Update Rate**: 30 Hz
- **Purpose**: Close-range collision avoidance with 360° proximity coverage

### Network Infrastructure

#### Primary Backbone
- **Bandwidth**: 1Gbps (1000 Mbps) Ethernet
- **Important Note**: Bandwidth specified in **Mb/s (Megabits per second)**, NOT MB/s (Megabytes per second)
- **Topology**: Hierarchical - connecting upper body compute modules to chassis IPC

#### Wireless Connectivity
- **Wi-Fi**: Dual-band (2.4GHz + 5GHz)
- **Bluetooth**: 5.0
- **Purpose**: External communication and remote control

---

## 4. Physical Specifications

### Robotic Arm System

**Linear Elevation**:
- **Lift Range**: 600mm vertical travel
- **Mounting**: Linear axis on upper body

**Workspace Optimization** (Under Evaluation):
- Potential increase in joint angles (J2, J4, J6) for improved elbow and wrist workspace
- Dependent on final housing design
- Payload target: ~7.5kg (not achievable with current hardware)
- Arm length target: 900mm (not achievable with current hardware)

### Dual 7-DOF Arm System (T-Motor Based)

**Configuration**:
- **Left Arm**: 7 DOF (Degrees of Freedom)
- **Right Arm**: 7 DOF (Degrees of Freedom)
- **Total**: 14 actuated joints

**Motor Specifications** (T-Motor AK80-6 V2):
- **Torque**: Continuous 6 Nm, Peak 9 Nm
- **Speed**: Max 41 rad/s (~391 RPM)
- **Control**: Position, Velocity, Torque, Hybrid modes
- **Communication**: CAN 2.0B
- **Encoder**: 16-bit absolute positioning

**CAN Communication**:
- **Left Arm CAN Bus**: can0 @ 1 Mbps
  - Motor IDs: 0x01 through 0x07
- **Right Arm CAN Bus**: can1 @ 1 Mbps
  - Motor IDs: 0x01 through 0x07
- **STM32H7 Interface**: Dual CAN transceivers
- **Control Rate**: 300 Hz (per arm)
- **Bandwidth Utilization**: 67% (33% safety margin)

**Control Architecture**:
- **Low-Level**: STM32H7 MCU (1ms real-time loop)
- **Mid-Level**: Verdin iMX8M Plus (ROS 2 driver node)
- **High-Level**: Pyro-based trajectory planning
- **Modes**: 8 control modes per joint
  - disable, enable, position, velocity, torque, damped_torque, vel_torque, full

**Kinematic Parameters** (Example - Per Arm):
- **Link Lengths**: [0.3, 0.3, 0.25, 0.25, 0.15, 0.12, 0.10] meters
- **Link Masses**: [2.0, 1.8, 1.5, 1.2, 0.8, 0.5, 0.3] kg
- **Total Reach**: ~1.57 meters
- **Payload**: Up to 3 kg per arm

**Software Stack**:
- **ROS 2 Package**: mipax_arms
- **Motor Driver**: mini-cheetah-tmotor-python-can
- **Trajectory Planning**: Pyro robotics library
- **Control Algorithms**: Computed torque, PD control, inverse kinematics
- **Safety Features**: Joint limits, collision avoidance, watchdog timers

**Integration**:
- Based on: [tmotor_ros2](https://github.com/SherbyRobotics/tmotor_ros2.git)
- See: [DUAL_ARM_INTEGRATION.md](DUAL_ARM_INTEGRATION.md) for complete details
- Package: [mipax_arms](mipax/mipax_arms/)

### Base Dimensions
- **Robot Radius**: 0.22m (220mm)
- **Current Status**: Chassis exceeds design specifications
- **Planned**: Size optimization in v3.71

### Power System
- **Battery**: 48V system
- **Display**: Internal battery data display on body
- **Issue**: Current battery position difficult to access (redesign planned)

### User Interface
- **Touchscreen**: 13.3-inch
- **Mounting**: Being updated to flush-mount design

---

## 5. Communication Architecture

### High-Speed Data Layer
```
┌─────────────────────────────────────────────────────┐
│           1Gbps Ethernet Backbone                    │
│              (1000 Mb/s)                             │
│                                                       │
│  ┌──────────────────┐         ┌──────────────────┐  │
│  │  Upper Body      │         │  Chassis/Base    │  │
│  │  - Jetson Orin × 2│◄──────►│  - IPC (ROS 2)  │  │
│  │  - Verdin iMX8M+ │         │  - Navigation    │  │
│  │  - Vision         │         │  - Sensors       │  │
│  └──────────────────┘         └──────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### Real-Time Control Layer
```
┌─────────────────────────────────────────────────────┐
│           CAN Bus (1 Mbps)                           │
│                                                       │
│  ┌──────────────────┐         ┌──────────────────┐  │
│  │  IPC / Upper     │         │  STM32H7 MCU     │  │
│  │  Body Control    │◄──────►│  - 1ms loop      │  │
│  │                  │         │  - Motor control │  │
│  │                  │         │  - Power mgmt    │  │
│  └──────────────────┘         └──────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### ROS 2 Topic Layer

**Primary Data Streams**:
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | nav_msgs/Odometry | 50 Hz | EKF-fused odometry |
| `/scan` | sensor_msgs/LaserScan | 10 Hz | 3D LiDAR scan |
| `/imu/data` | sensor_msgs/Imu | 50-100 Hz | IMU data |
| `/camera/image` | sensor_msgs/Image | 30 Hz | Wide-angle camera |
| `/secondary_camera/image` | sensor_msgs/Image | 30 Hz | Secondary camera |
| `/fisheye_front/image` | sensor_msgs/Image | 30 Hz | Front fish-eye |
| `/fisheye_rear/image` | sensor_msgs/Image | 30 Hz | Rear fish-eye |
| `/cmd_vel` | geometry_msgs/Twist | varies | Velocity commands |

---

## 6. Development Status & Improvements

### Version 3.1 Status

**Completed**:
- ✅ Dual Jetson Orin integration
- ✅ 1Gbps Ethernet backbone
- ✅ 8 ultrasonic sensor array
- ✅ 3D LiDAR integration
- ✅ Multiple camera system (wide-angle, secondary, fish-eye)
- ✅ Emergency stop button integration

**In Progress (v3.71)**:
- 🔄 Battery accessibility redesign
- 🔄 Chassis size optimization
- 🔄 Module relocation evaluation
- 🔄 Touchscreen flush-mount integration
- 🔄 335LG sensor repositioning
- 🔄 Structural optimization for maintenance access

**Issues Identified**:
- ❌ Motor failure (front-right drive) - maintenance difficulty
- ❌ Battery position prevents user swapping
- ❌ Chassis exceeds original dimensions
- ❌ Internal space constraints in upper body

### Production Standardization

**Planned Improvements**:
1. **Wiring Standards**:
   - Cable trays for organization
   - Wire labeling system
   - Power/signal cable separation

2. **Environmental Protection**:
   - Conformal coating on exposed PCBs
   - Dust and moisture protection

3. **Interface Stability**:
   - Structural adhesive at cable interfaces
   - Strain relief for critical connections

---

## 7. Regulatory & Deployment

### Certification Status
- **CE Certification**: Not yet obtained
- **Current Shipping**: Separate parts to clear customs
- **Target**: Complete unit shipping after certification

---

## 8. Technical Notes

### Bandwidth Calculation
**Critical**: All bandwidth measurements use **Mb/s (Megabits per second)**
- 1 Gbps = 1000 Mb/s = 125 MB/s
- Configuration errors possible if using MB/s incorrectly

### Discovery & Network
- **ROS 2 Domain**: 0 (default)
- **Discovery Protocol**: SIMPLE (automatic participant discovery)
- **DDS Implementation**: FastDDS (rmw_fastrtps_cpp)
- **Discovery Range**: Configurable (LOCALHOST or SUBNET)

---

## Sensor Configuration Summary Table

| Sensor Type | FOV | Resolution | Rate | Location | Purpose |
|-------------|-----|------------|------|----------|---------|
| **Wide-Angle RGBD** | 190°H / 114°V | 1280x720 | 30Hz | Upper Body | Primary vision |
| **Secondary RGBD** | 86°H / 58°V | 640x480 | 30Hz | Upper Body | Manipulation |
| **Fish-eye (Front)** | 120°H | 1920x1080 | 30Hz | Front | Surround view |
| **Fish-eye (Rear)** | 120°H | 1920x1080 | 30Hz | Rear | Surround view |
| **335LG Depth** | 18°V | 640x480 | 30Hz | Height-specific | Precision ranging |
| **3D LiDAR** | 360°H / ±15°V | Point cloud | 10Hz | Base center | SLAM/Mapping |
| **Ultrasonic (×8)** | 75° each | Range | 30Hz | Base perimeter | Collision avoidance |
| **IMU** | N/A | 6-DOF | 50-100Hz | Chassis | Motion tracking |

---

*Last Updated: 2026-01-12*
*Architecture Version: 3.1*
*Next Revision: v3.71 (Structural optimization)*
