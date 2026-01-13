# Laser Data Fusion Package

A ROS package for fusing multiple laser scanner data into a unified laser scan message.

## Overview

This package provides two main nodes for laser data fusion:
- **laserDataFusion**: Fuses data from two real laser scanners with intensity information
- **simlaserDataFusion**: Fuses data from two simulated laser scanners (intensity-free version)

The package transforms laser scan data from different coordinate systems into a unified base frame and merges them into a single 360-degree laser scan.

![](https://github.com/Haoyi-SJTU/laser_data_fusion/blob/main/rviz.jpg?raw=true)

## Features

- **Multi-laser Fusion**: Supports fusion of two laser scanners
- **Coordinate Transformation**: Transforms laser data to base_link frame
- **Real-time Processing**: Configurable update rate (default: 20Hz)
- **Configurable Parameters**: Easy configuration of laser positions and scan parameters
- **ROS Compatible**: Fully integrated with ROS messaging system

## Dependencies

### ROS Packages
- roscpp
- rospy  
- sensor_msgs
- std_msgs
- tf
- message_generation
- laser_geometry

### System Requirements
- CMake 2.8.3 or higher
- C++14 compatible compiler
- ROS Kinetic or newer

## Installation

1. Clone the package to your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone github.com/Haoyi-SJTU/laser_data_fusion
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source the workspace:
```bash
source devel/setup.bash
```

## Configuration

### Laser Parameters

The package uses configuration files to define laser scanner properties:

**For real lasers** (`laserInfo.h`):
- Laser offsets: Position and orientation relative to base
- Distance range: [0.0m, 12.0m]
- Angular filtering for each laser
- 1440 points per full scan (4 points per degree)

**For simulated lasers** (`simLaserInfo.h`):
- Simplified configuration without intensity data
- 2D laser offsets only

### Node Parameters

- **LOOP_RATE**: Processing frequency (default: 20Hz)
- **POINTS_PER_ANGLE**: Resolution (default: 4 points/degree)
- **DIST_RANGE**: Valid distance range [min, max]

## Usage

### Running the Nodes

**Real Laser Fusion:**
```bash
rosrun laser_data_fusion laserDataFusion
```

**Simulated Laser Fusion:**
```bash
rosrun laser_data_fusion simlaserDataFusion
```

### Topics

#### Subscribed Topics
- `/laser_scan0` (sensor_msgs/LaserScan): First laser scanner data
- `/laser_scan1` (sensor_msgs/LaserScan): Second laser scanner data
- `/sim_scan0`, `/sim_scan1`: Simulated laser topics

#### Published Topics
- `/scan` (sensor_msgs/LaserScan): Fused laser scan data

### Message Structure

The fused laser scan message includes:
- 360-degree coverage (0 to 2π radians)
- Configurable range and intensity data
- Timestamp and frame_id information
- Time-incremental scan timing

## Code Structure

```
laser_data_fusion/
├── src/
│   ├── laserFusion.cpp          # Main fusion node for real lasers
│   └── simLaserFusion.cpp       # Simplified version for simulation
├── include/
│   ├── laser_data_fusion/
│   │   ├── laserInfo.h          # Configuration for real lasers
│   │   └── simLaserInfo.h       # Configuration for simulated lasers
│   └── common/                  # Common utilities
├── CMakeLists.txt               # Build configuration
└── package.xml                  # Package metadata
```

## Key Components

### Data Processing Pipeline

1. **Coordinate Transformation**: Converts laser data to base_link frame
2. **Angular Classification**: Maps points to angular bins
3. **Temporal Filtering**: Removes stale data (>200ms old)
4. **Message Merging**: Combines data from multiple lasers

### Utility Libraries

The package includes common utilities:
- **Singleton Pattern**: Thread-safe singleton implementation
- **Thread Timer**: Precise timing utilities
- **Logging Macros**: Comprehensive logging system
- **Math Constants**: Mathematical constants and conversions

## Customization

### Modifying Laser Configuration

Edit the configuration headers to match your setup:

```cpp
// Laser positions relative to base_link [x, y, yaw]
constexpr double LASER_OFFSET[][3] = {{0.765, 0.365, 5*PI/4}, {-0.075, -0.365, PI/4}};

// Valid distance range
constexpr double DIST_RANGE[] = {0.0, 12.0};
```

### Adding New Lasers

1. Update the laser configuration array
2. Add new subscriber callbacks
3. Modify the scanning filter logic

## Troubleshooting

### Common Issues

1. **No output on /scan topic**: Check if input topics are publishing data
2. **Incorrect transformations**: Verify laser offset parameters
3. **High CPU usage**: Consider reducing LOOP_RATE parameter

### Debugging

Enable debug output by setting ROS log level:
```bash
roscore
rosconsole set roscpp DEBUG
```

## License

This project is licensed under the BSD License.

# 激光数据融合功能包

一个用于将多个激光扫描仪数据融合为统一激光扫描消息的ROS功能包。

## 概述

本功能包提供两个主要节点用于激光数据融合：
- **laserDataFusion**：融合两个真实激光扫描仪的数据（包含强度信息）
- **simlaserDataFusion**：融合两个模拟激光扫描仪的数据（简化版本，无强度信息）

该功能包将来自不同坐标系下的激光扫描数据转换到统一的基座标系，并将它们合并为单个360度激光扫描。
![](https://github.com/Haoyi-SJTU/laser_data_fusion/blob/main/rviz.jpg?raw=true)

## 功能特点

- **多激光融合**：支持两个激光扫描仪的融合
- **坐标变换**：将激光数据转换到base_link坐标系
- **实时处理**：可配置的更新频率（默认：20Hz）
- **可配置参数**：激光位置和扫描参数易于配置
- **ROS兼容**：完全集成ROS消息系统

## 依赖项

### ROS功能包
- roscpp
- rospy
- sensor_msgs
- std_msgs
- tf
- message_generation
- laser_geometry

### 系统要求
- CMake 2.8.3 或更高版本
- 支持C++14的编译器
- ROS Kinetic 或更新版本

## 安装

1. 将功能包克隆到ROS工作空间：
```bash
cd ~/catkin_ws/src
git clone github.com/Haoyi-SJTU/laser_data_fusion
```

2. 编译功能包：
```bash
cd ~/catkin_ws
catkin_make
```

3. 配置工作空间环境：
```bash
source devel/setup.bash
```

## 配置

### 激光参数

功能包使用配置文件定义激光扫描仪属性：

**真实激光** (`laserInfo.h`)：
- 激光偏移量：相对于基座的位置和方向
- 距离范围：[0.0m, 12.0m]
- 每个激光的角度过滤
- 每次完整扫描1440个点（每度4个点）

**模拟激光** (`simLaserInfo.h`)：
- 简化配置，无强度数据
- 仅2D激光偏移量

### 节点参数

- **LOOP_RATE**：处理频率（默认：20Hz）
- **POINTS_PER_ANGLE**：分辨率（默认：4点/度）
- **DIST_RANGE**：有效距离范围[最小值, 最大值]

## 使用方法

### 运行节点

**真实激光融合：**
```bash
rosrun laser_data_fusion laserDataFusion
```

**模拟激光融合：**
```bash
rosrun laser_data_fusion simlaserDataFusion
```

### 话题

#### 订阅的话题
- `/laser_scan0` (sensor_msgs/LaserScan)：第一个激光扫描仪数据
- `/laser_scan1` (sensor_msgs/LaserScan)：第二个激光扫描仪数据
- `/sim_scan0`, `/sim_scan1`：模拟激光话题

#### 发布的话题
- `/scan` (sensor_msgs/LaserScan)：融合后的激光扫描数据

### 消息结构

融合后的激光扫描消息包含：
- 360度覆盖范围（0到2π弧度）
- 可配置的距离和强度数据
- 时间戳和frame_id信息
- 时间增量扫描时序

## 代码结构

```
laser_data_fusion/
├── src/
│   ├── laserFusion.cpp          # 真实激光融合主节点
│   └── simLaserFusion.cpp       # 模拟激光简化版本
├── include/
│   ├── laser_data_fusion/
│   │   ├── laserInfo.h          # 真实激光配置
│   │   └── simLaserInfo.h       # 模拟激光配置
│   └── common/                  # 通用工具
├── CMakeLists.txt               # 构建配置
└── package.xml                  # 功能包元数据
```

## 核心组件

### 数据处理流程

1. **坐标变换**：将激光数据转换到base_link坐标系
2. **角度分类**：将点映射到角度区间
3. **时间过滤**：移除过时数据（>200ms）
4. **消息合并**：合并多个激光数据

### 工具库

功能包包含通用工具：
- **单例模式**：线程安全的单例实现
- **线程定时器**：精确的定时工具
- **日志宏**：全面的日志系统
- **数学常量**：数学常数和转换

## 自定义配置

### 修改激光配置

编辑配置文件以匹配您的设置：

```cpp
// 激光相对于base_link的位置 [x, y, 偏航角]
constexpr double LASER_OFFSET[][3] = {{0.765, 0.365, 5*PI/4}, {-0.075, -0.365, PI/4}};

// 有效距离范围
constexpr double DIST_RANGE[] = {0.0, 12.0};
```

### 添加新激光

1. 更新激光配置数组
2. 添加新的订阅回调函数
3. 修改扫描过滤逻辑


## 许可证

本项目基于BSD许可证授权。
