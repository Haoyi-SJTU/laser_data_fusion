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
git clone <repository-url>
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




