# Ground Segmentation ROS 2 Node (GSeg3D)

This file implements a **ROS 2 node** that wraps the [GSeg3D ground segmentation](https://git.hb.dfki.de/dfki-perception/ground_segmentation) algorithm and exposes it as a real-time perception component for robotic systems.

The node subscribes to LiDAR point clouds (optionally synchronized with IMU data), performs **two-phase grid-based ground segmentation**, and publishes ground and obstacle point clouds for downstream navigation and perception modules.

## Overview

The node:
- Receives a `sensor_msgs/PointCloud2` input
- Optionally fuses IMU orientation for gravity alignment
- Applies **GSeg3D Phase I (coarse)** and **Phase II (refinement)** segmentation
- Publishes:
  - Ground points
  - Obstacle (non-ground) points
  - Raw filtered input cloud

## Subscribed Topics

| Topic | Type | Description |
|------|------|-------------|
| `/ground_segmentation/input_pointcloud` | `sensor_msgs/msg/PointCloud2` | Input LiDAR point cloud |
| `/ground_segmentation/input_imu` | `sensor_msgs/msg/Imu` | IMU orientation (optional) |

IMU synchronization is enabled via the parameter `use_imu_orientation`.

## Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `/ground_segmentation/ground_points` | `sensor_msgs/msg/PointCloud2` | Segmented ground points |
| `/ground_segmentation/obstacle_points` | `sensor_msgs/msg/PointCloud2` | Segmented non-ground points |
| `/ground_segmentation/raw_points` | `sensor_msgs/msg/PointCloud2` | Filtered input cloud |

## Processing Pipeline

1. **Input Filtering**
   - ROI cropping (min/max X, Y, Z)
   - Optional voxel downsampling

2. **Synthetic Ground Injection**
   - Injects a ground seed beneath the robot
   - Ensures reliable ground region initialization

3. **Frame Alignment**
   - Transforms point cloud into `robot_frame`
   - Uses TF2 for frame lookup
   - Uses IMU data (if enabled) to compute gravity-aligned orientation

4. **Two-Phase Ground Segmentation**
   - **Phase I:** Coarse grid segmentation (large Z resolution)
   - **Phase II:** Fine grid refinement (small Z resolution)

5. **Post-Processing**
   - Radius-based filtering around the robot
   - Final ground / obstacle separation

6. **Publishing**
   - Results published as ROS 2 PointCloud2 messages

## Parameters (Key)

| Parameter | Description |
|---------|-------------|
| `robot_frame` | Target frame for segmentation |
| `use_imu_orientation` | Enable IMU-based gravity alignment |
| `cellSizeX`, `cellSizeY`, `cellSizeZ` | Grid resolution (Phase I) |
| `cellSizeZPhase2`| Grid resolution in z (Phase II) |
| `slopeThresholdDegrees` | Max slope for ground |
| `groundInlierThreshold` | Plane fitting inlier threshold |
| `centroidSearchRadius` | KD-tree expansion radius |
| `lidar_to_ground` | Lidar to ground distance |
| `transform_tolerance` | Tolerance for fetching transforms from TF |
| `show_benchmark` | Enable precision/recall evaluation |

### Build Instructions

```bash
colcon build --packages-up-to ground_segmentation_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

## Benchmarking Mode

When `show_benchmark=true`:

- Runtime statistics are printed to console

## Intended Use

- Real-time ground segmentation for mobile robots
- Safety-critical navigation and obstacle detection
- Traversability analysis
- Research 

## Notes

- Requires a valid TF tree between sensor, IMU, and robot frames
- Designed for CPU real-time execution

© DFKI Robotics Innovation Center