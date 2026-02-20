# Ground Segmentation ROS 2 Node (GSeg3D)

This file implements a **ROS 2 node** that wraps the [GSeg3D ground segmentation](https://github.com/dfki-ric/ground_segmentation.git) algorithm and exposes it as a real-time perception component for robotic systems.

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
| `cellSizeX`, `cellSizeY`, `cellSizeZ` | Grid resolution (Phase I) (m)|
| `cellSizeZPhase2`| Grid resolution in z (Phase II) (m)|
| `slopeThresholdDegrees` | Max slope for ground (degrees)|
| `groundInlierThreshold` | Plane fitting inlier threshold (m) |
| `centroidSearchRadius` | KD-tree expansion radius (m) |
| `lidar_to_ground` | Lidar to ground distance (m) |
| `transform_tolerance` | Tolerance for fetching transforms from TF (sec) |
| `show_benchmark` | Enable precision/recall evaluation |

## Usage Instructions

### System Requirements

OS: Ubuntu 22.04, Ubuntu 24.04

ROS2: Humble, Jazzy

### Prerequisite

Before building the ROS 2 wrapper, install dependencies and clone the core **ground_segmentation** library into your ROS 2 workspace:

```bash
sudo apt update
sudo apt install cmake libpcl-dev libeigen3-dev libgtest-dev libnanoflann-dev openjdk-17-jre
```

```bash
cd ~/ros2_ws/src
git clone https://github.com/dfki-ric/ground_segmentation.git
```

### Build Instructions

Clone the wrapper into your ROS 2 workspace `src` folder and build it:

```bash
cd ~/ros2_ws/src
git clone https://github.com/dfki-ric/ground_segmentation_ros2.git
colcon build --packages-up-to ground_segmentation_ros2 --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

### Running the ROS 2 Node

After building the package, launch the ground segmentation node using the provided ROS 2 launch file.

```bash
ros2 launch ground_segmentation_ros2 ground_segmentation.launch.py \
  pointcloud_topic:=<POINTCLOUD_TOPIC> \
  imu_topic:=<IMU_TOPIC> \
  sim:=<true/false>
```

Replace `<POINTCLOUD_TOPIC>` and `<IMU_TOPIC>` with the topics published by your LiDAR and IMU drivers. 

`sim` (boolean)
Controls whether simulation time is used.

Behavior:
- `true` → uses `/clock` (simulation or bag playback)
- `false` → uses system wall-clock time

Example:
```bash
sim:=true
```

### Node Configuration

- Parameters are loaded from:
  ```
  ground_segmentation_ros2/config/parameters.yaml
  ```
- Topic remapping is handled at launch time, allowing the node to remain independent of sensor-specific topic names.

### Runtime Note: `libjawt.so`

On some systems, the node may fail to start with:

```
error while loading shared libraries: libjawt.so: cannot open shared object file
```

### Cause
The visualization stack (PCL → VTK) may depend on Java AWT when VTK is built with Java support. Install JDK 17 using `sudo apt-get install openjdk-17-jre`.

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

## License

BSD-3 Clause License.

## Citation

If you use this work, please cite:

Muhammad Haider Khan Lodhi and Christoph Hertzberg, "GSeg3D: A High-Precision Grid-Based Algorithm for Safety-Critical Ground Segmentation in LiDAR Point Clouds" in *2025 7th International Conference on Robotics and Computer Vision (ICRCV)*, pp. 119-126, 2025. doi: [10.1109/ICRCV67407.2025.11349133](https://doi.org/10.1109/ICRCV67407.2025.11349133)

© DFKI Robotics Innovation Center