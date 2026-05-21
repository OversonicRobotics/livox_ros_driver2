# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

Repo lives inside `ros2_ws/src/`. Build from workspace root:

```bash
cd ~/oversonic/ros2_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

SDK is bundled as prebuilt static libs in `livox_sdk/lib/{x86_64,aarch64}/`. No external Livox-SDK2 install needed.

Depends on `oversonic_nav2_msgs` (custom ROS2 message package in same workspace) for `CustomMsg`/`CustomPoint` types.

## Run

```bash
ros2 launch livox_ros_driver2 [launch_file]
```

If `liblivox_sdk_shared.so` not found: `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib`

## Architecture

**Entry point:** `DriverNode` (rclcpp component, `livox_ros::DriverNode`) — registered via `rclcpp_components`. Spawns two polling threads: point cloud and IMU.

**Core classes:**
- `DriverNode` (`src/driver_node.cpp`) — ROS2 node, reads launch params, owns `Lddc`
- `Lddc` (`src/lddc.cpp`) — "Lidar Data Distributor and Controller"; converts raw SDK data to ROS messages, publishes topics, handles merge logic
- `LdsLidar` (`src/lds_lidar.cpp`) — wraps Livox SDK, receives raw UDP packets, feeds `LidarDataQueue`

**Data flow:** `LdsLidar` (SDK callbacks) → `LidarDataQueue` (ring buffer, `src/comm/`) → `Lddc::DistributePointCloudData()` (polling loop) → ROS2 publishers

**Point cloud merge:** When `merge_pointcloud=true`, `Lddc` collects per-LiDAR buffers then calls `MergeMessages()` to publish single fused cloud on `/livox/lidar_merged`.

**Published topics (default):**
- `/livox/lidar` — per-LiDAR or merged pointcloud
- `/livox/imu` — IMU data

Borromeo launch remaps these to `/lidar_3d/pointcloud[_custom_msg]` and `/lidar_3d/imu`.

## Launch Files

| File | LiDAR | Format | Notes |
|------|-------|--------|-------|
| `msg_MID360_launch.py` | MID360 | PointCloud2 | multi_topic=1, frame `livox_f_frame` |
| `MID360_borromeo_launch.py` | MID360 | CustomMsg (xfer=1) | Oversonic robot config, remapped topics |

**Key launch params:**
- `xfer_format`: 0=PointCloud2(XYZRTLT), 1=CustomMsg, 2=PCL PointXYZI
- `multi_topic`: 0=shared topic, 1=per-LiDAR topic
- `merge_pointcloud`: merge all LiDARs into single cloud (Oversonic addition)
- `user_config_path`: path to JSON config (IP, ports, extrinsics)
- `publish_freq`: 0.5–100.0 Hz (clamped in code)

## Config Files

JSON configs in `config/` set LiDAR IP, host IP, ports, and per-device extrinsic parameters (roll/pitch/yaw in degrees, x/y/z in mm).

`MID360_config.json` — current robot: LiDAR at `10.1.8.30`, host at `10.1.8.150`, yaw=180°, x=-680mm offset.

`MID360_config_borromeo.json` — Borromeo robot variant.

## Oversonic Customizations

This is a fork of upstream `Livox-SDK/livox_ros_driver2`. Local additions:
- `merge_pointcloud` parameter and `Lddc::MergeMessages()` (merged multi-LiDAR publishing)
- `oversonic_nav2_msgs::CustomMsg/CustomPoint` replaces upstream `livox_ros_driver2::CustomMsg` — check `lddc.h` type aliases
- Borromeo-specific launch and config files
- ROS1 support removed
