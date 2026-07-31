# ros2_rover_elevation_mapping_cpp

ROS 2 C++ local traversability mapping from `/merged_cloud`.

## Current mapping logic

- Input cloud is transformed to `base_link`.
- PCL normals produce a progressive slope cost.
- Height thresholds preserve obstacle detection for the 360-degree LiDAR.
- Bresenham ray tracing marks cells before each endpoint as free.
- Endpoint cost has priority over free-space ray tracing.
- Unknown cells remain `-1`.

Final cell priority:

1. measured endpoint: `max(slope_cost, height_cost)`;
2. ray-traced free cell: `0`;
3. unobserved cell: `-1`.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_rover_elevation_mapping_cpp --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch ros2_rover_elevation_mapping_cpp elevation_mapping.launch.py
```
