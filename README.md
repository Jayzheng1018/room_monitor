# room_monitor

`room_monitor` is a ROS2 Python package that provides:
- a FastAPI web UI for mapping/patrol workflows,
- ROS2 integration for Kobuki navigation/patrol control,
- map auto-segmentation and zone editing support.

## Build

From your ROS2 workspace root (the parent folder that contains `src/`):

```bash
colcon build --packages-select room_monitor
source install/setup.bash
```

## Run

```bash
ros2 run room_monitor robot_master
```

The web server will start on port `5010` by default.
