# psim_publisher

1. Build this workspace.

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

2. Run.

```
source install/setup.bash
ros2 launch psim_publisher psim_publisher.launch.xml
```