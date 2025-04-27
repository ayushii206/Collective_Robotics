# A2 Instructions

After cloning the repository, navigate to the `ros_work` folder:

```bash
cd ros_work
```

To build all packages, run:

```bash
colcon build --symlink-install
```

If you only want to build the important packages (collective_robotics and stage_ros2), run:

```bash
colcon build --symlink-install --packages-select collective_robotics stage_ros2
```

