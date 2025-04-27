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

to run the environment on one terminal using:

```bash
ros2 launch stage_ros2 demo.launch.py world:=cave
```

on the other terminal run any node for example :

```bash
ros2 run collective_robotics navigation
```

it should navigate in the environment

