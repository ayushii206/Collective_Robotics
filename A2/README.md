## Step 1: Setup Stage in your system using the following steps:

A. Install dependencies:

```bash
sudo apt install cmake libjpeg-dev libpng-dev libfltk1.3-dev libx11-dev libxt-dev libltdl-dev
```

B. Clone and build Stage:

```bash
cd ~/Downloads
git clone https://github.com/rtv/Stage.git
cd Stage
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

## Step 2: Clone the Github repository or unzip the project folder

## Step 3: Navigate to the `ros_work` folder and build the workspace

```bash
cd ros_work
colcon build --symlink-install 
```
## Step 4: To run the tasks follow the given steps:

Open a terminal and run the following command to load the environment:

```bash
ros2 launch stage_ros2 demo.launch.py world:=cave
```

### TASK 1

A. Open another terminal and run Obstacle avoidance node:

```bash
ros2 run collective_robotics navigation
```

Result: The robot should navigate in the environment avoiding obstacles.

B. Open another terminal and run Wall follower node:

```bash
ros2 run collective_robotics wall_follower
```

Result: The robot should navigate in the environment following the walls on its right side.

C. Open another terminal and run vaccum cleaning node:

```bash
ros2 run collective_robotics ...
```

Result: The robot should navigate in the environment showing the behavior of a vaccum cleaning robot.

#### Strategies to enure  that almost everything gets cleaned:

1.
2.
3.

D. Open another terminal and run the following nodes for different behaviors of the robot:

```bash
ros2 run collective_robotics ...
```

### TASK 2

To run Part b (Multi-Robot System) -- in my case its 5 beacuse 20 Robots were creating a heavy load in my system, i have made a launch file for each of the task 

For Part b task 1

```bash
ros2 launch collective_robotics multi_robot_behavior.launch.py
```
For Part b task 2

```bash
ros2 launch collective_robotics multi_robot_behavior_wait.launch.py
```
For Part c task 3 (Still correction required)

```bash
ros2 launch collective_robotics multi_robot_swarm_aggregate.launch.py
```

