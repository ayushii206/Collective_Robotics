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

A. To clone the repository:

```bash
cd 
git clone https://github.com/ayushii206/Collective_Robotics.git
```
B. OR Unzip the project folder

## Step 3: Navigate to the `ros_work` folder and build the workspace

```bash
cd ros_work
colcon build --symlink-install 
```
OR this command if cloned from GitHub

```bash
cd /Collective_Robotics/A2/ros_work
```

## Step 4: To run the tasks follow the given steps:

Open a terminal and run the following command to load the environment:

```bash
ros2 launch stage_ros2 demo.launch.py world:=new_maze
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
ros2 run collective_robotics cleaning_bot
```

Result: The robot should navigate in the environment showing the behavior of a vaccum cleaning robot.

#### Strategies to enure  that almost everything gets cleaned:

1. The robot moves in a straight line for a fixed duration and then makes a sharp 90° turn (alternating left and right), mimicking the pattern of mowing or cleaning rows. This ensures systematic sweeping of the environment.
2. Laser scan data is continuously monitored. If the robot encounters an obstacle (e.g., wall or object within 0.4m), it stops forward motion and rotates away, helping it avoid getting stuck and continue coverage.
3. The robot tracks which 0.5m × 0.5m grid cells it has visited using odometry data. This allows it to estimate how much of the area has been covered and gives feedback on cleaning effectiveness.
4. Once the robot has visited approximately 90% of the predefined area (based on the number of unique grid cells), it automatically stops, assuming the area has been sufficiently cleaned.
5. Alternating the turning direction after each row (left/right) minimizes redundant turns and helps systematically cover adjacent paths without excessive overlap.

D. Open another terminal and run the following node for different behaviors of the robot:

- Random goals are generated near the robot one after the other and the robot reaches those goals avoiding obstacles

```bash
ros2 run collective_robotics move_to_goal 
```
- The robot moves straight until an obstacle is detected, then makes a sharp 90-degree turn (left or right) based on the obstacle's position and continues forward.

```bash
ros2 run collective_robotics random_turning
```

### TASK 2

A. Open the terminal and run the following launch file:

(For 10 robots change world:=swarm5 robot_count:=5 to world:=swarm10 robot_count:=10)

```bash
ros2 launch collective_robotics multi_robot_behavior.launch.py world:=swarm5 robot_count:=5
```
Result: The robots stop when in close proximity with each other.

B. Open the terminal and run the following launch file:

```bash
ros2 launch collective_robotics multi_robot_behavior_wait.launch.py world:=swarm5 robot_count:=5
```
Result: The robot starts  moving again after waiting a certain period of time

C. Open the terminal and run the following launch file:

```bash
ros2 launch collective_robotics multi_robot_swarm_aggregate.launch.py world:=swarm5 robot_count:=5
```

### Comparison between Gazebo and PlayerStage

| Feature                     | **Gazebo**                                      | **Player/Stage**                              |
|-----------------------------|-------------------------------------------------|-----------------------------------------------|
| **Type**                     | 3D robot simulator                              | 2D (Player) and 3D (Stage) robot simulators   |
| **Development Focus**        | Realistic physics and sensor simulation        | Robot control with a focus on multi-robot systems |
| **Environment**              | 3D environments with realistic physics          | 2D (Player) and simple 3D (Stage) environments |
| **Physics Engine**           | ODE, Bullet, DART                              | Simpler physics                               |
| **ROS Integration**          | Tight integration with ROS                      | Has ROS integration, but not as seamless      |
| **Sensors**                  | High fidelity sensors (LIDAR, cameras, IMUs, etc.) | Basic sensors like lasers, cameras, and range finders |
| **Graphics**                 | Advanced 3D rendering with textures and lighting | Limited 2D/3D graphics                        |
| **Multi-Robot Simulation**   | Supports large-scale multi-robot simulations    | Focuses on multi-robot simulations in 2D     |
| **Complexity**               | More complex setup and configuration            | Easier to set up and use                     |
| **Realism**                  | Highly realistic physics and environment        | Simpler physics and environment              |
| **Compatibility**            | Cross-platform (Linux, Windows, macOS)          | Primarily Linux-based                        |
| **Community & Support**      | Large community, extensive documentation        | Smaller community, fewer resources           |
| **Development Status**       | Actively developed and maintained by Open Robotics | Less actively developed                      |
| **Use Cases**                | High-fidelity robot simulation, autonomous vehicles | Multi-robot research, basic robot control tasks |
| **Customization**            | Extensive (custom models, environments, plugins) | Limited compared to Gazebo                   |



