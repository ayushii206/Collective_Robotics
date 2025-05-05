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

D. Open another terminal and run the following nodes for different behaviors of the robot:

```bash
ros2 run collective_robotics ...
```

### TASK 2

A. Open the terminal and run the following launch file:

(For 20 robots change world:=swarm5 to world:=swarm20)

```bash
ros2 launch collective_robotics multi_robot_behavior.launch.py world:=swarm5
```
Result: The robots stop when in close proximity with each other.

B. Open the terminal and run the following launch file:

```bash
ros2 launch collective_robotics multi_robot_behavior_wait.launch.py world:=swarm5
```
Result: The robot starts  moving again after waiting a certain period of time

C. Open the terminal and run the following launch file:

```bash
ros2 launch collective_robotics multi_robot_swarm_aggregate.launch.py world:=swarm5
```


