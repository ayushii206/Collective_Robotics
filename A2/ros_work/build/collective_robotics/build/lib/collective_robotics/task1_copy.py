import rclpy
import math
import sys
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PoseArray, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from math import atan2, sqrt, pi
from tf_transformations import euler_from_quaternion
import numpy as np
import heapq
from scipy.ndimage import binary_dilation
from skimage.morphology import disk

np.set_printoptions(threshold=sys.maxsize)


class OdometryMotionModel(Node):
    def __init__(self):
        """Initializes the OdometryMotionModel node."""
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=5
        )
        super().__init__('odometry_motion_model')

        # Subscriptions
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        # self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10) # Update the Datatype
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goalpose', self.goal_pose_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, '/path_line', 10)
        self.path_point_publisher = self.create_publisher(PoseArray, '/path_points', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initial Variables
        self.current_pos_x, self.current_pos_y, self.orientation = 0.0, 0.0, 0.0
        self.goal_x, self.goal_y, self.goal_orientation = None, None, None
        self.map_data = None
        self.grid = None
        self.path = None
        self.update_grid = None
        self.path_worldframe = None
        self.path_index = 0  # Counter for the current point in the path
        # self.threshold = 10  # Adjusted threshold value for map padding (Simulation)
        self.threshold = 2  # Adjusted threshold value for map padding (Real)
        # self.max_distance = 0.5 # Astar replanning and obstacle avoidance starts here (Simulation)
        self.max_distance = 0.5 # Astar replanning and obstacle avoidance starts here (Real)

        # Movement Parameters
        self.move_speed = 3.0
        self.rotation_threshold = 0.1  # radians (~2.86 degrees)
        self.goal_pos_threshold = 0.25  # meters
        self.goal_orientation_threshold = 0.05  # radians, approximately 3 degrees

        # Thresholds for path following
        self.distance_threshold = 0.4
        self.orientation_threshold = 0.1  # Threshold for orientation check

        # Potential Field Constants
        self.k_a = 1.0  # Attractive force constant
        self.k_r = 0.05  # Repulsive force constant
        self.k_ro = 1.0  # For rotation while facing towards next goal direction
        self.rho_0 = 1.0  # Threshold distance for obstacles
        self.last_scan = None

        # Speed Limits
        # self.speed_up_lim = 0.5
        # self.speed_low_lim = 0.2
        # self.repulsive_speed_lim = -0.5
        # self.rotate_speed = 1.0

        self.speed_up_lim = 0.25
        self.speed_low_lim = 0.1
        self.repulsive_speed_lim = -0.3
        self.rotate_speed = 0.1

        # Replanning Control
        self.last_replan_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.replan_interval = 1.0  # seconds

        # Orientation Adjustment State
        self.is_adjusting_orientation = False  # New state variable

    ####################### Call_Back Functions Start ###################################

    def map_callback(self, msg):
        """Callback function for the '/map' topic. Processes incoming map data and inflates obstacles."""
        print("Received Map data...")
        self.map_data = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'z': msg.info.origin.position.z
            },
            'data': msg.data
        }
        self.grid = np.array(self.map_data['data']).reshape(
            self.map_data['height'], self.map_data['width'])

        # Inflate the obstacles
        structure = disk(self.threshold)  # Create a disk-shaped structure for dilation
        binary_occupancy_grid = np.where(
            self.grid == 100, 1, 0)  # Convert to binary grid
        dilated_grid = binary_dilation(
            binary_occupancy_grid, structure=structure)  # Inflate the walls
        self.grid = np.where(dilated_grid == 1, 100, 0)  # Reconvert to occupancy grid format
        print(f"Dilated Grid: \n{self.grid}")
        self.get_logger().info('Map data updated with threshold.')
        self.update_grid = np.copy(self.grid)

    ####################### For Simulation ###################################

    # def odom_callback(self, msg):
    #     """Callback function for the '/odom' topic. Updates the robot's current position and orientation."""
    #     pos = msg.pose.pose.position
    #     ori = msg.pose.pose.orientation
    #     self.current_pos_x = pos.x
    #     self.current_pos_y = pos.y
    #     _, _, self.orientation = euler_from_quaternion(
    #         [ori.x, ori.y, ori.z, ori.w])

    ####################### For Simulation ###################################
        
    def amcl_callback(self, msg): ## Change this accordingly to what the Datatype is
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.current_pos_x = pos.x
        self.current_pos_y = pos.y
        _, _, self.orientation = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    def laser_callback(self, msg):
        """Callback function for the '/scan' topic. Updates the latest LIDAR scan data."""
        # Reduced logging frequency
        # self.get_logger().info(f'Received Scan data')
        self.last_scan = msg  # Update last scan data

    def goal_pose_callback(self, msg):
        """Callback function for the '/goal_pose' topic. Receives the goal position and plans the path."""
        if self.map_data is None:
            self.get_logger().info('GoalPose_CB: Map data is not yet available.')
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        _, _, self.goal_orientation = euler_from_quaternion(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        start_pos = self.map_to_grid(
            self.current_pos_x, self.current_pos_y, self.map_data, "Start")
        goal_pos = self.map_to_grid(
            self.goal_x, self.goal_y, self.map_data, "Goal")
        path = self.a_star_search(start_pos, goal_pos, self.grid)

        # Simplify the path and update self.path
        # simplified_path = self.simplify_path(path)
        # self.path = self.further_simplify_path(simplified_path, segment_step=5)
        self.path = self.simplify_path(path)
        # self.path = self.further_simplify_path(simplified_path, segment_step=5)

        self.path_worldframe = self.grid_to_map(self.path, self.map_data)
        self.publish_path_points(self.path_worldframe)
        self.publish_path(self.path_worldframe)
        self.get_logger().info(f'A* Path in grid: {self.path}')
        self.get_logger().info(f'A* Path in world frame: {self.path_worldframe}')

    ####################### Call_Back Functions End ###################################

    def simplify_path(self, path):######################## Publsihing Path ############################################
        """Simplifies the path by removing unnecessary waypoints."""
        if not path or len(path) < 3:
            return path  # No need to simplify if the path is too short

        simplified_path = [path[0]]  # Always include the start point
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            current = path[i]
            next = path[i + 1]

            # Check if there's a change in both x and y, or if the direction changes
            if not ((prev[0] == current[0] == next[0]) or (prev[1] == current[1] == next[1])):
                simplified_path.append(current)

        simplified_path.append(path[-1])  # Always include the goal point
        return simplified_path

    def further_simplify_path(self, path, segment_step=5):
        """Further simplifies the path by segmenting and selecting every nth point."""
        if not path or len(path) < 3:
            return path  # No need to simplify if the path is too short

        final_path = [path[0]]  # Always start with the first point
        segment = [path[0]]  # Initialize the first segment

        for i in range(1, len(path)):
            prev = path[i - 1]
            current = path[i]

            # Check if current point is exactly one step away in either x or y direction
            if abs(current[0] - prev[0]) <= 1 and abs(current[1] - prev[1]) <= 1:
                segment.append(current)
            else:
                # If current point is farther away, process the segment
                if len(segment) > 1:
                    # Select every nth point from the segment
                    segment_simplified = [segment[j]
                                          for j in range(0, len(segment), segment_step)]
                    if segment[-1] not in segment_simplified:
                        segment_simplified.append(
                            segment[-1])  # Ensure the last point is included
                    final_path.extend(segment_simplified)
                else:
                    final_path.extend(segment)

                # Reset the segment with the current point
                segment = [current]

        # Process the last segment
        if len(segment) > 1:
            segment_simplified = [segment[j]
                                  for j in range(0, len(segment), segment_step)]
            if segment[-1] not in segment_simplified:
                segment_simplified.append(segment[-1])
            final_path.extend(segment_simplified)
        else:
            final_path.extend(segment)

        final_path.append(path[-1])  # Always end with the last point
        return final_path

    def map_to_grid(self, x, y, map_data, msg):
        """Converts world coordinates to grid indices."""
        mx = int(round((x - map_data['origin']['x']) / map_data['resolution']))
        my = int(round((y - map_data['origin']['y']) / map_data['resolution']))
        self.get_logger().info(
            f"Position of robot for {msg} on map: x: {mx}, y: {my}")
        return (mx, my)

    def grid_to_map(self, path_, map_data):
        """Converts grid indices back to world coordinates."""
        path_new = []
        origin_x = map_data['origin']['x']
        origin_y = map_data['origin']['y']
        resolution = map_data['resolution']
        for i in range(len(path_)):
            mx = origin_x + path_[i][0] * resolution
            my = origin_y + path_[i][1] * resolution
            path_new.append((mx, my))
        return path_new

    ####################### A Star Implementation Start ###################################

    def a_star_search(self, start, goal, grid):
        """Implements the A* search algorithm."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current, grid):
                new_cost = cost_so_far[current] + self.heuristic(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    heapq.heappush(open_set, (priority, next))
                    came_from[next] = current
        print(f"came_from len = {len(came_from)}")
        print(f"came_from = {came_from}")
        return self.reconstruct_path(came_from, start, goal)

    def heuristic(self, a, b):
        """Heuristic function using Euclidean distance."""
        return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def get_neighbors(self, pos, grid):
        """Retrieves valid neighbor positions for A* search."""
        directions = [(-1, -1), (-1, 0), (-1, 1),
                      (0, -1),         (0, 1),
                      (1, -1),  (1, 0),  (1, 1)]
        neighbors = []
        x, y = pos
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(grid[0]) and 0 <= ny < len(grid) and grid[ny][nx] != 100:
                neighbors.append((nx, ny))
        return neighbors
 
    def reconstruct_path(self, came_from, start, goal):
        """Reconstructs the path from the came_from dictionary."""
        current = goal
        path = []
        print(f"current = {current}, start = {start}")
        while current != start:
            path.append(current)
            current = came_from.get(current)  # Safely access dictionary keys
        path.append(start)
        path.reverse()
        return path

    ####################### A Star Implementation End ####################################
    ####################### Local Path Planning with Potential Field ###################################

    def update_map_with_lidar(self):
        """Updates the map with new obstacles detected by LIDAR and replans if necessary."""
        
        if self.map_data is None or self.last_scan is None:
            self.get_logger().info(
                'Map or Lidar data is not available for local planning.')
            return

        # Get robot's position on the grid
        robot_pos = self.map_to_grid(
            self.current_pos_x, self.current_pos_y, self.map_data, "Robot")

        # Create a set to store newly detected obstacle positions
        new_obstacle_positions = set()

        for i, distance in enumerate(self.last_scan.ranges):
            if distance < self.max_distance or distance == float('inf') or math.isnan(distance):
                continue

            # Calculate the angle of the lidar beam
            angle = self.last_scan.angle_min + i * self.last_scan.angle_increment

            # Calculate the position of the detected obstacle
            obstacle_x = robot_pos[0] + int(distance * math.cos(
                angle + self.orientation) / self.map_data['resolution'])
            obstacle_y = robot_pos[1] + int(distance * math.sin(
                angle + self.orientation) / self.map_data['resolution'])

            # Add the obstacle to the set, making sure it's within bounds
            if 0 <= obstacle_x < self.update_grid.shape[1] and 0 <= obstacle_y < self.update_grid.shape[0]:
                new_obstacle_positions.add((obstacle_x, obstacle_y))
                self.update_grid[obstacle_y, obstacle_x] = 100  # Mark as an obstacle

        # Iterate through the existing grid to find previous obstacles that are no longer present
        for y in range(self.update_grid.shape[0]):
            for x in range(self.update_grid.shape[1]):
                if self.update_grid[y, x] == 100 and (x, y) not in new_obstacle_positions:
                    # Clear previous obstacles that are not detected in the current scan
                    self.update_grid[y, x] = 0

        # Create a dilated version of the updated grid for path planning
        structure = disk(self.threshold)
        binary_occupancy_grid = np.where(self.update_grid == 100, 1, 0)
        dilated_grid = binary_dilation(binary_occupancy_grid, structure=structure)

        # Create a new variable for the dilated grid
        updated_dilated_grid = np.where(dilated_grid == 1, 100, 0)

        # Check if any point in the current path is now blocked
        path_blocked = False
        if self.path:
            for (x, y) in self.path:
                if 0 <= x < updated_dilated_grid.shape[1] and 0 <= y < updated_dilated_grid.shape[0]:
                    if updated_dilated_grid[y, x] == 100:
                        path_blocked = True
                        break

        # Only update the map and publish a new path if the current path is blocked
        if path_blocked:
            self.get_logger().info('Path blocked by an obstacle. Replanning path...')

            # Set the updated dilated grid as the current grid for path planning
            self.grid = updated_dilated_grid

            # Trigger re-planning using A* with the updated dilated map
            if self.goal_x is not None and self.goal_y is not None:
                start_pos = self.map_to_grid(
                    self.current_pos_x, self.current_pos_y, self.map_data, "Start")
                goal_pos = self.map_to_grid(
                    self.goal_x, self.goal_y, self.map_data, "Goal")
                path = self.a_star_search(start_pos, goal_pos, self.grid)

                # Simplify the path and update self.path
                # simplified_path = self.simplify_path(path)
                # self.path = self.further_simplify_path(simplified_path, segment_step=5)
                self.path = self.simplify_path(path)
                # self.path = self.further_simplify_path(simplified_path, segment_step=5)
                self.path_worldframe = self.grid_to_map(self.path, self.map_data)
                self.path_index = 0  # Reset the path index to start following the new path
                self.publish_path(self.path_worldframe)
                self.publish_path_points(self.path_worldframe)
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                print("Stopping because rerouting")

        else:
            print('Current path is clear. No replanning required.')

    def follow_path(self):
        """Controls the robot to follow the path using potential fields."""
        if self.is_adjusting_orientation:
            self.adjust_final_orientation()
            return

        if self.path_worldframe is None:
            return

        if self.path_index >= len(self.path_worldframe):
            self.is_adjusting_orientation = True
            self.adjust_final_orientation()
            return

        # Controlled replanning frequency
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_replan_time > self.replan_interval:
            self.update_map_with_lidar() # Comment this function to remove local path planner
            self.last_replan_time = current_time

        # self.get_logger().info(f'Moving Along the path index[{self.path_index}]')
        print("Moving along the path")
        goal_x, goal_y = self.path_worldframe[self.path_index]
        angle_diff = self.calculate_angular_difference(goal_x, goal_y)

        if self.path_index == len(self.path_worldframe) - 1:  # if it is the last goal, set different threshold
            if sqrt((self.current_pos_x - goal_x) ** 2 + (self.current_pos_y - goal_y) ** 2) < self.goal_pos_threshold:
                self.path_index += 1
                return
        else:
            if sqrt((self.current_pos_x - goal_x) ** 2 + (self.current_pos_y - goal_y) ** 2) < self.distance_threshold:
                self.path_index += 1
                return

        # self.get_logger().info(f'Calculating attractive forces')

        distance = self.calculate_distance(goal_x, goal_y)
        attraction_x, attraction_y = self.calculate_attractive_force(
            goal_x, goal_y)

        if self.last_scan:
            # self.get_logger().info(f'Last Scan data received for Repulsion forces')
            repulsion_x, repulsion_y, repulsion_angular = self.calculate_repulsive_forces(
                self.last_scan)
        else:
            repulsion_x, repulsion_y, repulsion_angular = 0.0, 0.0, 0.0

        if abs(attraction_y) > abs(attraction_x):
            total_force_x = abs(attraction_y) + repulsion_x
        else:
            total_force_x = abs(attraction_x) + repulsion_x
        
        total_force_y = repulsion_y
        angular_force = self.calculate_angular_correction(
            goal_x, goal_y) + repulsion_angular

        # Speed limits
        if total_force_x > self.speed_up_lim:
            total_force_x = self.speed_up_lim
        elif 0.0 < total_force_x < self.speed_low_lim:
            total_force_x = self.speed_low_lim
        elif total_force_x < self.repulsive_speed_lim:
            total_force_x = self.repulsive_speed_lim

        twist = Twist()
        if abs(angle_diff) > 0.785:  # If angle diff is greater than 45 degrees (0.785 radians), rotate and avoid
            twist.linear.x = 0.0
            twist.linear.y = total_force_y
            twist.angular.z = 0.6 * angular_force
        else:  # else move towards it
            twist.linear.x = total_force_x
            twist.linear.y = total_force_y
            twist.angular.z = angular_force

        self.publisher_.publish(twist)

    def calculate_distance(self, goal_x, goal_y):
        """Calculates the Euclidean distance to the goal."""
        distance = sqrt((goal_x - self.current_pos_x) ** 2 +
                        (goal_y - self.current_pos_y) ** 2)
        return distance

    def calculate_attractive_force(self, goal_x, goal_y):
        """Calculates attractive forces based on the goal position."""
        force_x = self.k_a * (goal_x - self.current_pos_x)
        force_y = self.k_a * (goal_y - self.current_pos_y)
        return force_x, force_y

    def calculate_angular_correction(self, goal_x, goal_y):
        """Calculates the angular correction needed to face the goal."""
        goal_direction = atan2(
            goal_y - self.current_pos_y, goal_x - self.current_pos_x)
        angle_difference = self.normalize_angle(goal_direction - self.orientation)
        return self.k_ro * angle_difference

    def calculate_angular_difference(self, goal_x, goal_y):
        """Calculates the angular difference between the current orientation and the goal."""
        goal_direction = atan2(
            goal_y - self.current_pos_y, goal_x - self.current_pos_x)
        angle_difference = self.normalize_angle(
            goal_direction - self.orientation)
        return angle_difference

    def adjust_final_orientation(self):
        """Adjusts the robot's orientation to match the goal orientation after reaching the positional goal."""
        print("Positional goal reached. Halting linear motion.")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0

        angle_difference = abs(self.normalize_angle(self.goal_orientation - self.orientation))

        if angle_difference > self.rotation_threshold:
            self.final_rotation_direction = self.determine_rotation_direction(
                self.goal_orientation)
            print(f"Adjusting orientation towards: {self.final_rotation_direction}")
            if self.final_rotation_direction == "right":
                twist.angular.z = -self.rotate_speed
                print("here1")
                print(self.rotate_speed)
            else:
                twist.angular.z = self.rotate_speed
                print("here")
                print(self.rotate_speed)
        else:
            print("Orientation goal reached. Halting rotation.")
            twist.angular.z = 0.0
            self.path_index = 0
            self.path = None
            self.path_worldframe = None
            self.is_adjusting_orientation = False  # Reset the state variable
            self.get_logger().info('Goal reached !!!')

        self.publisher_.publish(twist)

    def normalize_angle(self, angle):
        """Normalizes an angle to the range [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def determine_rotation_direction(self, goal_orientation):
        """Determines the direction to rotate to reach the goal orientation."""
        current_orientation = self.normalize_angle(self.orientation)
        goal_orientation = self.normalize_angle(goal_orientation)

        angle_difference = self.normalize_angle(
            goal_orientation - current_orientation)

        if angle_difference > 0:
            return "left"
        else:
            return "right"

    def calculate_repulsive_forces(self, msg):
        """Calculates repulsive forces based on LIDAR data."""
        force_x, force_y, angular_force = 0.0, 0.0, 0.0
        min_force_threshold = 0.1
        

        for i, distance in enumerate(msg.ranges):
            if distance > self.max_distance or distance == float('inf') or math.isnan(distance):
                continue

            intensity = (self.max_distance - distance) / self.max_distance

            angle = msg.angle_min + i * msg.angle_increment

            # Calculate obstacle position in robot frame
            obstacle_x = distance * math.cos(angle)
            obstacle_y = distance * math.sin(angle)

            # Avoid division by zero
            dist = max(distance, 0.01)

            # Repulsive force components
            repulsion_force = self.k_r * intensity
            force_x += repulsion_force * (-obstacle_x / dist)
            force_y += repulsion_force * (-obstacle_y / dist)

            # Angular force
            angular_contribution = - \
                (intensity * (i - len(msg.ranges) / 2)
                 / (len(msg.ranges) / 2))
            angular_force += angular_contribution * repulsion_force

        # Ensure forces are above minimum threshold
        if abs(force_x) < min_force_threshold:
            force_x = min_force_threshold * (1 if force_x >= 0 else -1)
        if abs(force_y) < min_force_threshold:
            force_y = min_force_threshold * (1 if force_y >= 0 else -1)

        return (force_x, force_y, angular_force)
    
    ####################### Local Path Planning with Potential Field ###################################
    ######################## Publsihing Path ############################################

    def publish_path_points(self, path):
        """Publishes the path as a PoseArray message."""
        if not path:
            self.get_logger().info('No path to publish.')
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"

        for (x, y) in path:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0  # Assuming the path is on the ground plane
            pose.orientation.w = 1.0  # Default orientation
            pose_array.poses.append(pose)

        self.path_point_publisher.publish(pose_array)
        self.get_logger().info('Path published successfully.')

    def publish_path(self, path):
        """Publishes the path as a Path message."""
        if not path:
            self.get_logger().info('No path to publish.')
            return

        path_msg = Path()
        current_time = self.get_clock().now().to_msg()
        path_msg.header.stamp = current_time
        path_msg.header.frame_id = "map"

        for (x, y) in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = current_time  # Use the same timestamp as the path header
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0  # Assuming the path is on the ground plane
            pose_stamped.pose.orientation.w = 1.0  # Default orientation

            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Path published successfully.')

    ######################## Publsihing Path ############################################

def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = OdometryMotionModel()

    while rclpy.ok():
        rclpy.spin_once(node)
        node.follow_path()
        time.sleep(0.01)  # Sleep for 10ms to control loop rate

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()