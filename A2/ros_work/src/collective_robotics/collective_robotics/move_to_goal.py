import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math
import random

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.laser_ranges = []

        self.goal_x = None
        self.goal_y = None

        self.goal_reached = False

        self.timer = self.create_timer(0.1, self.navigate)

        self.initial_position_logged = False
        self.get_logger().info("Goal Navigator Node Started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        if not self.initial_position_logged:
            self.get_logger().info(f"Initial position: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.2f}°")
            self.initial_position_logged = True

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges

    def generate_random_goal(self):
        """Generate a random goal pose within a moderate distance from the robot's current position."""
        min_distance = 1.0  
        max_distance = 3.0  

        goal_distance = random.uniform(min_distance, max_distance)  
        goal_angle = random.uniform(-math.pi, math.pi) 
        
        goal_x = self.x + goal_distance * math.cos(goal_angle)
        goal_y = self.y + goal_distance * math.sin(goal_angle)
        
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_reached = False 
        self.get_logger().info(f"New goal generated: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def navigate(self):
        if self.goal_reached or self.goal_x is None or self.goal_y is None:
            return

        if not self.laser_ranges:
            return  

        twist = Twist()

        distance_to_goal = math.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)

        if distance_to_goal < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Goal Reached!")
            self.goal_reached = True
            self.generate_random_goal()
            return

        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_error = self.normalize_angle(angle_to_goal - self.yaw)

        front_dist = min([r for r in self.laser_ranges if not math.isinf(r) and not math.isnan(r)], default=10.0)
        center_idx = len(self.laser_ranges) // 2
        front_window = 30  
        side_window = 60  

        front_ranges = self.laser_ranges[center_idx - front_window:center_idx + front_window]
        left_ranges = self.laser_ranges[center_idx + front_window:center_idx + front_window + side_window]
        right_ranges = self.laser_ranges[center_idx - front_window - side_window:center_idx - front_window]

        front_dist = min([r for r in front_ranges if not (r == float('inf') or r != r)], default=10.0)
        left_dist = min([r for r in left_ranges if not (r == float('inf') or r != r)], default=10.0)
        right_dist = min([r for r in right_ranges if not (r == float('inf') or r != r)], default=10.0)

        if front_dist < 0.5:
            twist.linear.x = 0.0
            if left_dist > right_dist:
                twist.angular.z = 0.5 
                #self.get_logger().info("Obstacle ahead! Turning left.")
            else:
                twist.angular.z = -0.5  # Turn right
                #self.get_logger().info("Obstacle ahead! Turning right.")
        elif front_dist < 1.0:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            #self.get_logger().info("Caution! Slowing down.")
        else:
            if abs(angle_error) > 0.2:
                twist.linear.x = 0.0
                twist.angular.z = 0.5 * angle_error 
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0 

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    
    node.generate_random_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
