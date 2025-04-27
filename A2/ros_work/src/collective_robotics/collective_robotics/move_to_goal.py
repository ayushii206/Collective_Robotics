import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import math

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.laser_ranges = []

        # Goal Position Variables
        self.goal_x = None
        self.goal_y = None

        self.goal_reached = False

        # Timer to control the movement
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

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False  # Reset when a new goal is received
        self.get_logger().info(f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def navigate(self):
        if self.goal_reached or self.goal_x is None or self.goal_y is None:
            return

        if not self.laser_ranges:
            return  # Wait for laser data

        twist = Twist()

        # Compute distance to goal
        distance_to_goal = math.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)

        if distance_to_goal < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Goal Reached!")
            self.goal_reached = True
            return

        # Compute angle to goal
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_error = self.normalize_angle(angle_to_goal - self.yaw)

        # Obstacle avoidance
        min_distance = min([r for r in self.laser_ranges if not math.isinf(r) and not math.isnan(r)], default=10.0)

        if min_distance < 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn left if obstacle too close
            self.get_logger().info("Obstacle detected! Turning left.")
        else:
            if abs(angle_error) > 0.2:
                twist.linear.x = 0.0
                twist.angular.z = 0.5 * angle_error  # Rotate towards the goal
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0  # Move straight towards goal

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()