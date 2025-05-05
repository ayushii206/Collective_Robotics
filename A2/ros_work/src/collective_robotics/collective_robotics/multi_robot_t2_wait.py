import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt
import tf_transformations
import random
from rclpy.parameter import Parameter

class SmartRobotClusterAvoider(Node):
    def __init__(self):
        super().__init__('smart_robot_cluster_avoider')

        self.declare_parameter('robot_name', 'robot_0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.all_robots = ["robot_0", "robot_1", "robot_2", "robot_3", "robot_4"]
        self.other_robots = [name for name in self.all_robots if name != self.robot_name]

        self.threshold_robot = 0.7
        self.threshold_wall = 0.5

        self.positions = {}
        self.is_stopped = False
        self.stop_time = None
        self.WAITING_TIME = 5.0

        self.escaping = False
        self.escape_start_time = None
        self.ESCAPE_DURATION = 2.0

        self.create_subscription(LaserScan, f'/{self.robot_name}/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.my_odom_callback, 10)

        for other in self.other_robots:
            self.create_subscription(Odometry, f'/{other}/odom', lambda msg, other=other: self.odom_callback(msg, other), 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        self.my_x = 0.0
        self.my_y = 0.0
        self.my_yaw = 0.0

    def my_odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.my_yaw) = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

    def odom_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions[robot_name] = (x, y)

    def laser_callback(self, msg):
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if self.escaping:
            if now - self.escape_start_time >= self.ESCAPE_DURATION:
                self.escaping = False
                self.get_logger().info(f"[{self.robot_name}] Done escaping, moving forward.")
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self.escape_direction
                self.cmd_vel_pub.publish(twist)
                return

        if self.is_stopped:
            if now - self.stop_time >= self.WAITING_TIME:
                self.get_logger().info(f"[{self.robot_name}] Done waiting, starting escape.")
                self.is_stopped = False
                self.start_escape(now)
            else:
                self.publish_stop()
                return

        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)
        angle = msg.angle_min + min_index * msg.angle_increment

        obs_x = self.my_x + min_distance * cos(self.my_yaw + angle)
        obs_y = self.my_y + min_distance * sin(self.my_yaw + angle)

        is_robot_near = False
        for name, (x, y) in self.positions.items():
            distance = sqrt((x - obs_x)**2 + (y - obs_y)**2)
            if distance < self.threshold_robot:
                is_robot_near = True
                break

        twist = Twist()

        if min_distance < self.threshold_wall:
            if is_robot_near:
                self.get_logger().info(f"[{self.robot_name}] Detected robot! Stopping and starting timer.")
                self.is_stopped = True
                self.stop_time = now
                self.publish_stop()
            else:
                self.get_logger().info(f"[{self.robot_name}] Wall detected! Turning...")
                twist.linear.x = 0.0
                twist.angular.z = 0.5
                self.cmd_vel_pub.publish(twist)
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def start_escape(self, now):
        self.escaping = True
        self.escape_start_time = now
        self.escape_direction = random.choice([-0.5, 0.5])  # Random left or right

def main(args=None):
    rclpy.init(args=args)
    node = SmartRobotClusterAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
