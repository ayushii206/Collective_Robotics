import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt, degrees
import tf_transformations
import random

class SmartRobotClusterAvoider(Node):
    def __init__(self):
        super().__init__('smart_robot_cluster_avoider')

        # Declare parameters
        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('robot_count', 20)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value

        self.all_robots = [f"robot_{i}" for i in range(robot_count)]
        self.other_robots = [name for name in self.all_robots if name != self.robot_name]

        self.threshold_robot = 0.7     # Distance to other robot to stop
        self.threshold_wall = 0.5      # Distance to wall to turn
        self.front_angle_limit = 30    # degrees for forward cone

        self.positions = {}
        self.is_stopped = False
        self.stop_time = None
        self.WAITING_TIME = 5.0

        self.escaping = False
        self.escape_start_time = None
        self.ESCAPE_DURATION = 2.0

        self.my_x = 0.0
        self.my_y = 0.0
        self.my_yaw = 0.0

        self.last_scan = None

        self.create_subscription(LaserScan, f'/{self.robot_name}/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.my_odom_callback, 10)

        for other in self.other_robots:
            self.create_subscription(Odometry, f'/{other}/odom',
                                     lambda msg, other=other: self.odom_callback(msg, other), 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        # Timer to check and act
        self.create_timer(0.1, self.control_loop)

    def my_odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.my_yaw) = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def odom_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions[robot_name] = (x, y)

    def laser_callback(self, msg):
        self.last_scan = msg

    def control_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        twist = Twist()

        if self.escaping:
            if now - self.escape_start_time >= self.ESCAPE_DURATION:
                self.escaping = False
                self.get_logger().info(f"[{self.robot_name}] Done escaping.")
            else:
                twist.angular.z = self.escape_direction
                self.cmd_vel_pub.publish(twist)
                return

        if self.is_stopped:
            if now - self.stop_time >= self.WAITING_TIME:
                self.is_stopped = False
                self.start_escape(now)
                return
            else:
                self.publish_stop()
                return

        if self.last_scan is None:
            return

        # Check all ranges in a forward cone for robots or walls
        msg = self.last_scan
        robot_near = False
        wall_near = False

        for i, distance in enumerate(msg.ranges):
            angle = degrees(msg.angle_min + i * msg.angle_increment)
            if abs(angle) > self.front_angle_limit:
                continue
            if distance == float('inf') or distance <= 0.01:
                continue

            obs_x = self.my_x + distance * cos(self.my_yaw + msg.angle_min + i * msg.angle_increment)
            obs_y = self.my_y + distance * sin(self.my_yaw + msg.angle_min + i * msg.angle_increment)

            if distance < self.threshold_wall:
                wall_near = True

            for name, (x, y) in self.positions.items():
                if sqrt((x - obs_x) ** 2 + (y - obs_y) ** 2) < self.threshold_robot:
                    robot_near = True
                    break
            if robot_near:
                break

        if robot_near:
            self.get_logger().info(f"[{self.robot_name}] Detected nearby robot. Stopping.")
            self.is_stopped = True
            self.stop_time = now
            self.publish_stop()
        elif wall_near:
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
        else:
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def start_escape(self, now):
        self.escaping = True
        self.escape_start_time = now
        self.escape_direction = random.choice([-0.5, 0.5])
        self.get_logger().info(f"[{self.robot_name}] Escaping with angular.z = {self.escape_direction}")

def main(args=None):
    rclpy.init(args=args)
    node = SmartRobotClusterAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

