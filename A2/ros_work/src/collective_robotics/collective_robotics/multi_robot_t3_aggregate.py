import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt
import tf_transformations
import random

class SmartRobotClusterer(Node):
    def __init__(self):
        super().__init__('smart_robot_clusterer')

        self.declare_parameter('robot_name', 'robot_0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.all_robots = [f"robot_{i}" for i in range(20)]
        self.other_robots = [name for name in self.all_robots if name != self.robot_name]

        self.threshold_robot = 0.3
        self.threshold_wall = 0.5

        self.positions = {}
        self.is_stopped = False
        self.stop_time = None
        self.WAITING_TIME = 8.0  # wait in cluster longer

        self.escaping = False
        self.escape_start_time = None
        self.ESCAPE_DURATION = 1.0  # short escape to stay near cluster

        self.my_x = 0.0
        self.my_y = 0.0
        self.my_yaw = 0.0

        self.create_subscription(LaserScan, f'/{self.robot_name}/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.my_odom_callback, 10)

        for other in self.other_robots:
            self.create_subscription(Odometry, f'/{other}/odom', lambda msg, other=other: self.odom_callback(msg, other), 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

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
        self.positions[robot_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def laser_callback(self, msg):
        now = self.get_clock().now().seconds_nanoseconds()[0]

        if self.escaping:
            if now - self.escape_start_time >= self.ESCAPE_DURATION:
                self.escaping = False
                self.get_logger().info(f"[{self.robot_name}] Finished escaping.")
            else:
                twist = Twist()
                twist.linear.x = 0.1  # gentle forward motion
                twist.angular.z = self.escape_direction  # curved escape
                self.cmd_vel_pub.publish(twist)
                return

        if self.is_stopped:
            if now - self.stop_time >= self.WAITING_TIME:
                self.get_logger().info(f"[{self.robot_name}] Done waiting. Escaping...")
                self.is_stopped = False
                self.start_escape(now)
            else:
                self.publish_stop()
                return

        # Analyze laser data
        min_distance = min(msg.ranges)
        min_index = msg.ranges.index(min_distance)
        angle = msg.angle_min + min_index * msg.angle_increment

        obs_x = self.my_x + min_distance * cos(self.my_yaw + angle)
        obs_y = self.my_y + min_distance * sin(self.my_yaw + angle)

        is_robot_near = False
        for name, (x, y) in self.positions.items():
            distance = sqrt((x - obs_x) ** 2 + (y - obs_y) ** 2)
            if distance < self.threshold_robot:
                is_robot_near = True
                break

        twist = Twist()

        if min_distance < self.threshold_wall:
            if is_robot_near:
                self.get_logger().info(f"[{self.robot_name}] Robot nearby — stopping.")
                self.is_stopped = True
                self.stop_time = now
                self.publish_stop()
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.5  # wall avoidance
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
        self.escape_direction = random.choice([-0.3, 0.3])  # small turn angle

def main(args=None):
    rclpy.init(args=args)
    node = SmartRobotClusterer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
