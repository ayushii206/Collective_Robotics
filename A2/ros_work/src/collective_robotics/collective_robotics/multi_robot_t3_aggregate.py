import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt, degrees
import tf_transformations
import random

class SmartAggregator(Node):
    def __init__(self):
        super().__init__('smart_robot_aggregator')

        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('robot_count', 20)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value

        self.all_robots = [f"robot_{i}" for i in range(robot_count)]
        self.other_robots = [name for name in self.all_robots if name != self.robot_name]

        self.positions = {}
        self.my_x = self.my_y = self.my_yaw = 0.0

        self.is_stopped = False
        self.stop_time = None
        self.waiting_time = random.uniform(3.0, 7.0)

        self.escaping = False
        self.escape_start_time = None
        self.escape_duration = random.uniform(1.0, 2.0)

        self.roam_start = None
        self.ROAM_INTERVAL = random.randint(12, 20)  # Dynamic interval

        # Parameters
        self.threshold_robot = 0.5
        self.threshold_wall = 0.4
        self.front_angle_limit = 45  # widened cone

        self.last_scan = None

        self.create_subscription(LaserScan, f'/{self.robot_name}/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.my_odom_callback, 10)

        for other in self.other_robots:
            self.create_subscription(Odometry, f'/{other}/odom',
                                     lambda msg, other=other: self.odom_callback(msg, other), 10)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

    def my_odom_callback(self, msg):
        self.my_x = msg.pose.pose.position.x
        self.my_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.my_yaw) = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def odom_callback(self, msg, robot_name):
        self.positions[robot_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def laser_callback(self, msg):
        self.last_scan = msg

    def control_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        twist = Twist()

        if self.escaping:
            if now - self.escape_start_time >= self.escape_duration:
                self.escaping = False
                self.get_logger().info(f"[{self.robot_name}] Escape complete.")
            else:
                # Safe movement while escaping
                if self.last_scan:
                    obstacle_ahead = any(
                        dist < self.threshold_wall
                        for i, dist in enumerate(self.last_scan.ranges)
                        if 0.01 < dist < float('inf') and abs(degrees(self.last_scan.angle_min + i * self.last_scan.angle_increment)) <= self.front_angle_limit
                    )
                else:
                    obstacle_ahead = False

                if obstacle_ahead:
                    twist.linear.x = 0.0
                    twist.angular.z = self.escape_direction
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = self.escape_direction * 0.5

                self.cmd_vel_pub.publish(twist)
                return

        if self.is_stopped:
            if now - self.stop_time >= self.waiting_time:
                self.is_stopped = False
                self.start_escape(now)
                return
            else:
                self.publish_stop()
                return

        if self.last_scan is None:
            return

        # Wall detection
        wall_near = any(
            dist < self.threshold_wall
            for i, dist in enumerate(self.last_scan.ranges)
            if 0.01 < dist < float('inf') and abs(degrees(self.last_scan.angle_min + i * self.last_scan.angle_increment)) <= self.front_angle_limit
        )

        # Robot detection
        robot_near = False
        for i, dist in enumerate(self.last_scan.ranges):
            angle_deg = degrees(self.last_scan.angle_min + i * self.last_scan.angle_increment)
            if abs(angle_deg) > self.front_angle_limit:
                continue
            if dist == float('inf') or dist <= 0.01:
                continue

            obs_x = self.my_x + dist * cos(self.my_yaw + self.last_scan.angle_min + i * self.last_scan.angle_increment)
            obs_y = self.my_y + dist * sin(self.my_yaw + self.last_scan.angle_min + i * self.last_scan.angle_increment)

            for name, (x, y) in self.positions.items():
                if sqrt((x - obs_x)**2 + (y - obs_y)**2) < self.threshold_robot:
                    robot_near = True
                    break
            if robot_near:
                break

        # Cluster behavior
        if robot_near:
            self.is_stopped = True
            self.stop_time = now
            self.waiting_time = random.uniform(3.0, 7.0)
            self.get_logger().info(f"[{self.robot_name}] Stopped in cluster for {self.waiting_time:.1f}s.")
            self.publish_stop()
            return

        # Wall avoidance
        if wall_near:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            return

        # Occasional exploration
        if self.roam_start is None:
            self.roam_start = now
        elif now - self.roam_start > self.ROAM_INTERVAL:
            twist.angular.z = random.choice([-0.3, 0.3])
            self.roam_start = now
            self.ROAM_INTERVAL = random.randint(12, 20)
            self.get_logger().info(f"[{self.robot_name}] Exploring briefly.")
        else:
            twist.linear.x = 0.15
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
        self.escape_direction = random.choice([-0.5, 0.5])
        self.escape_duration = random.uniform(1.0, 2.0)
        self.get_logger().info(f"[{self.robot_name}] Escaping with turn {self.escape_direction} for {self.escape_duration:.1f}s")

def main(args=None):
    rclpy.init(args=args)
    node = SmartAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

