import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt
import tf_transformations
import random
from rclpy.parameter import Parameter

class SmartSwarmRobot(Node):
    def __init__(self):
        super().__init__('smart_swarm_robot')

        self.declare_parameter('robot_name', 'robot_0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.all_robots = ["robot_0", "robot_1", "robot_2", "robot_3", "robot_4"]
        self.other_robots = [name for name in self.all_robots if name != self.robot_name]

        # State machine
        self.state = "MOVING_FORWARD"
        self.wait_start_time = None
        self.WAIT_DURATION = 5.0  # seconds

        self.positions = {}

        # Subscriptions
        self.create_subscription(LaserScan, f'/{self.robot_name}/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.my_odom_callback, 10)

        for other in self.other_robots:
            self.create_subscription(Odometry, f'/{other}/odom', lambda msg, other=other: self.odom_callback(msg, other), 10)

        # Publisher
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

        min_distance = min(msg.ranges)

        # 1. Emergency wall avoidance first
        if min_distance < 0.4:
            self.get_logger().warn(f"[{self.robot_name}] Wall too close! Avoiding...")
            self.avoid_wall()
            return

        # 2. Check robot proximity if not wall avoiding
        robot_nearby = self.detect_nearby_robot()

        if self.state == "MOVING_FORWARD":
            if robot_nearby:
                self.get_logger().info(f"[{self.robot_name}] Robot detected! Stopping for 5 seconds...")
                self.state = "WAITING_STOP"
                self.wait_start_time = now
                self.publish_stop()
            else:
                self.move_forward()

        elif self.state == "WAITING_STOP":
            if now - self.wait_start_time >= self.WAIT_DURATION:
                self.get_logger().info(f"[{self.robot_name}] Done waiting! Escaping left...")
                self.state = "ESCAPE_LEFT"
            else:
                self.publish_stop()

        elif self.state == "ESCAPE_LEFT":
            self.escape_left()

    def detect_nearby_robot(self):
        for name, (x, y) in self.positions.items():
            distance = sqrt((x - self.my_x)**2 + (y - self.my_y)**2)
            if distance < 0.3:  
                return True
        return False

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def escape_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Turn left
        self.cmd_vel_pub.publish(twist)
        # After a small time, go back to moving forward
        self.state = "MOVING_FORWARD"

    def avoid_wall(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = random.choice([-1.0, 1.0])  # Turn randomly
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SmartSwarmRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
