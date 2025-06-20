import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt, degrees
import tf_transformations

class SmartRobotAvoider(Node):
    def __init__(self):
        super().__init__('smart_robot_avoider')

        self.declare_parameter('robot_name', 'robot_0')
        self.declare_parameter('robot_count', 20)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value

        self.all_robots = [f"robot_{i}" for i in range(robot_count)]
        self.other_robots = [name for name in self.all_robots if name != self.robot_name]

        self.threshold_robot = 0.6  # meters
        self.front_angle_limit = 20  # degrees from front
        self.positions = {}
        self.my_x, self.my_y, self.my_yaw = 0.0, 0.0, 0.0
        self.stopped = False

        self.create_subscription(LaserScan, f'/{self.robot_name}/base_scan', self.laser_callback, 10)
        self.create_subscription(Odometry, f'/{self.robot_name}/odom', self.my_odom_callback, 10)

        for other in self.other_robots:
            self.create_subscription(
                Odometry,
                f'/{other}/odom',
                lambda msg, other=other: self.odom_callback(msg, other),
                10
            )

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
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions[robot_name] = (x, y)

    def laser_callback(self, msg):
        twist = Twist()

        if self.stopped:
            self.cmd_vel_pub.publish(twist)
            return

        # Scan only in forward cone
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = degrees(angle)

            # Check only ±20° cone
            if abs(angle_deg) > self.front_angle_limit:
                continue

            if distance == float('inf') or distance <= 0.01:
                continue

            # Calculate the obstacle point in world frame
            obs_x = self.my_x + distance * cos(self.my_yaw + angle)
            obs_y = self.my_y + distance * sin(self.my_yaw + angle)

            # Match with known robot positions
            for name, (x, y) in self.positions.items():
                dist = sqrt((x - obs_x) ** 2 + (y - obs_y) ** 2)
                if dist < self.threshold_robot:
                    self.get_logger().info(f"[{self.robot_name}] Stopping due to robot: {name}")
                    self.stopped = True
                    self.cmd_vel_pub.publish(twist)
                    return

        # If nothing in cone is close → keep moving forward
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SmartRobotAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

