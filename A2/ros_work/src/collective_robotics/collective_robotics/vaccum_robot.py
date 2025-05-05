import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class VacuumCleaner(Node):
    def __init__(self):
        super().__init__('vacuum_cleaner')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.laser_ranges = []

        self.visited = set()
        self.grid_size = 0.5

        self.moving_forward = True
        self.zigzag_direction = 1
        self.forward_timer = 0
        self.max_forward_time = 50

        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info("Vacuum Cleaner Node Started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion(orientation_q)

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges

    def navigate(self):
        if not self.laser_ranges:
            return 

        cell_x = round(self.x / self.grid_size)
        cell_y = round(self.y / self.grid_size)
        self.visited.add((cell_x, cell_y))

        twist = Twist()

        min_distance = min([r for r in self.laser_ranges if not math.isinf(r) and not math.isnan(r)], default=10.0)

        if min_distance < 0.4:
            twist.linear.x = 0.0
            twist.angular.z = 0.5  
            self.get_logger().info("Obstacle detected! Turning.")
        else:
            if self.forward_timer < self.max_forward_time:
                twist.linear.x = 0.2
                twist.angular.z = 0.2 * self.zigzag_direction
                self.forward_timer += 1
            else:
                self.zigzag_direction *= -1
                self.forward_timer = 0

        self.cmd_pub.publish(twist)

        self.get_logger().info(f"Visited {len(self.visited)} cells.")

    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw


def main(args=None):
    rclpy.init(args=args)
    node = VacuumCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
