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

        self.state = 'FORWARD'
        self.state_timer = 0
        self.forward_ticks = 50
        self.turn_ticks = 20
        self.zigzag_direction = 1  # 1 = left, -1 = right

        self.timer = self.create_timer(0.1, self.navigate)

        # Area bounding box for max coverage
        self.min_x, self.max_x = -5, 5
        self.min_y, self.max_y = -5, 5
        self.total_cells = int((self.max_x - self.min_x) / self.grid_size) * int((self.max_y - self.min_y) / self.grid_size)

        self.get_logger().info("Improved Vacuum Cleaner Node Started")

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

        # Track visited grid cells
        cell_x = round(self.x / self.grid_size)
        cell_y = round(self.y / self.grid_size)
        self.visited.add((cell_x, cell_y))

        # Check coverage
        coverage = len(self.visited) / self.total_cells
        self.get_logger().info(f"Visited {len(self.visited)} cells. Coverage: {coverage*100:.1f}%")

        # Stop if area mostly covered
        if coverage >= 0.9:
            self.get_logger().info("Coverage complete! Stopping.")
            self.cmd_pub.publish(Twist())  # Stop the robot
            return

        twist = Twist()

        # Basic obstacle avoidance
        min_distance = min([r for r in self.laser_ranges if not math.isinf(r) and not math.isnan(r)], default=10.0)
        if min_distance < 0.4:
            self.get_logger().warn("Obstacle too close! Rotating.")
            twist.angular.z = 0.5
            self.cmd_pub.publish(twist)
            return

        # Main movement logic
        if self.state == 'FORWARD':
            twist.linear.x = 0.2
            self.state_timer += 1
            if self.state_timer > self.forward_ticks:
                self.state = 'TURN'
                self.state_timer = 0
        elif self.state == 'TURN':
            twist.angular.z = 0.8 * self.zigzag_direction
            self.state_timer += 1
            if self.state_timer > self.turn_ticks:
                self.state = 'FORWARD'
                self.state_timer = 0
                self.zigzag_direction *= -1  # Alternate turns

        self.cmd_pub.publish(twist)

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
