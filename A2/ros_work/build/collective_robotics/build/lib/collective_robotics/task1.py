import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Obstacle Avoidance Node Started')

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)

        cmd = Twist()
        if min_distance < 0.5:
            self.get_logger().info('Obstacle detected! Turning...')
            cmd.angular.z = 0.5  # Rotate to avoid
        else:
            self.get_logger().info('Path is clear. Moving forward...')
            cmd.linear.x = 0.2  # Move forward

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
