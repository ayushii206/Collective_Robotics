import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class CollisionAvoider(Node):
    def __init__(self):
        super().__init__('collision_avoider')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.laser_callback,
            10
        )
        self.get_logger().info("Collision Avoidance Node Started")

    def laser_callback(self, msg: LaserScan):
        ranges = msg.ranges
        print(ranges[0])

        center_idx = len(ranges) // 2
        front_window = 30  
        side_window = 60   

        front_ranges = ranges[center_idx - front_window:center_idx + front_window]
        left_ranges = ranges[center_idx + front_window:center_idx + front_window + side_window]
        right_ranges = ranges[center_idx - front_window - side_window:center_idx - front_window]

        front_dist = min([r for r in front_ranges if not (r == float('inf') or r != r)], default=10.0)
        left_dist = min([r for r in left_ranges if not (r == float('inf') or r != r)], default=10.0)
        right_dist = min([r for r in right_ranges if not (r == float('inf') or r != r)], default=10.0)

        twist = Twist()

        if front_dist < 0.5:
            twist.linear.x = 0.0
            if left_dist > right_dist:
                twist.angular.z = 0.5  
                self.get_logger().info("Obstacle ahead! Turning left.")
            else:
                twist.angular.z = -0.5 
                self.get_logger().info("Obstacle ahead! Turning right.")
        elif front_dist < 1.0:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.get_logger().info("Caution! Slowing down.")
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Path clear. Moving forward.")

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
