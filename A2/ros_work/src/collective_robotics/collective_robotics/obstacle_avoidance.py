import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.max_distance = 0.5
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Obstacle Avoider Node Started")

    def laser_callback(self, msg):
        ranges = list(msg.ranges)

        num_readings = len(ranges)
        left = ranges[:num_readings // 3]
        center = ranges[num_readings // 3:num_readings * 2 // 3]
        right = ranges[num_readings * 2 // 3:]

        min_left = min(left)
        min_center = min(center)
        min_right = min(right)

        twist = Twist()

        if min_center < self.max_distance:
            if min_left > min_right:
                twist.angular.z = self.angular_speed 
                self.get_logger().info("Obstacle ahead! Turning LEFT")
            else:
                twist.angular.z = -self.angular_speed  
                self.get_logger().info("Obstacle ahead! Turning RIGHT")
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.get_logger().info("Path is clear. Moving forward.")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
