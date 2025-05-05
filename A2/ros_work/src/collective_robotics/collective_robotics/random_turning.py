import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RandomTurningWithAvoidance(Node):
    def __init__(self):
        super().__init__('random_turning_with_avoidance')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)
        self.get_logger().info("Random Turning with Obstacle Avoidance Started")
        
        self.linear_speed = 0.2  # Forward speed
        self.angular_speed = 1.0  # Angular speed for sharp turns
        self.max_distance = 0.5  # Minimum distance to obstacle before avoiding
        
        self.is_turning = False  # Flag to check if robot is turning after avoiding obstacle

    def laser_callback(self, msg: LaserScan):
        twist = Twist()

        # Get distance values from front, left, and right areas
        center_idx = len(msg.ranges) // 2
        front_range = min(msg.ranges[center_idx - 30:center_idx + 30], default=10.0)  # Check front 60 degrees
        left_range = min(msg.ranges[center_idx + 30:center_idx + 90], default=10.0)  # Check left
        right_range = min(msg.ranges[center_idx - 90:center_idx - 30], default=10.0)  # Check right

        if front_range < self.max_distance:  # If obstacle is ahead
            # Decide turn direction based on the left/right distance
            twist.linear.x = 0.0  # Stop moving forward
            if left_range > right_range:
                twist.angular.z = 0.5  # Turn left if the left side is clearer
                self.get_logger().info("Obstacle detected ahead! Turning left.")
            else:
                twist.angular.z = -0.5  # Turn right if the right side is clearer
                self.get_logger().info("Obstacle detected ahead! Turning right.")
            self.is_turning = True
        else:
            # Move straight if no obstacles are detected
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.is_turning = False

        # Publish the twist message for movement
        self.publisher_.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RandomTurningWithAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
