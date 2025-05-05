import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.laser_callback,
            10
        )
        self.get_logger().info("Wall Follower Node Started")

    def laser_callback(self, msg: LaserScan):
        ranges = msg.ranges
        print(ranges)
        num_readings = len(ranges)
        # print(num_readings)
        right_idx = num_readings // 4       
        front_idx = num_readings // 2      
        # print("right_idx :", right_idx, "front_idx :", front_idx)
        def safe_min(data):
            return min([r for r in data if not (r == float('inf') or r != r)], default=10.0)

        front_window = 10
        right_window = 10

        front_dist = safe_min(ranges[front_idx - front_window : front_idx + front_window])
        right_dist = safe_min(ranges[right_idx - right_window : right_idx + right_window])

        twist = Twist()

        target_wall_dist = 0.6    
        front_thresh = 0.5        
        kp = 0.5                   

        if front_dist < front_thresh:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info("Wall Follower: Obstacle ahead, turning left")
        else:
            error = target_wall_dist - right_dist
            twist.linear.x = 0.2
            twist.angular.z = kp * error
            self.get_logger().info(f"Wall Follower: Following wall | Right dist: {right_dist:.2f}, error: {error:.2f}")

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
