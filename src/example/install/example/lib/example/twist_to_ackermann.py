#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # or your twist topic
            self.twist_callback,
            10)
        
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_steering_controller/reference',
            10)
        
        # Get wheelbase from parameters or set default
        self.declare_parameter('wheelbase', 0.6)
        self.wheelbase = self.get_parameter('wheelbase').value

    def twist_callback(self, msg):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = 'base_link'
        
        # Convert twist to ackermann
        ackermann_msg.drive.speed = msg.linear.x
        
        # Calculate steering angle from angular velocity
        # steering_angle = atan(angular_z * wheelbase / linear_x)
        if abs(msg.linear.x) > 0.01:
            ackermann_msg.drive.steering_angle = \
                (msg.angular.z * self.wheelbase) / msg.linear.x
        else:
            ackermann_msg.drive.steering_angle = 0.0
        
        self.publisher.publish(ackermann_msg)

def main():
    rclpy.init()
    node = TwistToAckermann()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()