#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.vel = Twist()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Controller Node Initialized")

    def control(self, msg):
        if msg == 'Find':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.5
            self.get_logger().info("Finding Path")
        elif msg == 'Left':
            self.vel.linear.x = 0.08
            self.vel.angular.z = 0.18
            self.get_logger().info("Adjusting Left")
        elif msg == 'Right':
            self.vel.linear.x = 0.08
            self.vel.angular.z = -0.18
            self.get_logger().info("Adjusting Right")
        elif msg == 'Center':
            self.vel.linear.x = 0.2
            self.vel.angular.z = 0.0
            self.get_logger().info("Going Straight")
        else:
            self.get_logger().warn(f"Unknown command: {msg}")
            return
        
        self.publisher.publish(self.vel)
        self.get_logger().info(f"Published velocity: {self.vel.linear.x}, {self.vel.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    control = Controller()

    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        control.get_logger().info('Shutting down')

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
