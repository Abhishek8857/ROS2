#!usr/bin/env python3

import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from rclpy.node import Node
from .detection import Detection
from .controller import Controller


class Follow(Node):
    def __init__(self):
        super().__init__('follower')
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.follow_path,
            10
        )
        self.subscriptions
        
        self.detection = Detection()
        self.robot_control = Controller()
        
        
    def follow_path(self, msg):
        direction = self.detection.detection(msg)
        self.robot_control.control(direction)
        
    def save_video(self):
        if self.detection.video_writer is not None:
            for frame in self.detection.video_data:
                self.detection.video_writer.write(frame)
            self.detection.video_writer.release()
            self.get_logger().info('Saving Video File')

def main(args=None):
    rclpy.init(args=args)
    follow_node = Follow()
    
    try:
        rclpy.spin(follow_node)
    except KeyboardInterrupt:
        follow_node.save_video()
        follow_node.get_logger().info('Shutting Down')
    
    follow_node.detection.destroy_node()
    follow_node.robot_control.destroy_node()
    follow_node.destroy_node()
    
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
