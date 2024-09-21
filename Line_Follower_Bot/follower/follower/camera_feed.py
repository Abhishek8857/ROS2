#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Feed(Node):
    def __init__(self):
        super().__init__('camera_feed')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10)
        self.subscriptions


    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        cv2.imshow('Camera Feed', self.cv_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()
            
            
def main(args=None):
    rclpy.init(args=args)
    camera_feed = Feed()
    
    try:
        rclpy.spin(camera_feed)
    except KeyboardInterrupt:
        camera_feed.get_logger().info('Shutting Down.')
        
    camera_feed.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
