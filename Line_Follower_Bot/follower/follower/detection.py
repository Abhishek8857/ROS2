#!usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class Detection(Node):
    def __init__(self):
        super().__init__('detection')        
        self.bridge = CvBridge()
        self.video_data = []
        
        if not os.path.exists('/home/abhishek/workspaces/line_follower_ws/src/recordings'):
            os.makedirs('/home/abhishek/workspaces/line_follower_ws/src/recordings')
        
        # Record Frames
        self.video_writer = cv2.VideoWriter(filename='/home/abhishek/workspaces/line_follower_ws/src/recordings/detection.avi', 
                                            fourcc=cv2.VideoWriter.fourcc(*'XVID'), 
                                            fps=30, 
                                            frameSize=(640, 480))

        
    def detection(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.blurred = cv2.GaussianBlur(self.cv_image, ksize=(3, 3), sigmaX=0.1, sigmaY=0.1)
        self.hsv_image = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)
        
        self.height, self.width, self.length = self.cv_image.shape
                    
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(self.hsv_image, lowerb=lower_red, upperb=upper_red)
        
        # Remove top part of the mask
        y_limit = int(self.height * 0.4)
        mask[:y_limit] = 0
        
        # Clean up the noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Get moments
        moments = cv2.moments(mask)
        
        try:
            # Get the centroids
            self.centroid_x = int(moments['m10']/moments['m00'])
            self.centroid_y = int(moments['m01']/moments['m00'])
            
            # Get contours and rectangle co-ordinates
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(max(contours, key=cv2.contourArea)) 
            
                # Plot the rectangle on the image
                cv2.rectangle(
                    img=self.cv_image,
                    pt1=(rect_x, rect_y),
                    pt2=(rect_x + rect_w, rect_y + rect_h),
                    color=(0, 255, 0),
                    thickness=2
                )
                
                # Put text on the bounding box drawn
                cv2.putText(
                    img=self.cv_image,
                    text='Detected Path',
                    org=(rect_x, rect_y - 8),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale=0.4, 
                    color=(0, 255, 0),
                    lineType=cv2.LINE_AA
                )       
                
                # DEBUG - Plot the Centroid
                # cv2.circle(
                #     img=self.cv_image,
                #     center=(self.centroid_x, self.centroid_y),
                #     radius=3,
                #     thickness=-1,
                #     color=(0, 255, 0)
                # )    
                
                # DEBUG - Plot the Image Center
                # cv2.circle(
                #     img=self.cv_image,
                #     center=(round(self.width/2), round(self.height/2)),
                #     radius=3,
                #     thickness=-1,
                #     color=(255, 255, 255)
                # )    
                
                # Set Tolerance for deviation from Path
                tolerance = 20
                image_center_x = round(self.width/2)
                deviation_from_center = image_center_x - self.centroid_x
                
            
                self.camera_feed()                                
                # Controller Logic
                if deviation_from_center >= tolerance:
                    return "Left"
                elif deviation_from_center <= -tolerance:
                    return "Right"
                elif deviation_from_center < tolerance or deviation_from_center > tolerance:
                    return "Center"
            
        except ZeroDivisionError:
            cv2.putText(
                img = self.cv_image, 
                text='[WARNING] No Path Detected. Searching .... ',
                org=(100, 150),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.6,
                color=(0, 255, 255),
                lineType=cv2.LINE_AA
            )
            self.camera_feed()
            return "Find"
    
    
    def camera_feed(self):
        cv2.imshow('Camera Feed', self.cv_image)
        self.video_data.append(self.cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    detected_feed = Detection()
    
    try:
        rclpy.spin(detected_feed)
    except KeyboardInterrupt:
        detected_feed.get_logger().info('Shutting down')
        
    detected_feed.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
    
    
        
if __name__ == '__main__':
    main()
    
    
    