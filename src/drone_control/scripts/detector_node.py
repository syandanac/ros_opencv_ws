#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class OrangeDetector:
    def __init__(self):
        rospy.init_node('orange_detector_node')
        self.bridge = CvBridge()
        
        # Subscribe to your specific downcam topic
        self.image_sub = rospy.Subscriber("roscam_down/down_cam/down_image_raw", Image, self.callback)
        self.blob_pub = rospy.Publisher("/blob/point_blob", Point, queue_size=1)

        # HSV Range for Orange in Gazebo (Adjust if needed)
        self.orange_min = np.array([5, 100, 100])
        self.orange_max = np.array([25, 255, 255])

    def callback(self, data):
        try:
            # Convert ROS image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        h, w, _ = cv_image.shape
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.orange_min, self.orange_max)
        
        # Calculate Moments for the center of the blob
        M = cv2.moments(mask)
        point_msg = Point()

        if M['m00'] > 500:  # Noise filter
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # Normalize coordinates to range [-1.0, 1.0]
            # x: horizontal error, y: vertical error
            point_msg.x = (cx - (w / 2.0)) / (w / 2.0)
            point_msg.y = (cy - (h / 2.0)) / (h / 2.0)
            point_msg.z = 1.0  # Object detected flag
            
            # Draw for debugging (optional - can be seen via rqt_image_view)
            cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
        else:
            point_msg.z = 0.0  # Not detected

        self.blob_pub.publish(point_msg)
        # cv2.imshow("Debug View", cv_image) # Uncomment to see visual if running locally
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = OrangeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass