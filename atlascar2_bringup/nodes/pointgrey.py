#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from wfov_camera_msgs.msg import WFOVImage
from cv_bridge import CvBridge
import cv2

class ImageConverter:
    def __init__(self):
        rospy.init_node('image_converter', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/top_left_camera/image_color', WFOVImage, self.image_callback)
        self.image_pub = rospy.Publisher('/top_left_camera/image', Image, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert WFOVImage to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting WFOVImage to OpenCV image: %s", e)
            return

        # Convert OpenCV image to Image message
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_msg.header = msg.header

        # Publish the converted Image message
        self.image_pub.publish(image_msg)

if __name__ == '__main__':
    try:
        converter = ImageConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
