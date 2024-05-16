#!/usr/bin/python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from video import VideoStitcher  # Assuming you have a VideoStitcher class defined elsewhere
import numpy as np
import cv2

class ImageReceiver:
    def __init__(self):
        # Define the topics for left and right images
        topic_img_left = '/top_left_camera/image_color'
        topic_img_right = '/top_right_camera/image_raw'

        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.left_stamp = None
        self.right_stamp = None

        # Subscribe to image topics
        self.img_left_sub = rospy.Subscriber(topic_img_left, Image, self.img_left_callback)
        self.img_right_sub = rospy.Subscriber(topic_img_right, Image, self.img_right_callback)

    def img_left_callback(self, left_msg):
        self.left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='passthrough')
        self.left_stamp = left_msg.header.stamp
        # print(left_msg.header.stamp)
      

    def img_right_callback(self, right_msg):
        self.right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')
        self.right_stamp = right_msg.header.stamp
        print(self.right_stamp, self.left_stamp)

if __name__ == '__main__':
    rospy.init_node('panorama_real', anonymous=True)
    receiver = ImageReceiver()

    # rate = rospy.Rate(10)  # Adjust the publishing rate as needed
    while not rospy.is_shutdown():
        if receiver.left_image is not None and receiver.right_image is not None:
            # Assuming you have implemented VideoStitcher class elsewhere
            panorama = VideoStitcher(left_video_in_path=receiver.left_image, right_video_in_path=receiver.right_image)
            result = panorama.run()
            # print(result.shape)
            cv2.imshow("window", result)
            cv2.waitKey(1)
            # rospy.sleep(3)
            # if cv2.waitKey(1) == ord('p'):
            #     break
            # Publish the stitched image
            # result_msg = receiver.bridge.cv2_to_imgmsg(result, encoding="passthrough")
            # result_msg.header.stamp = receiver.left_stamp  # Use the stamp from the left image
            # receiver.pub.publish(result_msg)
        # rate.sleep()
    cv2.destroyAllWindows()
