#!/usr/bin/python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber
from video import VideoStitcher
import numpy as np
import multiprocessing

class ImageReceiver:
    def __init__(self):
        topic_img_left = '/top_left_camera/image_raw'
        topic_img_right = '/top_right_camera/image_raw'
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.stamp = None

        # Subscribe to image topics
        self.img_left_sub = Subscriber(topic_img_left, Image)
        self.img_right_sub = Subscriber(topic_img_right, Image)
        self.pub = rospy.Publisher("/panorama_img", Image, queue_size=1)

        # Synchronize image topics
        self.sync = TimeSynchronizer([self.img_left_sub, self.img_right_sub], 1)
        self.sync.registerCallback(self.img_callback)

    def img_callback(self, left_msg, right_msg):
        self.left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='passthrough')
        self.right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')
        self.stamp = left_msg.header.stamp

        

if __name__ == '__main__':
    rospy.init_node('panorama', anonymous=True)
    receiver = ImageReceiver()

    # Multiprocessing
    # multiprocessing.Process(target=receiver.run).start()

    rate = rospy.Rate(10)  # Adjust the publishing rate as needed
    while not rospy.is_shutdown():
        if receiver.left_image is not None and receiver.right_image is not None:
            panorama = VideoStitcher(left_video_in_path=receiver.left_image, right_video_in_path=receiver.right_image)
            result = panorama.run()
            print(result.shape)
            result_msg = receiver.bridge.cv2_to_imgmsg(result, encoding="passthrough")
            result_msg.header.stamp = receiver.stamp
            receiver.pub.publish(result_msg)
        rate.sleep()