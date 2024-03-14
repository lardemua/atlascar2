#!/usr/bin/python3

from panorama import Panaroma
import imutils
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber
from stitching import Stitcher, AffineStitcher
from video import VideoStitcher
import numpy as np
import time
from queue import Queue
import threading
from panorama import Panaroma


class ImageReceiver:
    def __init__(self):
        topic_img_left = '/top_left_camera/image_raw'
        topic_img_right = '/top_right_camera/image_raw'
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.left_image_2 = None
        self.right_image_2 = None
        self.left_image_ready = False
        self.right_image_ready = False
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
        self.stamp = left_msg.header.stamp
     

        
        self.right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='passthrough')
    
class ImageProcessingThread(threading.Thread):
    def __init__(self, receiver):
        threading.Thread.__init__(self)
        self.receiver = receiver

    def run(self):
        while not rospy.is_shutdown():
            if self.receiver.left_image is not None and self.receiver.right_image is not None:
                panorama = VideoStitcher(left_video_in_path=self.receiver.left_image, right_video_in_path=self.receiver.right_image)
                result = panorama.run()
                result_msg = self.receiver.bridge.cv2_to_imgmsg(result, encoding="passthrough")
                result_msg.header.stamp = self.receiver.stamp
                self.receiver.pub.publish(result_msg)
                

if __name__ == '__main__':
    rospy.init_node('panorama', anonymous=True)
    
    # panorama = Stitcher()
    # panorama = Panaroma()
    receiver = ImageReceiver()
    processing_thread = ImageProcessingThread(receiver)
    processing_thread.start()
    rospy.spin()
    # while not rospy.is_shutdown():
        
    #     if receiver.left_image is not None and receiver.right_image is not None:
        
    #         panorama = VideoStitcher(left_video_in_path=receiver.left_image, right_video_in_path=receiver.right_image)
    #         # result = panorama.stitch([receiver.left_image, receiver.right_image])
    #         result = panorama.run()
    #         # (result, _) = panorama.image_stitch([receiver.left_image, receiver.right_image], match_status=True)
            

    #         # cv2.imshow("panorama", result)
    #         # cv2.waitKey(1) 
    #         result_msg = receiver.bridge.cv2_to_imgmsg(result, encoding="passthrough")
    #         result_msg.header.stamp = receiver.stamp
    #         receiver.pub.publish(result_msg)
 
            # rospy.sleep(3)
            # if cv2.waitKey(1) & 0xFF == ord("q"):

            #     break
            # receiver.left_image_ready = False
            # receiver.right_image_ready = False


    # cv2.destroyAllWindows()