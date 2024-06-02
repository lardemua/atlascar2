#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

time_A = time.time()
frame_rate = 10

class ImageRepublisher:

    def __init__(self):
        rospy.init_node('image_republisher', anonymous=True)
        
        self.image_sub = rospy.Subscriber("/panorama_img", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/panorama_img1", Image, queue_size=1)

        self.bridge = CvBridge()



    def image_callback(self, data):
            
        self.image_pub.publish(data)
 



if __name__ == '__main__':

    image_republisher = ImageRepublisher()

    rospy.spin()