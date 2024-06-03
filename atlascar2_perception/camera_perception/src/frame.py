#!/usr/bin/python3





import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageRepublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_republisher', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the input image topic
        self.image_sub = rospy.Subscriber('/panorama_img', Image, self.image_callback)

        # Create a publisher for the output image topic
        self.image_pub = rospy.Publisher('/panorama_img1', Image, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Perform any processing on the image here (if needed)
            # For this example, we'll just change the encoding to 'mono8' (grayscale)
            # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert the processed OpenCV image back to a ROS Image message
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Publish the output image
            self.image_pub.publish(output_msg)

        except CvBridgeError as e:
            rospy.logerr('CvBridge Error: {}'.format(e))

if __name__ == '__main__':
    try:
        # Create an instance of the ImageRepublisher class
        republisher = ImageRepublisher()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
