#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage

def usb_camera_publisher():
    # Initialize ROS node
    rospy.init_node('usb_camera_publisher', anonymous=True)
    
    with open('/home/rafael/Desktop/USB/ost.yaml', 'r') as file:
        camera_params = yaml.safe_load(file)
    # Create publisher for publishing images
    image_pub = rospy.Publisher('/top_right_camera/image_raw', Image, queue_size=1)
    image_compressed_pub = rospy.Publisher('/top_right_camera/image_raw/compressed', CompressedImage, queue_size=1)
    camera_info_pub = rospy.Publisher('/top_right_camera/camera_info', CameraInfo, queue_size=1)
    # Create CvBridge object
    bridge = CvBridge()
    camera_matrix_data = camera_params['camera_matrix']['data']
    dist_coeffs_data = camera_params['distortion_coefficients']['data']
    rectification_matrix_data = camera_params['rectification_matrix']['data']
    projection_matrix_data = camera_params['projection_matrix']['data']

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "top_right_camera_optical" 
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.width = camera_params['image_width']
    camera_info_msg.height = camera_params['image_height']
    camera_info_msg.K = camera_matrix_data
    camera_info_msg.D = dist_coeffs_data
    camera_info_msg.R = rectification_matrix_data
    camera_info_msg.P = projection_matrix_data

    # Initialize OpenCV video capture object
    cap = cv2.VideoCapture(0)  


    # Set video capture properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)  # Adjust width as needed
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Adjust height as needed
    frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print("Frame width:", frame_width)
    print("Frame height:", frame_height)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("Frame rate:", fps)
    # rate = rospy.Rate(30)  # Define publishing rate (30Hz in this case)

    while not rospy.is_shutdown():
        ret, frame = cap.read()  # Read frame from camera

        if ret:
            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)
            # Convert OpenCV image to ROS message

            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.frame_id = "top_right_camera_optical"
            time = rospy.Time.now()
            ros_image.header.stamp = time
            ros_image_compressed = bridge.cv2_to_compressed_imgmsg(frame)

            # Publish ROS message
            image_pub.publish(ros_image)
            image_compressed_pub.publish(ros_image_compressed)
            camera_info_pub.publish(camera_info_msg)
     

        # rate.sleep()

    # Release video capture object
    cap.release()

if __name__ == '__main__':
    try:
        usb_camera_publisher()
    except rospy.ROSInterruptException:
        pass
