#!/usr/bin/python3

import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
# Global variable to store the ego vehicle's speed
vx = 0.0
translation_offset = 0

def vel_sub(vel):
	global vx
	vx, vy = vel.twist.twist.linear.x, vel.twist.twist.linear.y

def model_states_callback(msg):
    # Find the index of your model in the ModelStates message
    global model_position

    idx = msg.name.index("atlascar2")
    # Extract the pose information
    model_pose = msg.pose[idx]
    model_position = model_pose.position
    model_orientation = model_pose.orientation


def broadcast_tf():
    global vx, translation_offset, model_position

    # Initialize ROS node
    rospy.init_node('tf_broadcaster')

    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create a TransformStamped message
    transform_stamped = geometry_msgs.msg.TransformStamped()

    # Populate the transform message
    transform_stamped.header.frame_id = 'world'  # Assuming the ego vehicle's frame is 'map' (replace with actual frame)
    transform_stamped.child_frame_id = 'base_footprint'  # Specify the name of the fixed frame

    # Adjust the translation based on the ego vehicle's speed
    translation_offset += vx * 0.01  # Example scaling factor to convert speed to translation (adjust as needed)
    transform_stamped.transform.translation.x = model_position.x
    transform_stamped.transform.translation.y = 0.0
    transform_stamped.transform.translation.z = 0.0

    # Example constant orientation (no rotation)
    transform_stamped.transform.rotation.x = 0.0
    transform_stamped.transform.rotation.y = 0.0
    transform_stamped.transform.rotation.z = 0.0
    transform_stamped.transform.rotation.w = 1.0

    # Set the timestamp
    transform_stamped.header.stamp = rospy.Time.now()

    # Publish the transform
    tf_broadcaster.sendTransform(transform_stamped)

    rospy.loginfo("Published transform from map to fixed_frame")

if __name__ == '__main__':
    try:
        # Initialize subscriber to ego vehicle speed
        rospy.Subscriber("/ackermann_steering_controller/odom", Odometry, vel_sub)
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

        # Publish the transform repeatedly
        while not rospy.is_shutdown():
            broadcast_tf()
            rospy.sleep(0.01)  # Publish the transform every 1 second
    except rospy.ROSInterruptException:
        pass
