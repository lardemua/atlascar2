#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist, Quaternion
import time
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler


def spawn_model(name, x, y, path):
    rospy.init_node('spawn_model')

    # Wait for the spawn_model service to become available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Define the pose at which to spawn the model
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0  

    # Load the SDF file of your object
    with open(path, 'r') as f:
        model_xml = f.read()

    # Name your model
    model_name = name

    # Spawn the model
    spawn_model(model_name, model_xml, '', pose, 'world')

    return model_name

def move_model(model_name):
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub_atlascar = rospy.Publisher('/ackermann_steering_controller/cmd_vel', Twist, queue_size=1)


    

    
    # Set the initial pose
    pose = Pose()
    pose.position.x = -45
    pose.position.y = -60
    pose.position.z = 0  # Adjust the height as needed

    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    # Create a Quaternion message for orientation
    orientation = Quaternion()
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]

    # Assign orientation to the pose
    pose.orientation = orientation

    model_state_msg = ModelState()
    model_state_msg.model_name = model_name
    model_state_msg.pose = pose

    atlascar_msg = Twist()
    atlascar_msg.linear.x = 2
    # Set the movement parameters
    speed = -0.1  # Adjust as needed
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pose.position.y += speed
        model_state_msg.pose = pose
        pub.publish(model_state_msg)
        # pub_atlascar.publish(atlascar_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        model_name = spawn_model('car_beetle', -45, -60, '/home/rafael/catkin_ws/src/gazebo_cars/models/car_beetle/model.sdf')
        # person = spawn_model('person_standing', -55, -73, '/home/rafael/catkin_ws/src/models/person_standing/model.sdf')
        move_model(model_name)
    except rospy.ROSInterruptException:
        pass
