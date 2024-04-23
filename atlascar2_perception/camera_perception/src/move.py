#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Twist, Quaternion
import time
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler


def spawn_model(name, x, y, path):
    

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

def move_model(speed_x, ang_z):
    # pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub_atlascar = rospy.Publisher('/ackermann_steering_controller/cmd_vel', Twist, queue_size=1)   

    
    # Set the initial pose
    # pose = Pose()
    # pose.position.x = -40
    # pose.position.y = -80
    # pose.position.z = 0  # Adjust the height as needed

    # roll = 0.0
    # pitch = 0.0
    # yaw = 3
    # quaternion = quaternion_from_euler(roll, pitch, yaw)

    # # Create a Quaternion message for orientation
    # orientation = Quaternion()
    # orientation.x = quaternion[0]
    # orientation.y = quaternion[1]
    # orientation.z = quaternion[2]
    # orientation.w = quaternion[3]

    # # Assign orientation to the pose
    # pose.orientation = orientation

    # model_state_msg = ModelState()
    # model_state_msg.model_name = model_name
    # model_state_msg.pose = pose
    # model_state_msg.reference_frame = 'base_footprint'

    atlascar_msg = Twist()
    atlascar_msg.linear.x = speed_x
    atlascar_msg.angular.z = ang_z
    # speed = 0.06  # Adjust as needed
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # pose.position.y += speed
        # print(pose.position.y)
        # # pose.position.x += 0.1
        # model_state_msg.pose = pose
        # pub.publish(model_state_msg)
        pub_atlascar.publish(atlascar_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('spawn_model')
    ang_z = rospy.get_param('~ang_z', 0)
    speed_x = rospy.get_param('~speed_x', 0)
    try:
        # beetle = spawn_model('car_beetle', -45, -60, '/home/rafael/catkin_ws/src/gazebo_cars/models/car_beetle/model.sdf')
        # person = spawn_model('human_female_1', -40, -80, '/home/rafael/.gazebo/models/human_female_1/model.sdf')
        # suv = spawn_model('suv', -45, -60, '/home/rafael/catkin_ws/src/models/suv/model.sdf')
        move_model(speed_x, ang_z)
    except rospy.ROSInterruptException:
        pass