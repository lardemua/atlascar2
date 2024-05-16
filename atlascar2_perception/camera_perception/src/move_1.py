#!/usr/bin/env python3

import rospy
import argparse
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler

def move_model(model_name, x, y, roll, pitch, yaw, path):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)       
    
    with open(path, 'r') as f:
        model_xml = f.read()

    quaternion = quaternion_from_euler(roll, pitch, yaw)
    # Set the initial pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    spawn_model(model_name, model_xml, '', pose, 'world')

    model_state_msg = ModelState()
    model_state_msg.model_name = model_name
    model_state_msg.pose = pose

    return model_state_msg

def main():
   

    rospy.init_node('spawn_model')
    model_name = rospy.get_param('~model_name', 'car_beetle')
    model_path = rospy.get_param('~model_path', '/home/rafael/catkin_ws/src/gazebo_cars/models/car_beetle/model.sdf')
    x = rospy.get_param('~x', -45)
    y = rospy.get_param('~y', -60)
    roll = rospy.get_param('~roll', 0)
    pitch = rospy.get_param('~pitch', 0)
    yaw = rospy.get_param('~yaw', 0)
    speed_y = rospy.get_param('~speed_y', -0.2)
    speed_x = rospy.get_param('~speed_x', 0)
    try:
        model = move_model(model_name, x, y, roll, pitch, yaw, model_path)

        pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        rate = rospy.Rate(20)  # 10 Hz
        while not rospy.is_shutdown():
            # Update the position of the model
            model.pose.position.y += speed_y  # Adjust as needed
            model.pose.position.x += speed_x
            # Publish the updated position of the model
            pub.publish(model)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
