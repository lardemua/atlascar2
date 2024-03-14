#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64
from time import time

def load_poses_from_file(filepath):
    poses = []
    with open(filepath, 'r') as file:
        for line in file:
            pose_data = line.strip().split(',')
            pose = [float(coord) for coord in pose_data]
            poses.append(pose)
    return poses

def move_model():
    global loaded_poses, model_state_msg
    rate = rospy.Rate(0.5)  # Adjust the rate as needed
    pose_index = 0
    while not rospy.is_shutdown() and pose_index < len(loaded_poses):
        current_time = rospy.Time.now()
        model_state_msg.model_name = "charuco_800x600_5x5_100"
        model_state_msg.pose.position.x = loaded_poses[pose_index][0]
        model_state_msg.pose.position.y = loaded_poses[pose_index][1]
        model_state_msg.pose.position.z = loaded_poses[pose_index][2]
        model_state_msg.pose.orientation.x = loaded_poses[pose_index][3]
        model_state_msg.pose.orientation.y = loaded_poses[pose_index][4]
        model_state_msg.pose.orientation.z = loaded_poses[pose_index][5]
        model_state_msg.pose.orientation.w = loaded_poses[pose_index][6]
        
        try:
            set_state(model_state_msg)
            print('Published:', loaded_poses[pose_index])
            pose_index += 1
        except rospy.ServiceException as e:
            print("Service call failed:", e)
        
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("pattern_pos_pub", anonymous=True)

    poses_filepath = '/home/rafael/catkin_ws/src/atlascar2/atlascar2_calibration/scripts/poses_1.txt'
    loaded_poses = load_poses_from_file(poses_filepath)

    model_state_msg = ModelState()

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    try:
        move_model()
    except rospy.ServiceException:
        pass



        
        
       