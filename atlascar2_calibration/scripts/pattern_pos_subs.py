#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
import sys
import tty
import termios
import os
import signal

def pose_callback(msg):
    global poses_msg
    poses_msg = msg

def keyboard_listener():
    # Set terminal settings to capture keyboard input
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            # Wait for a keypress
            key = sys.stdin.read(1)
            if key == '\x03':  # Ctrl+C to exit
                break
            elif key == 'p':  # Press 'p' to append current pose
                append_pose_once()
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def signal_handler(sig, frame):
    print("Final Poses List:", poses)
    save_poses_to_file('/home/rafael/catkin_ws/src/atlascar2/atlascar2_calibration/scripts', 'poses')
    sys.exit(0)

def append_pose_once():
    if poses_msg is not None and len(poses_msg.pose) > 0:
        pose = get_current_pose(poses_msg)
        poses.append(pose)
        print("Pose appended:", pose)
    else:
        print("No pose message received. Make sure the model_states topic is publishing.")

def get_current_pose(msg):
    index = msg.name.index("charuco_800x600_5x5_100")
    pose = msg.pose[index]
    position = pose.position
    orientation = pose.orientation
    return [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]

def save_poses_to_file(directory, base_filename):
    counter = 1
    while True:
        filename = f"{base_filename}_{counter}.txt"
        filepath = os.path.join(directory, filename)
        if not os.path.exists(filepath):
            break
        counter += 1
    
    with open(filepath, 'w') as file:
        for pose in poses:
            file.write(','.join(str(coord) for coord in pose) + '\n')
    print("Poses saved to", filepath)

if __name__ == '__main__':
    rospy.init_node("pose_subscriber")
    poses = []
    poses_msg = None  
    rospy.Subscriber("/gazebo/model_states", ModelStates, pose_callback)
    
    signal.signal(signal.SIGINT, signal_handler)
    keyboard_listener()

    