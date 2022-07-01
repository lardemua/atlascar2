#!/usr/bin/env python3

from functools import partial
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

def modelStatesCallback(message, states):

    # If the model is atlascar2, get its model state and send the tf
    for idx, model in enumerate(message.name):
        if model == "atlascar2":
            states['pose'] =  message.pose[idx]
            states['start'] = True


def main():

    rospy.init_node('model_states_to_tf')

    # t is the transform to send and br is the broadcaster to send it    
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'odom'
    t.child_frame_id = 'base_footprint'

    states = {'pose': [], 'transform': t, 'broadcaster': br, 'start': False}

    modelStatesPartialCallback = partial(modelStatesCallback, states=states)
    
    rospy.Subscriber('/gazebo/model_states', ModelStates, modelStatesPartialCallback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if states['start']:
            states['transform'].header.stamp = rospy.Time.now()
            states['transform'].transform.translation = states['pose'].position
            states['transform'].transform.rotation = states['pose'].orientation
            states['broadcaster'].sendTransform(states['transform'])
        rate.sleep()


if __name__ == '__main__':
    main()