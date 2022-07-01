#!/usr/bin/env python3

from functools import partial
import rospy
from can_msgs.msg import Frame
from ackermann_msgs.msg import AckermannDriveStamped
import math


def can_msgs_callback(msg, can_msgs):
    if msg.id == 566:
        can_msgs['previous_angle'] = can_msgs['angle']
        can_msgs['angle'] = msg.data
        can_msgs['new_angle'] = True
    elif msg.id == 1280:
        can_msgs['pulse'] = int.from_bytes(msg.data, "big", signed=True)
        can_msgs['pulse_time'] = rospy.Time.now()
        can_msgs['new_pulse'] = True

def receive_all(ack_pub):
    """Receives the steering angle and encoder tick messages"""
    steering_angle = 0
    steer_velocity = 0
    speed = 0
    ackMsg = AckermannDriveStamped()
    maxPPR = 1000
    wheelbase = rospy.get_param('~wheelbase', 2.55)
    wheel_radius = 0.285
    can_msgs = {'previous_angle': None, 'previous_angle_time': rospy.Time.now(), 'angle': None, 'angle_time': rospy.Time.now(), 'new_angle': False, 
     'previous_pulse': None, 'previous_pulse_time': rospy.Time.now(), 'pulse': None, 'pulse_time': rospy.Time.now(), 'new_pulse': False}
    can_msgs_callback_partial = partial(can_msgs_callback, can_msgs=can_msgs)
    rospy.Subscriber('/received_messages', Frame, can_msgs_callback_partial)

    while not rospy.is_shutdown():
        # print(can_msgs)
        if can_msgs['new_pulse']:
            if can_msgs['pulse_time'].to_sec() - can_msgs['previous_pulse_time'].to_sec() < 1:
                continue
            # calculates the speed in m/s
            if can_msgs['previous_pulse'] != None:
                frequency = (can_msgs['pulse'] - can_msgs['previous_pulse']) / \
                (can_msgs['pulse_time'].to_sec() - can_msgs['previous_pulse_time'].to_sec())
                rps = frequency / (4 * maxPPR)
                speed = (rps * math.pi * wheel_radius * 2)
                ackMsg.header.stamp = rospy.Time.now()
                ackMsg.header.frame_id = "/ackermann_msgs"
                ackMsg.drive.speed = speed
                ackMsg.drive.steering_angle_velocity = steer_velocity
                ackMsg.drive.steering_angle = steering_angle
                ack_pub.publish(ackMsg)
                print(f'Ackermann message published.\nLinear velocity: {speed}; Angular velocity:{steer_velocity}; Steering angle: {steering_angle}')
            can_msgs['previous_pulse'] = can_msgs['pulse']
            can_msgs['previous_pulse_time'] = can_msgs['pulse_time']
        if can_msgs['new_angle']:
            if can_msgs['previous_angle'] != None:
                # to get the steering angle its the following formula:
                # ((B0*256 + B1) -4096)/2
                # to get the wheel angle : divide the formula by 16.06
                steering_angle = ((can_msgs['angle'][0] * 256 + can_msgs['angle'][1]) - 4096) / (2 * 16.06)
                steering_angle = (math.pi * steering_angle) / 180
                # calculate the angular velocity
                steer_velocity = math.tan(steering_angle)*(speed/wheelbase)
                ackMsg.header.stamp = rospy.Time.now()
                ackMsg.header.frame_id = "/ackermann_msgs"
                ackMsg.drive.speed = speed
                ackMsg.drive.steering_angle_velocity = steer_velocity
                ackMsg.drive.steering_angle = steering_angle
                print(f'Ackermann message published.\nLinear velocity: {speed}; Angular velocity:{steer_velocity}; Steering angle: {steering_angle}')
                ack_pub.publish(ackMsg)
            can_msgs['previous_angle'] = can_msgs['angle']
            can_msgs['previous_angle_time'] = can_msgs['angle_time']


def main():
    rospy.init_node('ackermann_publisher')
    ack_pub = rospy.Publisher('ackermann_steering_controller/ackermann_drive', AckermannDriveStamped, queue_size=10)
    receive_all(ack_pub)
    # rospy.spin()


if __name__ == '__main__':
    main()
