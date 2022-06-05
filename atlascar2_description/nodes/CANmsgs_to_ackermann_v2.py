#!/usr/bin/env python

import rospy
import can
from can.bus import BusState
from ackermann_msgs.msg import AckermannDriveStamped
import math

ack_pub = rospy.Publisher('ackermann_steering_controller/ackermann_drive', AckermannDriveStamped, queue_size=10)


def receive_all():
    """Receives the steering angle and encoder tick messages"""
    global ack_pub
    steering_angle = 0
    steer_velocity = 0
    speed = 0
    ackMsg = AckermannDriveStamped()
    oldposition = 0
    maxPPR = 1000
    wheelbase = rospy.get_param('~wheelbase', 2.55)
    wheel_radius = 0.285
    oldtime_pos = rospy.Time.now()

    with can.interface.Bus(bustype="socketcan", channel="can0", bitrate=500000) as bus:
        # filter all the messages and only let's the 0x500 and 0x236 message ID
        bus.set_filters([{"can_id": 0x500, "can_mask": 0x530}, {"can_id": 0x236, "can_mask": 0x237}])
        while not rospy.is_shutdown():
            # receives the message
            msg = bus.recv(1)
            # print(msg)
            if msg is not None:
                if msg.arbitration_id == 0x500:
                    # gets the encoder ticks
                    newposition = int.from_bytes(msg.data, "big", signed=True)
                    newtime_pos = rospy.Time.now()
                    if newtime_pos.to_sec() - oldtime_pos.to_sec() < 0.1:
                        continue
                    # calculates the speed in m/s
                    frequency = (newposition - oldposition) / (newtime_pos.to_sec() - oldtime_pos.to_sec())
                    rps = frequency / maxPPR
                    speed = (rps * math.pi * wheel_radius * 2)
                    ackMsg.header.stamp = rospy.Time.now()
                    ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                    ackMsg.drive.speed = speed
                    ackMsg.drive.steering_angle_velocity = steer_velocity
                    ackMsg.drive.steering_angle = steering_angle
                    # print(f"newposition: {newposition} , oldposition: {oldposition}")
                    print(speed, steer_velocity, steering_angle)
                    oldposition = newposition
                    oldtime_pos = newtime_pos
                    ack_pub.publish(ackMsg)

                if msg.arbitration_id == 0x236:
                    # to get the steering angle its the following formula:
                    # ((B0*256 + B1) -4096)/2
                    # to get the wheel angle : divide the formula by 16.06
                    steering_angle = ((msg.data[0] * 256 + msg.data[1]) - 4096) / (2 * 16.06)
                    steering_angle = (math.pi * steering_angle) / 180
                    # calculate the angular velocity
                    steer_velocity = math.tan(steering_angle)*(speed/wheelbase)
                    ackMsg.header.stamp = rospy.Time.now()
                    ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                    ackMsg.drive.speed = speed
                    ackMsg.drive.steering_angle_velocity = steer_velocity
                    ackMsg.drive.steering_angle = steering_angle
                    print(speed, steer_velocity, steering_angle)
                    ack_pub.publish(ackMsg)


def main():
    rospy.init_node('ackermann_publisher')
    receive_all()
    rospy.spin()


if __name__ == '__main__':
    main()
