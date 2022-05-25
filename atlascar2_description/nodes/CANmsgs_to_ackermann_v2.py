#!/usr/bin/env python

"""
Shows how the receive messages via polling.
"""

import rospy
import can
from can.bus import BusState
from ackermann_msgs.msg import AckermannDriveStamped
import math

ack_pub = rospy.Publisher("atlascar2/ackermann_msgs", AckermannDriveStamped, queue_size=10)


def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""
    global ack_pub
    steering_angle = 0
    speed = 0
    ackMsg = AckermannDriveStamped()
    rate = rospy.Rate(100)
    newposition = 0
    oldposition = 0
    maxPPR = 1000
    newtime = rospy.Time.now()
    oldtime = rospy.Time.now()

    with can.interface.Bus(bustype="socketcan", channel="can0", bitrate=500000) as bus:
        bus.set_filters([{"can_id": 0x500, "can_mask": 0x530}, {"can_id": 0x236, "can_mask": 0x237}])
        # bus.set_filters([{"can_id": 0x236, "can_mask": 0x237}])
        # right now I need the B1 with the ID 0x412 and B0 with the ID 0x236
        while not rospy.is_shutdown():
            msg = bus.recv(1)
            # print(msg)
            if msg is not None:
                # if msg.arbitration_id == 0x412:
                #     # speed is in km/h
                #     speed_km = msg.data[1]
                #     speed_ms = (speed_km * 1000)/3600
                #     # print(speed_ms)

                # if msg.arbitration_id == 0x298:
                #     motor_rpm = (msg.data[6]*256 + msg.data[7]) - 10000
                #     # 7.065 -> final drive ratio from the vehicle
                #     # 0.285 -> radius of the wheel
                #     speed = (motor_rpm*math.pi*0.285)/(30*7.065)
                #     # print(speed)
                #     ackMsg.header.stamp = rospy.Time.now()
                #     ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                #     ackMsg.drive.speed = speed
                #     # steering needs to be in radians
                #     ackMsg.drive.steering_angle = (math.pi * steering_angle) / 180
                #     # print(speed, steering_angle)
                #     ack_pub.publish(ackMsg)
                #     rate.sleep()
                if msg.arbitration_id == 0x500:
                    # speed is in km/h
                    newposition = int.from_bytes(msg.data, "big", signed=True)
                    newtime = rospy.Time.now()
                    if newtime.to_sec() - oldtime.to_sec() < 1:
                        continue
                    frequency = (newposition - oldposition) / (newtime.to_sec() - oldtime.to_sec())
                    rps = frequency / maxPPR
                    speed = (rps * math.pi * 0.285 * 2)
                    ackMsg.header.stamp = rospy.Time.now()
                    ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                    ackMsg.drive.speed = speed
                    # steering needs to be in radians
                    ackMsg.drive.steering_angle = (math.pi * steering_angle) / 180
                    # print(f"newposition: {newposition} , oldposition: {oldposition}")
                    print(speed,steering_angle)
                    oldposition = newposition
                    oldtime = newtime
                    # print(frequency)
                    ack_pub.publish(ackMsg)
                    # rate.sleep()

                if msg.arbitration_id == 0x236:
                    # to get the steering angle its the following formula:
                    # ((B0*256 + B1) -4096)/2
                    # sendo B1 o byte 1 do identificador 0x412 e B0 0 byte 0 do identificador 0x236
                    # angulo do volante e não o das rodas. steering ratio : 16.06
                    steering_angle = ((msg.data[0] * 256 + msg.data[1]) - 4096) / (2 * 16.06)
                    steering_angle = (math.pi * steering_angle) / 180
                    ackMsg.header.stamp = rospy.Time.now()
                    ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                    ackMsg.drive.speed = speed
                    # steering needs to be in radians
                    ackMsg.drive.steering_angle = steering_angle
                    print(speed, steering_angle)
                    ack_pub.publish(ackMsg)
            # rate.sleep()


def main():
    rospy.init_node('ackermann_publisher')
    receive_all()
    rospy.spin()


if __name__ == '__main__':
    main()
