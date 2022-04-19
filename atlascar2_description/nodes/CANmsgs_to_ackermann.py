#!/usr/bin/env python

"""
Shows how the receive messages via polling.
"""

import rospy
import can
from can.bus import BusState
from ackermann_msgs.msg import AckermannDriveStamped

ack_pub = rospy.Publisher("atlascar2/ackermann_msgs", AckermannDriveStamped, queue_size=10)

def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""
    steering_angle = None
    speed = None
    global ack_pub
    ackMsg = AckermannDriveStamped()
    # rate = rospy.Rate(100)

    with can.interface.Bus(bustype="socketcan", channel="can0", bitrate=500000) as bus:

        # right now I need the B1 with the ID 0x412 and B0 with the ID 0x236
        while not rospy.is_shutdown():
            msg = bus.recv(1)
            if msg is not None:
                if msg.arbitration_id == 0x412:
                    B1_value = msg.data[1]
                    speed = B1_value

                if msg.arbitration_id == 0x236:
                    # to get the steering angle its the following formula:
                    # ((B0*256 + B1) -4096)/2
                    # sendo B1 o byte 1 do identificador 0x412 e B0 0 byte 0 do identificador 0x236
                    steering_angle = ((msg.data[0] * 256 + msg.data[1]) - 4096) / 2

                if (steering_angle is not None) & (speed is not None):
                    ackMsg.drive.speed = speed
                    ackMsg.drive.steering_angle = steering_angle
                    ackMsg.header.stamp = rospy.Time.now()
                    ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                    ack_pub.publish(ackMsg)
                    print(steering_angle)
                    # rate.sleep()


def main():

    rospy.init_node('ackermann_publisher')
    receive_all()
    rospy.spin()


if __name__ == '__main__':
    main()
