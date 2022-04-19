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
    B0_value = None
    B1_value = None
    global ack_pub
    ackMsg = AckermannDriveStamped()

    with can.interface.Bus(bustype="socketcan", channel="can0", bitrate=500000) as bus:

        # right now I need the B1 with the ID 0x412 and B0 with the ID 0x236
        while not rospy.is_shutdown():
            msg = bus.recv(1)
            if msg is not None:
                if msg.arbitration_id == 0x412:
                    # hex_B1_value = str(msg.data.hex())
                    # B1_value = int(hex_B1_value[12:14], base=16)
                    B1_value = msg.data[1]
                    # print(B1_value)

                if msg.arbitration_id == 0x236:
                    # to get the steering angle its the following formula:
                    # ((B0*256 + B1) -4096)/2
                    # sendo B1 o byte 1 do identificador 0x412 e B0 0 byte 0 do identificador 0x236
                    # hex_B0_value = str(msg.data.hex())
                    # B0_value = int(hex_B0_value[14:16], base=16)
                    B0_value = msg.data[0]
                    # print(B0_value)

                if (B0_value is not None) & (B1_value is not None):
                    steering_angle = ((B0_value*256 + B1_value) - 4096)/2
                    speed = B1_value
                    ackMsg.drive.speed = speed
                    ackMsg.drive.steering_angle = steering_angle
                    ackMsg.header.stamp = rospy.Time.now()
                    ackMsg.header.frame_id = "atlascar2/ackermann_msgs"
                    ack_pub.publish(ackMsg)


def main():

    rospy.init_node('ackermann_publisher')
    receive_all()
    rospy.spin()


if __name__ == '__main__':
    main()
