#!/usr/bin/env python

"""
Shows how the receive messages via polling.
"""

import rospy
import can
from can.bus import BusState
from ackermann_msgs.msg import AckermannDriveStamped


def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""

    with can.interface.Bus(
        bustype="socketcan", channel="can0", bitrate=500000) as bus:
        # bus = can.interface.Bus(bustype='ixxat', channel=0, bitrate=250000)
        # bus = can.interface.Bus(bustype='vector', app_name='CANalyzer', channel=0, bitrate=250000)

        # set to read-only, only supported on some interfaces
        # bus.state = BusState.PASSIVE

        # right now I need the B1 with the ID 0x412 and B0 with the ID 0x236
        try:
            while True:
                msg = bus.recv(1)
                if msg is not None:
                    # print(msg)
                    if msg.arbitration_id == 0x412:
                        print(msg)

        except KeyboardInterrupt:
            pass  # exit normally


if __name__ == "__main__":
    receive_all()