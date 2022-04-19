#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped


def odom_callback(data):
    global last_time, current_time, x, y, th, vx, vy, vth, wheelbase
    global odom_pub, odom_broadcaster
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    x += delta_x
    y += delta_y
    th += delta_th

    # method 3 from  https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
    offset = 0.015
    vx = data.drive.speed
    vy = 0.0
    vth = (data.drive.speed*math.tan(data.drive.steering_angle))/(wheelbase)
    odomMsg = Odometry()
    odomMsg.twist.twist.linear.x = vx
    odomMsg.twist.twist.linear.y = vy
    odomMsg.twist.twist.angular.z = vth

    odomMsg.header.frame_id = 'atlascar2/odom'
    odomMsg.child_frame_id = 'atlascar2/base_footprint'

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "atlascar2/base_footprint",
        "atlascar2/odom"
    )

    # set the position
    odomMsg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # publish the message
    odom_pub.publish(odomMsg)
    last_time = current_time


def main():
    global last_time, current_time, x, y, th, vx, vy, vth, wheelbase
    global odom_pub, odom_broadcaster

    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("atlascar2/odom", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    wheelbase = rospy.get_param('~wheelbase', 2.55)
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    rospy.Subscriber('atlascar2/ackermann_msgs', AckermannDriveStamped, odom_callback,
                     queue_size=10)
    # rospy.Subscriber('atlascar2/ackermann_steering_controller/ackermann_drive', AckermannDriveStamped, odom_callback, queue_size=10)
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0

    rospy.spin()


if __name__ == '__main__':
    main()