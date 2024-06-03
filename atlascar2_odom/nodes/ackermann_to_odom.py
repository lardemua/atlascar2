#!/usr/bin/env python3

from functools import partial
from math import sin, cos


import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped


def odomCallback(msg, odom_velocity):
    """Subscribe to the ackermann drive message and retrieve its velocity value#s

    Args:
        msg (AckermannDriveStamped): Retrieved from /ackermann_steering_controller/ackermann_drive
        odom_velocity (dictionary): iterable variable to store the values
    """

    odom_velocity['vx'] = msg.drive.speed
    odom_velocity['vth'] = msg.drive.steering_angle_velocity


def main():
    rospy.init_node('odometry_publisher')

    # Defining initial variables
    odom_velocity = {'vx': 0, 'vy': 0, 'vth': 0}
    odomMsg = Odometry()
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    covariance_twist = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1000.0]
    covariance_pose = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       1000.0]
    
    x = 0.0
    y = 0.0
    th = 0.0

    # Defining callback partial
    odomCallback_partial = partial(odomCallback, odom_velocity=odom_velocity)

    # subscribe to the ackermann messages
    rospy.Subscriber('ackermann_steering_controller/ackermann_drive', AckermannDriveStamped, odomCallback_partial, queue_size=10)

    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = odom_velocity['vx'] * cos(th) * dt
        delta_y = odom_velocity['vx'] * sin(th) * dt
        delta_th = odom_velocity['vth'] * dt
        x += delta_x
        y += delta_y
        th += delta_th

        odomMsg.header.stamp = rospy.Time.now()
        odomMsg.twist.twist.linear.x = odom_velocity['vx']
        odomMsg.twist.twist.linear.y = odom_velocity['vy']
        odomMsg.twist.twist.angular.z = odom_velocity['vth']
        odomMsg.twist.covariance = covariance_twist

        odomMsg.header.frame_id = 'odom'
        odomMsg.child_frame_id = 'base_footprint'

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # set the position
        odomMsg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odomMsg.pose.covariance = covariance_pose

        # publish the message
        odom_pub.publish(odomMsg)
        last_time = current_time
        current_time = rospy.Time.now()
        
        rate.sleep()


if __name__ == '__main__':
    main()
