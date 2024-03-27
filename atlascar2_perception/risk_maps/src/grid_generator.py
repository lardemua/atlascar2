#!/usr/bin/env python3
'''
@author: Unmesh Patil
@project: Collision risk estimation at CHROMA, INRIA, France (April, 2021)
@description: check out my paper: https://hal.inria.fr/hal-03416222 
About code: Shows predicted future occupancy for a scenario in carla simulator.
First play the bagfile, then run the carla_objects code and then orsp_carla.py. Open rviz config to visualize.
'''
################################################################################### section 1 imports
import rospy, sys, os
import numpy as np
from visualization_msgs.msg import MarkerArray
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from nav_msgs.msg import Odometry
from probability_machine import prob_machine_gridgen

########################################################################### section 2 defaults and global vars
marker_array_ = MarkerArray()

selfid = 1
Sequence = True;
objectList =  np.zeros((12,12), dtype=np.float64)
marker_list = np.zeros((3, 5), dtype=np.float64)
t = 2  #time horizon
resolution = 0.1
n = int(1/resolution)
w = 80*n + 1
h = 80*n + 1
originX = 0
originY = 0
vxmy, vymy, wmy = 1.0 , 1.0, 0.0

#################################################################################### section 4 helper functions
def gridMap_generator(rspaceData):
	global resolution, n, w, h, originX, originY
	rspaceData = np.asarray(rspaceData, dtype=np.float64).reshape(h,w)
	rspaceData = np.rot90(rspaceData, 2).flatten().tolist()
	gridmap = GridMap()
	multi_array = Float32MultiArray()
	multi_array.layout.dim.append(MultiArrayDimension())
	multi_array.layout.dim.append(MultiArrayDimension())
	multi_array.layout.dim[0].label = "column_index"
	multi_array.layout.dim[0].size = w
	multi_array.layout.dim[0].stride = h*w
	multi_array.layout.dim[1].label = "row_index"
	multi_array.layout.dim[1].size = h
	multi_array.layout.dim[1].stride = w
	multi_array.data = rspaceData
	gridmap.layers.append("elevation")
	gridmap.data.append(multi_array)
	gridmap.info.length_x = 80
	gridmap.info.length_y = 80
	gridmap.info.pose.position.x = originY + 40
	gridmap.info.pose.position.y = originX 
	gridmap.info.header.frame_id = "base_footprint"
	gridmap.info.resolution = 0.1
	grid_pub.publish(gridmap)


############################################################################################# section 5 callbacks	

def callback_sub(marker_data):
	global Sequence, marker_list, objectList
	global vxmy, vymy, wmy, selfid
	count = len(marker_data.markers)
	car_count = 0.0

	for i in range(count):
		if count != 0:
			car_count += 1
			ttype = 4
			print(len(marker_data.markers))
			if marker_data.markers[i].text == "car": ttype = 1.0
			if marker_data.markers[i].text == "rider": ttype = 2.0
			if marker_data.markers[i].text == "pedestrian": ttype = 4.0
				
		
			# DelT = marker_data.markers[i].header.stamp.to_sec() - marker_list[i][0]
			DelT = 1/2.5
			x_ = marker_data.markers[i].pose.position.x
			y_ = marker_data.markers[i].pose.position.y
			# print(DelT)
			# print(marker_data.markers[i].header.stamp.to_sec())
			# vx = ( x_ - objectList[i][3])/ (DelT)
			# vy = ( y_ - objectList[i][4])/ (DelT)
			vx = x_ / DelT
			vy = y_ / DelT
			#relative velocity
			vx += vxmy
			vy += vymy
			# rot = []
			# rot = [0, 0, marker_data.markers[i].pose.orientation.z, marker_data.markers[i].pose.orientation.w]
			# (roll, pitch, yaw) = euler_from_quaternion(rot)
			# direction = np.array([pose_odom.point.x, pose_odom.point.y]) - np.array([pose_odom_prev_x, pose_odom_prev_y])
			# yaw = atan2(direction[1], direction[0])
			yaw =  marker_data.markers[i].pose.orientation.z

			print('yaw:', yaw)
			omega = (yaw - objectList[i][5])/DelT
			print('omega:', omega)
			#relative omega
			omega = (wmy + omega)
			objectList[i][0] = marker_data.markers[i].id
			objectList[i][1] = (vx - objectList[i][8])/ DelT #ax
			objectList[i][2] = (vy - objectList[i][9])/ DelT #ay
			objectList[i][3] = x_
			objectList[i][4] = y_
			objectList[i][5] = yaw
			objectList[i][6] = ttype
			objectList[i][7] = omega
			objectList[i][8] = vx 
			objectList[i][9] = vy
			objectList[i][10] = marker_data.markers[i].scale.y #w
			objectList[i][11] = marker_data.markers[i].scale.x #h
			# pose_odom_prev_x = pose_odom.point.x
			# pose_odom_prev_y = pose_odom.point.y 
			print('x:', x_)
			print('y:', y_)
		#print(vxmy, vymy, wmy, vx, vy, omega, marker_data.markers[i].id)
	rsp = prob_machine_gridgen(car_count,originX, originY -40, objectList, vxmy, vymy, t)
	print(rsp)
	gridMap_generator(rsp)

def vel_sub(vel):
	global vxmy, vymy, wmy
	vxmy, vymy = vel.twist.twist.linear.x, vel.twist.twist.linear.y
	wmy = vel.twist.twist.angular.z
	

#################################################################################################### section 6 main body

if __name__ == '__main__':
	try:
		rospy.init_node('motion_models', anonymous=True)
		# rospy.Subscriber("/filter_objects_data", MarkerArray, callback_sub)
		# rospy.Subscriber("/self_vel", TwistStamped, vel_sub)
		rospy.Subscriber("/fused_detection", MarkerArray, callback_sub)
		rospy.Subscriber("/ackermann_steering_controller/odom", Odometry, vel_sub)
		grid_pub = rospy.Publisher("/rspaceGrid2", GridMap, queue_size=1 )
		rospy.spin()
	except rospy.ROSInterruptException:
		pass