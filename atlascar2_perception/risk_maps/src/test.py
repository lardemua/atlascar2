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
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from nav_msgs.msg import Odometry
from probability_machine_1 import prob_machine_gridgen, prob_machine_riskgen, prob_machine_cplotter

########################################################################### section 2 defaults and global vars
marker_array_ = MarkerArray()

selfid = 1
Sequence = True;
objectList =  np.zeros((12,16), dtype=np.float64)
marker_list = np.zeros((3, 5), dtype=np.float64)
t = 3  #time horizon
resolution = 0.1
n = int(1/resolution)
w = 40*n + 1
h = 40*n + 1
originX = 0
originY = 0
vxmy, vymy, wmy = 1.0 , 1.0, 0.0
previous_time = None

#################################################################################### section 4 helper functions
class OrientationSmoothingFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.orientation_history = []

    def update(self, yaw):
        self.orientation_history.append(yaw)
        if len(self.orientation_history) > self.window_size:
            self.orientation_history = self.orientation_history[-self.window_size:]
        return sum(self.orientation_history) / len(self.orientation_history)
	

def gridMap_generator(rspaceData):
	global resolution, n, w, h, originX, originY
	
	rspaceData = np.asarray(rspaceData, dtype=np.float64).reshape(h,w)


	rspaceData = np.rot90(rspaceData, 2)



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
	multi_array.data = rspaceData.flatten().tolist()
	gridmap.layers.append("elevation")
	gridmap.data.append(multi_array)
	gridmap.info.length_x = 40
	gridmap.info.length_y = 40
	gridmap.info.pose.position.x = originY + 20
	gridmap.info.pose.position.y = originX 
	gridmap.info.header.frame_id = "base_footprint"
	gridmap.info.resolution = 0.1
	grid_pub.publish(gridmap)


############################################################################################# section 5 callbacks	

def callback_sub(marker_data):
	global Sequence, marker_list, objectList, previous_time
	global vxmy, vymy, wmy, selfid
	count = len(marker_data.markers)
	car_count = 0.0
	
	for i in range(count):
		if count != 0:
			car_count += 1
			ttype = 4
			
			if marker_data.markers[i].text == "car": ttype = 1.0
			if marker_data.markers[i].text == "rider": ttype = 2.0
			if marker_data.markers[i].text == "pedestrian": ttype = 4.0
			current_time = marker_data.markers[i].header.stamp.to_sec()
			
	
			if objectList[i][12] != 0:  
				elapsed_time = current_time - objectList[i][12]
				if (abs(marker_data.markers[i].pose.position.y - objectList[i][4])) > 0:
					if elapsed_time > 0:
						DelT = elapsed_time
						x_ = marker_data.markers[i].pose.position.x
						y_ = marker_data.markers[i].pose.position.y

						# print('time:', DelT)
						# print('y:', y_)
						vx = ( x_ - objectList[i][3])/ (DelT)
						vy = ( y_ - objectList[i][4])/ (DelT)
				
						# print('vx:', vx)
						omega = (marker_data.markers[i].pose.orientation.z - objectList[i][5])/DelT
						omega = (wmy + omega)
	
						vx += vxmy
						vy += vymy	
									
						# vy = -3
						# vx = 0
					
						# print('Vy:', vy)
						ax = (vx - objectList[i][8])/ DelT 
						ay = (vy - objectList[i][9])/ DelT
						# ay = -2
						# ax = 0
						objectList[i][12] = current_time
				else:
					vx = objectList[i][8]
					vy = objectList[i][9]
					ax = objectList[i][1]
					ay = objectList[i][2]
					x_ = objectList[i][3] 
					y_ = objectList[i][4]
					omega = objectList[i][7] 
				
				objectList[i][0] = marker_data.markers[i].id
				objectList[i][1] = ax
				objectList[i][2] = ay 
				objectList[i][3] = x_
				objectList[i][4] = y_
				objectList[i][5] = marker_data.markers[i].pose.orientation.z
				objectList[i][6] = ttype
				objectList[i][7] = omega			
				objectList[i][8] = vx
				objectList[i][9] = vy		
				objectList[i][10] = marker_data.markers[i].scale.y #w
				objectList[i][11] = marker_data.markers[i].scale.x #h
			
		
			else:
				objectList[i][12] = current_time
			
			# print('Vx:', vx)
			
		
			# objectList[i][12] = previous_time
			

	rsp_1 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, t)
	# rsp_2 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1.5)
	# rsp_3 = prob_machine_gridgen(car_count,originX, originY -40, objectList, vxmy, vymy, 1.75)
	# rsp_4 = prob_machine_gridgen(car_count,originX, originY -40, objectList, vxmy, vymy, 1.25)
	# rsp = rsp_1 * rsp_2 
	gridMap_generator(rsp_1)
	risk = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, t)
	print('risk:', risk)
	


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
		grid_ocupancy = rospy.Publisher("/ocupancy_grid", OccupancyGrid, queue_size=1 )

		rospy.spin()
	except rospy.ROSInterruptException:
		pass