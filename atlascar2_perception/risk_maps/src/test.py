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
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32
from nav_msgs.msg import Odometry
from probability_machine_2 import prob_machine_gridgen, prob_machine_riskgen, prob_machine_pedc
from math import atan2, pi



########################################################################### section 2 defaults and global vars
marker_array_ = MarkerArray()

selfid = 1
pedestrian = True
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
class MarkerProcessor:
	def __init__(self):
		self.smoother_list = []

	def callback_sub(self, marker_data):
		global Sequence, marker_list, objectList
		global vxmy, vymy, wmy, selfid
		count = len(marker_data.markers)
		car_count = 0.0
		person = False
		if len(self.smoother_list) != count:
			self.initialize_smoothers(count)
		
		for i in range(count):
			if count != 0:
				car_count += 1
				if marker_data.markers[i].scale.y * marker_data.markers[i].scale.x * marker_data.markers[i].scale.z <= 0.19*0.54*1.44: 
					ttype = 4
				else:
					ttype = 1
						
				# if 	count > 2:	
				# 	print(marker_data.markers[2])
				if marker_data.markers[i].text == "car": ttype = 1.0
				if marker_data.markers[i].text == "rider": ttype = 2.0
				if marker_data.markers[i].text == "pedestrian": ttype = 4.0; person = True
				current_time = marker_data.markers[i].header.stamp.to_sec()
				

				if objectList[i][12] != 0:  
					elapsed_time = current_time - objectList[i][12]
					distance = np.linalg.norm(np.array([marker_data.markers[i].pose.position.x , marker_data.markers[i].pose.position.y]) - np.array([objectList[i][3], objectList[i][4]]))
					
					# print('distance:', distance)
					# print('vxmy:', vxmy)
					# if (abs(marker_data.markers[i].pose.position.y - objectList[i][4])) > 0:
					
					if distance > 0: 
						if elapsed_time > 0:
							DelT = elapsed_time				


							x_ = marker_data.markers[i].pose.position.x 
							y_ = marker_data.markers[i].pose.position.y
							# yaw = pi + atan2(y_,x_)
							# direction = np.array([x_ , y_]) - np.array([objectList[i][3], objectList[i][4]])
							# yaw = atan2(direction[1], direction[0])
							# yaw = self.smoother_list[i].update(yaw)
							# print('time:', DelT)
							# print('y:', y_)
							vx = ( x_ - objectList[i][3])/ (DelT)
							vy = ( y_ - objectList[i][4])/ (DelT)
							
							# print('vx:', vx)
							# omega = (marker_data.markers[i].pose.orientation.z - objectList[i][5])/DelT
							# omega = (wmy + omega)
						
							vx += vxmy
							vy += vymy	
							v = np.hypot(vx, vy)
							# print('V:', v)
							# vy = 1.1
							# vx = 0
							# print('vx:', vx,  'vy:', vy)			
							# vy = -3
							# if marker_data.markers[i].text == "unknown" :
							# 	distance = 0.05  
							# 	vx = 0
							# 	vy = 0
							# print('distance:', distance, 'speed:', v)
							# print(v, marker_data.markers[i].id)
							# vx = 0
							if v < 1:							
								if y_ >= 0:
									yaw = -pi/2
								else:
									yaw = - pi/2 + pi/40						
							else:								
								yaw = atan2(vy,vx)
								yaw = self.smoother_list[i].update(yaw)	

							if ttype == 4:
								if v > 0.3: 
									yaw = atan2(vy,vx)
									yaw = self.smoother_list[i].update(yaw)	
								else:
									vx = 0
									vy = 0
									if y_ >= 0:
										yaw = -pi/2
									else:
										yaw = - pi/2 + pi/40				

							if marker_data.markers[i].pose.position.y > 8 or marker_data.markers[i].pose.position.y < -8:
								vx = 0
								vy = 0
								if y_ >= 0:
									yaw = -pi/2
								else:
									yaw = - pi/2 + pi/40	


						
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
						yaw = objectList[i][5]
						# omega = objectList[i][7] 
					
					# if vxmy > 0.1:

					# 	if ( x_ - objectList[i][3]) < 0 and ( y_ - objectList[i][4]) < 0:
					# 		yaw = marker_data.markers[i].pose.orientation.z - (marker_data.markers[i].pose.orientation.z) - pi/2
					# 	elif ( x_ - objectList[i][3]) < 0 and ( y_ - objectList[i][4]) > 0:
					# 		yaw = marker_data.markers[i].pose.orientation.z - (marker_data.markers[i].pose.orientation.z) + pi/2
					# elif vxmy < 0.1:

					# 	if ( x_ - objectList[i][3]) > 0 and ( y_ - objectList[i][4]) < 0:
					# 		yaw = marker_data.markers[i].pose.orientation.z - (marker_data.markers[i].pose.orientation.z) - pi/2
					# 	elif ( x_ - objectList[i][3]) < 0 and ( y_ - objectList[i][4]) > 0:
					# 		yaw = marker_data.markers[i].pose.orientation.z - (marker_data.markers[i].pose.orientation.z) + pi/2
					

					# print('yaw:', yaw)
					# print('vx:', vx, 'vy:', vy)
					objectList[i][0] = marker_data.markers[i].id
					objectList[i][1] = ax
					objectList[i][2] = ay 
					objectList[i][3] = x_
					objectList[i][4] = y_
					objectList[i][5] = yaw
					objectList[i][6] = ttype
					# objectList[i][7] = omega			
					objectList[i][8] = vx
					objectList[i][9] = vy
							
					objectList[i][10] = marker_data.markers[i].scale.y #w
					objectList[i][11] = marker_data.markers[i].scale.x #h

					
					# msg = Float32()
					# msg.data = round(risk,2)
					# collision_risk_3_pub.publish(msg)
					
				else:
					objectList[i][12] = current_time
				
				

		# if ttype == 4:
		# 	risk = prob_machine_pedc(car_count,originX, originY -20, objectList, vxmy, vymy, t)
		# 	risk_2 = prob_machine_pedc(car_count,originX, originY -20, objectList, vxmy, vymy, 2)
		# else:	
		risk = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, t)
		risk_2 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 2)
		risk_1 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1)
		# print('risk_3:', round(risk[0],3), round(risk[1],3), 'risk_2:', round(risk_2[0],3), round(risk_2[1],3), 'risk_1:', round(risk_1[0],3), round(risk_1[1],3))	
		# print(f'risk_3: {round(risk[0], 3):<12} {round(risk[1], 3):<12} risk_2: {round(risk_2[0], 3):<12} {round(risk_2[1], 3):<12} risk_1: {round(risk_1[0], 3):<12} {round(risk_1[1], 3):<12}')

		rsp_1 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, t)
		rsp_2 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 2)
		
		# rsp_3 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1)
		rsp_4 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1)
		# # # rsp_4 = prob_machine_gridgen(car_count,originX, originY -40, objectList, vxmy, vymy, 1.25)
		rsp = rsp_1 + rsp_2 + rsp_4
		
		rsp[rsp > 1] = 1
		gridMap_generator(rsp)
	
	def initialize_smoothers(self, count):
	
		self.smoother_list = [OrientationSmoothingFilter(5) for _ in range(count)]
	
	


def vel_sub(vel):
	global vxmy, vymy, wmy
	vxmy, vymy = vel.twist.twist.linear.x, vel.twist.twist.linear.y
	wmy = vel.twist.twist.angular.z
	

#################################################################################################### section 6 main body

if __name__ == '__main__':
	
	
	try:
		rospy.init_node('motion_models', anonymous=True)
	
		processor = MarkerProcessor()
		rospy.Subscriber("/fused_detection", MarkerArray, processor.callback_sub)
		
		
		rospy.Subscriber("/ackermann_steering_controller/odom", Odometry, vel_sub)
		grid_pub = rospy.Publisher("/rspaceGrid2", GridMap, queue_size=1 )
		collision_risk_3_pub = rospy.Publisher('/risk_3', Float32, queue_size=1)


		rospy.spin()
	except rospy.ROSInterruptException:
		pass