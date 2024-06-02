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
import csv
from visualization_msgs.msg import MarkerArray
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32
from nav_msgs.msg import Odometry
from probability_machine_2 import prob_machine_gridgen, prob_machine_riskgen
from math import atan2, pi
from collections import Counter



########################################################################### section 2 defaults and global vars
marker_array_ = MarkerArray()

selfid = 1
pedestrian = True
objectList =  np.zeros((16,16), dtype=np.float64)
yaw_list = [[] for _ in range(15)]
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

marker_dict = {}
id_to_new_id = {}
new_id_counter = 0

frame = 0
frame_1 = 0
#################################################################################### section 4 helper functions
def sum_across_rows(*arrays):
    total_sum = np.zeros(100)
    for array in arrays:
        for row in array:
            total_sum += row[1]
    return total_sum
	

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
# class MarkerProcessor:
# 	def __init__(self):
# 		self.smoother_list = []

def callback_sub(marker_data):
	global Sequence, marker_list, objectList, marker_dict, id_to_new_id, new_id_counter, yaw_list
	global vxmy, vymy, wmy, selfid, frame, frame_1
	count = len(marker_data.markers)
	car_count = 0.0	
	stopped = 0
	actual_ids = []		


	for marker in marker_data.markers:
		
		if marker.id not in id_to_new_id:
		
			id_to_new_id[marker.id] = new_id_counter
			new_id_counter += 1

		new_id = id_to_new_id[marker.id]
		actual_ids.append(new_id)
		marker_dict[new_id] = marker  # Assuming 'mark' is a typo and it should be 'marker'


	# if len(self.smoother_list) < len(marker_dict):
	# 	self.initialize_smoothers(len(marker_dict) - len(self.smoother_list))

	# print(vxmy)
	# vxmy = 2
	vymy = 0.5

	for i in marker_dict:
		marker = marker_dict[i]
		# if i in actual_ids:
		if count != 0:
			car_count += 1
			if marker.scale.y * marker.scale.x * marker.scale.z <= 0.148 : # 0.19*0.54*1.44 
				ttype = 4
			else:
				ttype = 1
					
			# if 	count > 2:	
			# 	print(marker_data.markers[2])
			if marker.text == "car": ttype = 1.0
	
			if marker.text == "train": ttype = 1.0
			if marker.text == "pedestrian": ttype = 4.0
			current_time = marker.header.stamp.to_sec()
			
			

			if objectList[i][12] != 0:  
				elapsed_time = current_time - objectList[i][12]


				distance = np.linalg.norm(np.array([marker.pose.position.x , marker.pose.position.y]) - np.array([objectList[i][3], objectList[i][4]]))
				
				# print('distance:', distance, "elapsed:" ,elapsed_time)
				
				# print('vxmy:', vxmy)
				# if (abs(marker.pose.position.y - objectList[i][4])) > 0:
				
				if distance > 0: 
	
					if elapsed_time > 0:
						DelT = elapsed_time	
						# print(DelT)			


						x_ = marker.pose.position.x 
						y_ = marker.pose.position.y
						# yaw = pi + atan2(y_,x_)
						# direction = np.array([x_ , y_]) - np.array([objectList[i][3], objectList[i][4]])
						# yaw = atan2(direction[1], direction[0])
						# yaw = self.smoother_list[i].update(yaw)
						# print('time:', DelT)
						# print('y:', y_)
						vx = ( x_ - objectList[i][3])/ (DelT)
						vy = ( y_ - objectList[i][4])/ (DelT)
						# if  marker.pose.position.y > 0:
						# 	print(vx, vy)
						# print('vx:', vx)
						# omega = (marker.pose.orientation.z - objectList[i][5])/DelT
						# omega = (wmy + omega)
				
						# print(v)
						vx += vxmy
						vy += vymy	
						v = np.hypot(vx, vy)
						# print(v)
						# if marker.pose.position.y < -2: 
						# 	if vx > 0.1:
						# 		vx = 0.1
						# 	if abs(vy) > 0.7:
						# 		vy = 0.6
						
						# print('V:', v)
						# vy = 1.1
						# vx = 0
						# print('vx:', vx,  'vy:', vy)			
						# vy = -3
						# if marker.text == "unknown" :
						# 	distance = 0.05  
						# 	vx = 0
						# 	vy = 0
						# print('distance:', distance, 'speed:', v)
			
						# vx = 0
						# if marker.text == "truck":
						# 	print(v, i)
						# if v < 1:							
						# 	if y_ >= 0:
						# 		yaw = -pi/2
						# 	else:
						# 		yaw = - pi/2 + pi/40						
						# else:								
						yaw = atan2(vy,vx)
						# if v < 50:
						yaw_list[i].append(yaw % (2*pi))						

						if len(yaw_list[i]) > 3:
							yaw_list[i] = yaw_list[i][-3:]				

						smoothed_yaw = sum(yaw_list[i])/len(yaw_list[i])
						# smoothed_yaw = yaw
						# else:
						# 	smoothed_yaw = 0
						# if marker.pose.position.y > 0:
						# 	print(yaw_list[i])
						# smoothed_yaw = yaw
							# yaw = -pi/2
							# yaw = self.smoother_list[i].update(yaw)	
			
						# if ttype == 4:
						# 	if v > 0.3: 
						# 		yaw = atan2(vy,vx)
						# 		# yaw = self.smoother_list[i].update(yaw)	
						# 	else:
						# 		vx = 0
						# 		vy = 0
						# 		if y_ >= 0:
						# 			yaw = -pi/2
						# 		else:
						# 			yaw = - pi/2 + pi/40				

						# if  marker.scale.y < 2 and marker.text == "unknown": 
						# 	ttype = 2
						if marker.pose.position.y > -1:
							ttype = 2
							# if y_ >= 0:
							# 	smoothed_yaw = -pi/2
							# else:
							# 	smoothed_yaw = - pi/2 + pi/40	
					


					
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
						smoothed_yaw = objectList[i][5]
						ttype = objectList[i][6] 

				else:
					vx = objectList[i][8]
					vy = objectList[i][9]
					ax = objectList[i][1]
					ay = objectList[i][2]
					x_ = objectList[i][3] 
					y_ = objectList[i][4]
					smoothed_yaw = objectList[i][5]
					ttype = objectList[i][6]
					# omega = objectList[i][7] 
				
				# if vxmy > 0.1:

				# 	if ( x_ - objectList[i][3]) < 0 and ( y_ - objectList[i][4]) < 0:
				# 		yaw = marker.pose.orientation.z - (marker.pose.orientation.z) - pi/2
				# 	elif ( x_ - objectList[i][3]) < 0 and ( y_ - objectList[i][4]) > 0:
				# 		yaw = marker.pose.orientation.z - (marker.pose.orientation.z) + pi/2
				# elif vxmy < 0.1:

				# 	if ( x_ - objectList[i][3]) > 0 and ( y_ - objectList[i][4]) < 0:
				# 		yaw = marker.pose.orientation.z - (marker.pose.orientation.z) - pi/2
				# 	elif ( x_ - objectList[i][3]) < 0 and ( y_ - objectList[i][4]) > 0:
				# 		yaw = marker.pose.orientation.z - (marker.pose.orientation.z) + pi/2
				

				# print('yaw:', yaw)
				# print('vx:', vx, 'vy:', vy)
				objectList[i][0] = marker.id
				objectList[i][1] = ax
				objectList[i][2] = ay 
				objectList[i][3] = x_
				objectList[i][4] = y_
				objectList[i][5] = smoothed_yaw
				objectList[i][6] = ttype		
				objectList[i][8] = vx
				objectList[i][9] = vy
				if marker.text == "truck":
					objectList[i][10] = marker.scale.x #w
					objectList[i][11] = marker.scale.y #h
				else:
					objectList[i][10] = marker.scale.y #w
					objectList[i][11] = marker.scale.x #h
	
				
			else:
				objectList[i][12] = current_time

	for i in range(len(objectList)):
		if i not in actual_ids:
			
			objectList[i] = [0] * len(objectList[0])
		
		

	# if ttype == 4:
	# 	risk = prob_machine_pedc(car_count,originX, originY -20, objectList, vxmy, vymy, t)
	# 	risk_2 = prob_machine_pedc(car_count,originX, originY -20, objectList, vxmy, vymy, 2)
	# else:	

	risk = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, t)
	risk_2 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 2)
	risk_1 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1)
	# risk_3 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 2.5)
	# risk_5 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1.5)
	# risk_6 = prob_machine_riskgen(car_count,originX, originY -20, objectList, vxmy, vymy, 0.5)
	# print('risk_3:', round(risk[0],3), round(risk[1],3), 'risk_2:', round(risk_2[0],3), round(risk_2[1],3), 'risk_1:', round(risk_1[0],3), round(risk_1[1],3))	
	# print(f'risk_3: {round(risk[0], 3):<12} {round(risk[1], 3):<12} risk_2: {round(risk_2[0], 3):<12} {round(risk_2[1], 3):<12} risk_1: {round(risk_1[0], 3):<12} {round(risk_1[1], 3):<12}')
	# print("risk:" , risk.flatten().tolist())
	rsp_1 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, t)
	rsp_2 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 2)
	# rsp_5 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1.5)
	# rsp_6 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 2.5)
	# rsp_7 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 0.5)
	
	
	# rsp_3 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1)
	rsp_4 = prob_machine_gridgen(car_count,originX, originY -20, objectList, vxmy, vymy, 1)
	# # # rsp_4 = prob_machine_gridgen(car_count,originX, originY -40, objectList, vxmy, vymy, 1.25)
	# for i in range(len(rsp_1)):
	rsp = rsp_1 + rsp_2 + rsp_4
	risk_4 = risk + risk_1 + risk_2

	# risk_4 = risk[0,:] + risk_2[0,:] + risk_1[0,:] 
	
	# print(rsp_1)
	rsp[rsp > 1] = 1
	risk_4[risk_4 > 1] = 1
	gridMap_generator(rsp)

	result = [round(x,3) for x in risk_4[:-1] if x > 0]
	if result:
		max_value = max(result)
		if len(result) >= 20:
			highest_20 = np.sort(np.partition(result, -20)[-20:])
		else:
			highest_20 = result
		medium_value = np.mean(highest_20)
		counter = Counter(highest_20)
		most_common = counter.most_common(1)
		most_common_value = most_common[0][0]
		std = np.std(highest_20)
		limite_superior = medium_value + 2 * std
		limite_inferior = medium_value - 2 * std
		# print(most_common)
		print("max:", max_value, "medium:", medium_value, "median:", most_common_value, "desvio:", std)
		frame += 1
		print(frame)

	else:
		max_value = 0
		medium_value = 0
		most_common_value = 0
		std = 0

	frame_1 += 1
	print(frame_1)
	
		# valores_filtrados = [x for x in result if (x <= limite_inferior or x >= limite_superior)]
		# print(valores_filtrados)
	# if frame_1 > 30:

	# 	file_exists = os.path.isfile('/home/rafael/Dados/Dados_sim/stopped.csv')
	# 	with open('/home/rafael/Dados/Dados_sim/stopped.csv', mode='a', newline='') as file:
	# 		writer = csv.writer(file)
	# 		if not file_exists:
	# 			writer.writerow(["Frame", "maxvalA", "meanA", "stdevA", "medianA", "maxvalB", "meanB", "stdevB", "medianB"])
	# 			# writer.writerow(["Frame", "maxval", "mean", "stdev", "median"])
	# 		# Write the values
	# 		if frame_1 <= 65:		
	# 			writer.writerow([frame_1-30, max_value, round(medium_value,3), round(std,3), most_common_value, 0,0,0,0])

	# 		else:
				
	# 			writer.writerow([frame_1-30, 0,0,0,0, max_value, round(medium_value,3), round(std,3), most_common_value])
	
			# writer.writerow([frame, max_value, round(medium_value,3), round(std,3), most_common_value])




	
	# def initialize_smoothers(self, count):
	
	# 	self.smoother_list.extend([OrientationSmoothingFilter(5) for _ in range(count)])
	
	


def vel_sub(vel):
	global vxmy, vymy, wmy
	vxmy, vymy = vel.twist.twist.linear.x, vel.twist.twist.linear.y
	wmy = vel.twist.twist.angular.z
	
	

#################################################################################################### section 6 main body

if __name__ == '__main__':
	
	
	try:
		rospy.init_node('risk_maps', anonymous=False)
	
		# processor = MarkerProcessor()
		rospy.Subscriber("/fused_detection", MarkerArray, callback_sub)
		
		
		rospy.Subscriber("/ackermann_steering_controller/odom", Odometry, vel_sub)
		# rospy.Subscriber("/odom", Odometry, vel_sub)
		grid_pub = rospy.Publisher("/rspaceGrid", GridMap, queue_size=1)
		# collision_risk_3_pub = rospy.Publisher('/risk_3', Float32, queue_size=1)


		rospy.spin()
	except rospy.ROSInterruptException:
		pass