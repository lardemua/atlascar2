'''
@author: Unmesh Patil
@project: Collision risk estimation at CHROMA, INRIA, France (April, 2021)
@description: check out my paper: https://hal.inria.fr/hal-03416222 
About code: This code contains implementation of stochastic motion moels presented in the paper. 
The probabiity machine fuctions recieve object-level data from other codes. 
There are three major functions: 1. Grid generation for visualisation 2. Collision risk estimation 3. Spatial deviation or FDE
These functions use NJIT decorator for fast and parallel processing of entire grid.
'''

from numba import njit
from numba import float64 as f64
from numba import float32 as f32
from numba import int8
import numpy as np

resolution = 0.1
n = int(1/resolution) #number of grid cells per meter
w = 40*n + 1 
h = 40*n + 1
#car model constants
angcons = 7.1
lincons = 4.5
k = 150 #reach of neighbourhood
@njit(f64[:](int8,int8, int8, f64[:,:], f32, f32, int8), nogil=True, fastmath=True, cache=True)
def prob_machine_gridgen(f, originX, originY, objectList, vsx, vsy, t):
	MaxD = n*n*t #m
	rspaceDat = np.zeros((w*h), dtype=np.float64)
	dx = vsx*n*t + 5 # distance x in cells  
	for i in range(f):
		ttype = objectList[i][6]
		if ttype > -1:
			vx, vy = objectList[i][8], objectList[i][9] 
			v = np.hypot(vx, vy)  #speed magnitude of the object
			# print('V:', v) 
			x = (objectList[i][3] - 0.5 - originX) * n #distance x to the car in cells
			y = (objectList[i][4] - originY)* n  
			theta_given = objectList[i][5]       #yaw
			sx = n*objectList[i][10]*0.5         #object dimensions in cells
			sy = n*objectList[i][11]*0.5   
			point_count = 1.0
			point_intr = 0.0
			# print(v)
			if v > 1:
				#distance y to the car in cells
				#h
		
				ax, ay = objectList[i][1], objectList[i][2]       #acell 
				a = np.hypot(ax, ay)				
				# print('A:', a)							  #acell magnitude		
				Vf = v + a*t 
				Dx = x + vx*t + 0.5*ax*t*t
				Dy = y + vy*t + 0.5*ay*t*t
				D = (np.hypot(Dx, Dy))*resolution*t
				if D > MaxD: D = MaxD
				# print ('distance:' , D)
				ymin = int(y - k) if (y - k) > originY else originY
				ymax = int(y + k) if (y + k) < w else w
				xmin = int(x - k) if (x - k) > originX else originX
				xmax = int(x + k) if (x + k) < h else h
				yg = np.arange(ymin, ymax) - y
				factor = v*t*((v-1)/(v+1)) + 0.5*a*t*t*((a-1)/(a+1))
				# factor = 0
				# Vf = 20
				# print('D:', D)
				rspLocal = np.zeros((w*h), dtype=np.float64)
				if ttype < 3:
					for xg in range(xmin, xmax, 1):
						d1 = np.hypot(yg, xg-x) - D
						delth = np.arctan2(yg, xg-x) - theta_given 
						for j in range(len(d1)):
							y_ = int(yg[j] + y)
							deltha, d = delth[j], d1[j]
							if abs(theta_given) > 2.57: #technical issue with yaw angle from data
								if deltha > 6:
									deltha = deltha - 6.28
								elif deltha < -6:
									deltha = deltha + 6.28
					
							Pa = 1 - (deltha*deltha*Vf*angcons/ (t *t))
							Pl = 1 - (d*d / (lincons*factor + 0.0001))
						
							if Pa < 0: Pa = 0
							if Pa > 1: Pa = 1
							if Pl < 0: Pl = 0 #exceptional cases
							p = Pa*Pl
						
							if p > 1: p = 1
							orient = 2*np.arctan((y_-y) / (xg-x)) - theta_given
							if 1 > p > 0:
							
								point_count += p 
								cosi = np.cos(orient) #equations to get an oriented rectangle
								sine = np.sin(orient)
								TRx = xg + (sx * cosi) - (sy * sine)
								TRy = y_ + (sx * sine) + (sy * cosi)
								TLx = xg - (sx * cosi) - (sy * sine)
								TLy = y_ - (sx * sine) + (sy * cosi)
								BLx = xg - (sx * cosi) + (sy * sine)
								BLy = y_ - (sx * sine) - (sy * cosi)
								BRx = xg + (sx * cosi) + (sy * sine)
								BRy = y_ + (sx * sine) - (sy * cosi)
								arx = np.array([ TRx, TLx, BLx, BRx])
								ary = np.array([ TRy, TLy, BLy, BRy])
								xma, xmi, yma, ymi = np.max(arx), np.min(arx), np.max(ary), np.min(ary)
								DT = (TRx - BRx) * (y_ - BRy) - (xg - BRx) * (TRy - BRy)
								DL = (TLx - BLx) * (y_ - BLy) - (xg - BLx) * (TLy - BLy)
								DR = (TLx - TRx) * (y_ - TRy) - (xg - TRx) * (TLy - TRy)
								DB = (BRx - BLx) * (y_ - BLy) - (xg - BLx) * (BRy - BLy)
								for xl in range(xmi, xma, 1): #box operator
									for yi in range(ymi, yma, 1):
										if (w > xl > 0) and (h > yi > 0):
											DTl = (TRx - BRx) * (yi - BRy) - (xl - BRx) * (TRy - BRy)
											if (DTl*DT) > 0:
												DLl = (TLx - BLx) * (yi - BLy) - (xl - BLx) * (TLy - BLy)
												if (DLl*DL > 0):
													DRl = (TLx - TRx) * (yi - TRy) - (xl - TRx) * (TLy - TRy)
													if (DRl*DR) > 0:
														DBl = (BRx - BLx) * (yi - BLy) - (xl - BLx) * (BRy - BLy)
														if (DBl*DB) > 0:
															rspLocal[h*yi + xl] += p
							
				if ttype > 3: #pedestrian
					D_ped = v*t
					pedmax = 3*t #maximum range of pedestrian
					if D_ped > pedmax: D_ped = pedmax
					for xg in range(xmin, xmax, 1):
						d1 = resolution*np.hypot(yg, xg-x) - D_ped
						delth = np.arctan2(yg, xg-x) - theta_given
						for j in range(len(d1)):
							y_ = int(yg[j] + y)
							deltha, d = delth[j], d1[j]
							Pa = 1 - abs(1.2*np.sin(deltha/2))
							Pl = 1 - (d*d /pedmax) 
							if Pa < 0: Pa = 0
							if Pa > 1: Pa = 1
							if Pl < 0: Pl = 0
							p = Pl*Pa
							if 1 > p > 0.3 :
								point_count = 1
								if (w > xg > 0) and (h > y_ > 0):
									rspLocal[h*y_ + xg] = p
				if point_count>0: rspLocal = rspLocal / point_count #gives normalized dist of COM
				for z in range(w*h):
					if rspaceDat[z] < rspLocal[z]: rspaceDat[z] = rspLocal[z]
					
			if v < 1: #static object
		
				cosi = np.cos(theta_given)  #0
				sine = np.sin(theta_given)  #1
				arx = np.array([(sx  * cosi) - (sy * sine), (sx  * cosi) + (sy * sine), -(sx  * cosi) - (sy * sine), -(sx  * cosi) + (sy * sine)])
				ary = np.array([(sx  * sine) + (sy * cosi), -(sx  * sine) + (sy * cosi), -(sx  * sine) - (sy * cosi), (sx  * sine) - (sy * cosi)])
				Rxmax, Rxmin = np.max(arx), np.min(arx)
				Rymax, Rymin = np.max(ary), np.min(ary)
			
				xma, xmi, ymi, yma = int(x + Rxmax), int(x + Rxmin), int(y + Rymin), int(y+ Rymax)
				for xl in range(xmi, xma, 1):
					for yi in range(ymi, yma, 1):
				
						if (w > xl > 0) and (h > yi > 0): 						
							rspaceDat[h*yi + xl] = 1	
											
	for x in range(0, dx, 1):
		y = 200 + (x*x*vsy / (26*vsx + 0.01)) #ego vehicle trajectory
		for y in range(y - 10, y + 10):
			rspaceDat[int(h*y+ x)] = 1

	return rspaceDat


@njit(f64(int8, int8, int8, f64[:,:], f64, f64, f64), nogil=True, fastmath=True, cache=True)
def prob_machine_riskgen(f, originX, originY, objectList, vsx, vsy, t):
	MaxD = 10*n*t #m
	rspaceDat = np.zeros((w*h), dtype=np.float64)
	dx = vsx*n*t + 5
	for i in range(f):
		ttype = objectList[i][6]
		oid = objectList[i][0]
		x = (objectList[i][3] - 0.5 - originX) * n 
		y = (objectList[i][4] - originY)* n
		vx = objectList[i][8]
		vy = objectList[i][9]
		v = np.hypot(vx, vy)
		theta_given = objectList[i][5]
		
		sx = n*objectList[i][10]*0.5
		sy = n*objectList[i][11]*0.5 #h
		point_count = 1.0
		if v > 1:
			omega = 0
			ax = objectList[i][1]
			ay = objectList[i][2]
			a = np.hypot(ax, ay)
			Vf = v + a*t 
			Dx = x + vx*t + 0.5*ax*t*t
			Dy = y + vy*t + 0.5*ay*t*t
			D = (np.hypot(Dx, Dy))*resolution*t
			if D > MaxD: D = MaxD
			ymin = int(y - k) if (y - k) > originY else originY
			ymax = int(y + k) if (y + k) < w else w
			xmin = int(x - k) if (x - k) > originX else originX
			xmax = int(x + k) if (x + k) < h else h
			yg = np.arange(ymin, ymax) - y
			factor = v*t*((v-1)/(v+1)) + 0.5*a*t*t*((a-1)/(a+1))
			rspLocal = np.zeros((w*h), dtype=np.float64)
			point_intr = 0
			if ttype < 3 :
				for xg in range(xmin, xmax, 1):
					d1 = np.hypot(yg, xg-x) - D
					delth = np.arctan2(yg, xg-x) - theta_given - omega
					for j in range(len(d1)):
						y_ = int(yg[j] + y)
						deltha, d = delth[j], d1[j]
						if abs(theta_given) > 2.57: 
							if deltha > 6:
								deltha = deltha - 6.28
							elif deltha < -6:
								deltha = deltha + 6.28
						Pa = 1 - (deltha*deltha*Vf*angcons/ (t *t))
						Pl = 1 - (d*d / (lincons*factor + 0.0001))
						if Pa < 0: Pa = 0
						if Pa > 1: Pa = 1
						if Pl < 0: Pl = 0
						p = Pa*Pl
						if p > 1: p = 1
						orient = 2*np.arctan((y_-y) / (xg-x)) - theta_given
						if p > 0:
							point_count += p
							cosi = np.cos(orient)
							sine = np.sin(orient)
							arx = np.array([(sx  * cosi) - (sy * sine), (sx  * cosi) + (sy * sine), -(sx  * cosi) - (sy * sine), -(sx  * cosi) + (sy * sine) ])
							ary = np.array([(sx  * sine) + (sy * cosi), -(sx  * sine) + (sy * cosi), -(sx  * sine) - (sy * cosi), (sx  * sine) - (sy * cosi)])
							Rxmax, Rxmin = np.max(arx), np.min(arx)
							Rymax, Rymin = np.max(ary), np.min(ary)
							xma, xmi, ymi, yma = int(xg + Rxmax), int(xg + Rxmin), int(y_ + Rymin), int(y_ + Rymax)
							for xl in range(xmi, xma, 1):
								for yi in range(ymi, yma, 1):
									rspLocal[h*yi + xl] += p
									if (0 < xg < dx):
										yrange = 200 + (xg*xg*vsy / (26*vsx + 0.01))
										if (yrange - 10 < y_ < yrange+10):
											if rspLocal[h*yi + xl]  > point_intr: point_intr = rspLocal[h*yi + xl]
			if ttype > 3:
				D_ped = v*t
				pedmax = 3*t #maximum range of pedestrian
				if D_ped > pedmax: D_ped = pedmax
				for xg in range(xmin, xmax, 1):
					d1 = resolution*np.hypot(yg, xg-x) - D_ped
					delth = np.arctan2(yg, xg-x) - theta_given
					for j in range(len(d1)):
						y_ = int(yg[j] + y)
						deltha, d = delth[j], d1[j]
						Pa = 1 - abs(1.2*np.sin(deltha/2))
						Pl = 1 - (d*d /pedmax) #1 - (d + 0.8*D)*(d + 0.8*D)/(D*t*2 + 0.001)
						if Pa < 0: Pa = 0
						if Pa > 1: Pa = 1
						if Pl < 0: Pl = 0
						p = Pl*Pa
						if 1 > p > 0:
							point_count = 1
							rspLocal[h*y_ + xg] = p
							if (0 < xg < dx):
								yrange = 200 + (xg*xg*vsy / (26*vsx + 0.01))
								if (yrange - 10 < y_ < yrange+10):
									if rspLocal[h*yi + xl]  > point_intr: point_intr = rspLocal[h*yi + xl]
		if v < 1:
			cosi = np.cos(theta_given)
			sine = np.sin(theta_given)
			arx = np.array([(sx  * cosi) - (sy * sine), (sx  * cosi) + (sy * sine), -(sx  * cosi) - (sy * sine), -(sx  * cosi) + (sy * sine) ])
			ary = np.array([(sx  * sine) + (sy * cosi), -(sx  * sine) + (sy * cosi), -(sx  * sine) - (sy * cosi), (sx  * sine) - (sy * cosi)])
			Rxmax, Rxmin = np.max(arx), np.min(arx)
			Rymax, Rymin = np.max(ary), np.min(ary)
			xma, xmi, ymi, yma = int(x + Rxmax), int(x + Rxmin), int(y + Rymin), int(y+ Rymax)
			for xl in range(xmi, xma, 1):
				for yi in range(ymi, yma, 1):
					if (400 > xl > 0) and (400 > yi > 0): 
						if (0 < xl < dx):
							yrange = 200 + (xl*xl*vsy / (26*vsx + 0.01))
							if (yrange - 10 < yi < yrange+10):
								point_intr = 1.0
	if point_count > 0: point_intr = point_intr / point_count
	return point_intr

@njit(f64(int8, int8,int8,f64[:,:], f64, f64, f64), nogil=True, fastmath=True, cache=True)
def prob_machine_pedc(f,originX, originY, objectList, vsx, vsy, t):
	rspaceData = np.zeros((w*h), dtype=np.float64)
	dx = vsx*n*t + 5
	point_intr = 0.0
	for i in range(f):
		x = (objectList[i][3] - 0.5 - originX)* n 
		y = (objectList[i][4] - originY)* n
		vx = objectList[i][8]
		vy = objectList[i][9]
		v = np.hypot(vx, vy)
		theta_given = objectList[i][5]
		point_count = 0.0
		ymin = int(y - k) if (y - k) > 0 else 0
		ymax = int(y + k) if (y + k) < w else w
		xmin = int(x - k) if (x - k) > 0 else 0
		xmax = int(x + k) if (x + k) < h else h
		yg = np.arange(ymin, ymax) - y
		rspLocal = np.zeros((w*h), dtype=np.float64)
		D_ped = v*t
		pedmax = 3*t #maximum range of pedestrian
		if D_ped > pedmax: D_ped = pedmax
		for xg in range(xmin, xmax, 1):
			d1 = resolution*np.hypot(yg, xg-x) - D_ped
			delth = np.arctan2(yg, xg-x) - theta_given
			for j in range(len(d1)):
				y_ = int(yg[j] + y)
				deltha, d = delth[j], d1[j]
				Pa = 1 - abs(1.2*np.sin(deltha/2))
				Pl = 1 - (d*d /pedmax)
				if Pa < 0: Pa = 0
				if Pa > 1: Pa = 1
				if Pl < 0: Pl = 0
				p = Pl*Pa
				if 1 > p > 0:
					point_count += p
					rspLocal[h*y_ + xg] += p
					if (0 < xg < dx):
						yrange = 200 + (xg*xg*vsy / (26*vsx + 0.01))
						if (yrange - 10 < y_ < yrange+10):
							if rspLocal[h*y_ + xg]  > point_intr: 
								point_intr = rspLocal[h*y_ + xg]
							
	return point_intr