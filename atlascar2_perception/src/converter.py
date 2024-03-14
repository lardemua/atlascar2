#!/usr/bin/env python3

import numpy as np
import torch
import torch.nn as nn

class sensor_params:
    def estimate_homography_from_camera_params(self, K1, R1, K2, R2):
        # Compute the intrinsic matrix inverses
        K1_inv = np.linalg.inv(K1)
        K2_inv = np.linalg.inv(K2)
        
        # Compute the homography matrix
        H = np.dot(K2, R2) @ np.linalg.inv(R1) @ K1_inv
    
        return H
    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def __init__(self):

        R_lidar = self.euler_to_rotation_matrix(-2.5211140907686757e-05, 0.0001891034550860639, 0.00040354094775341983)
        T_lidar = np.array([[0.00087525277684157],
                            [5.140457190614583e-05],
                            [1.4507274931216392+0.0377]])
        R_camera_left = self.euler_to_rotation_matrix(3.9247483963657337e-16, 0.3999999999999997, -3.8484785469263477e-16)
        T_camera_left = np.array([[0.2000000000000004],
                            [0.22000000000000017],
                            [1.4500000000000004]])

        R_camera_right = self.euler_to_rotation_matrix(-2.7612334362795395e-05, 0.40000521549500057, 2.3033103860088408e-05)
        T_camera_right = np.array([[0.200077309120255],
                            [-0.21999937612038148],
                            [1.4499624688682895]])

        R_camera_left_to_otica = self.euler_to_rotation_matrix(-1.570796, 0.0, -1.570796)
        T = np.vstack((np.hstack((R_camera_left_to_otica, np.array([[0],[0],[0]]))), [0, 0, 0, 1]))

        self.M_lidar = np.vstack((np.hstack((R_lidar, T_lidar)), [0, 0, 0, 1]))
        # Extrinsic matrix left camera
        self.M_camera_left = np.vstack((np.hstack((R_camera_left, T_camera_left)), [0, 0, 0, 1]))
        # Extrinsic matrix right camera
        self.M_camera_right = np.vstack((np.hstack((R_camera_right, T_camera_right)), [0, 0, 0, 1]))

        # Transformation matrix lidar-left camera
        self.M_lidar_inv = np.linalg.inv(self.M_lidar)
        M_camera_left_inv = np.linalg.inv(self.M_camera_left)
        self.M_lidar_camera = np.dot(np.dot(self.M_lidar_inv, self.M_camera_left),T)
        self.M_lidar_camera_inv = np.linalg.inv(self.M_lidar_camera)
        
        # Intrinsics left camera
        self.K_camera_left = np.array([[1101.581198742484, 0.0, 636.5],
                                [0.0, 1101.581198742484, 508.5],
                                [0.0, 0.0, 1.0]])

        # Intrinsics right camera
        self.K_camera_right = np.array([[1101.581198742484, 0.0, 636.5],
                                [0.0, 1101.581198742484, 508.5],
                                [0.0, 0.0, 1.0]])

        # Projection matrix of the left camera
        self.P_camera_left = np.array([[1101.581198742484, 0.0, 636.5, -77.1106839119739],
                                       [0.0, 1101.581198742484, 508.5, 0.0],
                                       [0.0, 0.0, 1.0, 0.0]])

        
        self.homography = np.array([[ 5.40304579e-01, -2.66912257e-01,  8.46703839e+02],
 [-9.64704842e-04,  8.91943912e-01,  1.34286931e+00],
 [-3.58766786e-04, -3.54147781e-05,  1.00000000e+00]])

        # Projection matrix of the left camera
        self.P_camera_right = np.dot(self.K_camera_right, self.M_camera_right[:3, :4])

params = sensor_params()
# print(params.P_camera_left)


# print(params.M_lidar_camera[:3,3])
