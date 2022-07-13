#!/usr/bin/env python3

"""
ICP visual odometry from open3d
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import argparse
from cmath import e
import copy
import json
import math
import os
from collections import OrderedDict
from cv2 import transform
from matplotlib.collections import Collection

import numpy as np
import open3d as o3d
import cv2
import tf
from colorama import Style, Fore
from atom_evaluation.utilities import atomicTfFromCalibration
from atom_core.atom import getTransform
from atom_core.dataset_io import addNoiseToInitialGuess, saveResultsJSON
from atom_core.vision import depthToPointCloud

def drawRegistrationResults(source, target, transformation, initial_transformation):
    """
    Visualization tool for open3d
    """
    source_temp = copy.deepcopy(source)
    source_initial_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([0, 1, 1])
    source_initial_temp.paint_uniform_color([0, 0.5, 0.5])
    target_temp.paint_uniform_color([1, 0, 0])
    source_temp.transform(transformation)
    source_initial_temp.transform(initial_transformation)
    o3d.visualization.draw_geometries([source_initial_temp, source_temp, target_temp])

def pickPoints(pcd):
    """
    Open a window to pick points in order to align two pointclouds
    """
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press q for close the window")
    print("3) Press [shift + '] or [shift + «] to increase or decrease the size of the picked points")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def ICPTransformations(source_point_cloud, target_point_cloud, threshold, T_init, show_images):
    """
    Use ICP to estimate the transformation between two range sensors
    """
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_point_cloud, target_point_cloud, threshold, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)

    if show_images:
        drawRegistrationResults(source_point_cloud, target_point_cloud, reg_p2p.transformation, T_init)

    return reg_p2p


def saveICPTransforms(dataset, transforms, json_file, od):
    """
    Save a JSON file with the data of the ICP visual odometry
    """
    parent_link = 'odom'
    child_link = 'base_footprint'
    frame = parent_link + '-' + child_link
    for collection_key, collection in od.items():
        transform = transforms[collection_key]
        quat = tf.transformations.quaternion_from_matrix(transform)
        dataset['collections'][collection_key]['transforms'][frame]['quat'] = quat
        dataset['collections'][collection_key]['transforms'][frame]['trans'] = transform[0:3, 3]

    # Save results to a json file
    saveResultsJSON(json_file, dataset)



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str,
                    required=True)
    ap.add_argument("-s", "--sensor", help="Sensor used.", type=str, required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']
    show_images = args['show_images']

    # Read json file
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Define variables
    threshold = 0.1
    od = OrderedDict(sorted(dataset['collections'].items(), key=lambda t: int(t[0])))
    pointclouds = {}
    transforms = {}
    filename_results_json = os.path.dirname(json_file) + '/dataset_corrected_odom.json'

    print(Fore.YELLOW + '\nRetrieving pointclouds\n' + Style.RESET_ALL)

    for collection_key, collection in od.items():
        print('Reading pointcloud from collection ' + collection_key)
        # Retrieve pointclouds
        if dataset['calibration_config']['sensors'][args['sensor']]['modality'] == 'lidar3d':
            filename = os.path.dirname(
                args['json_file']) + '/' + dataset['collections'][collection_key]['data'][args['sensor']]['data_file']
            point_cloud = o3d.io.read_point_cloud(filename)
            pointclouds[collection_key] = point_cloud
        elif dataset['calibration_config']['sensors'][args['sensor']]['modality'] == 'depth':
            filename = os.path.dirname(json_file) + '/' + \
                        dataset['collections'][collection_key]['data'][args['sensor']]['data_file']
            img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
            pixels = img.shape[0] * img.shape[1]
            idxs_to_trim = range(pixels)
            idxs = [idx for idx in idxs_to_trim if not idx % args['sub_sample_depth']]
            point_cloud = depthToPointCloud(dataset, collection_key, args['json_file'], args['sensor'], idxs)
            pointclouds[collection_key] = point_cloud
        else:
            print('The sensor ' + args['sensor'] +  ' does not have a pointcloud to retrieve, so ICP would not work.\nShutting down...')
            exit(0)

    print('Source collection is ' + list(od.keys())[0])    

    od = {'001': None, '002': None, '003': None}

    # Defining source points
    source_point_cloud = pointclouds[list(od.keys())[0]]
    while True:
            source_picked_points = pickPoints(source_point_cloud)
            # Conditions of alignment
            if not (len(source_picked_points) >= 3):
                print('\nYou have chosen less than 3 points in the pointcloud, please redo it.')
                continue            
            else:
                break    

    for collection_key, collection in od.items():
        target_point_cloud = pointclouds[collection_key]
        # While cycle to force the user to choose the correct number of points
        while True:
            # Align the pointclouds
            print('\nAligning pointclouds of collection ' + collection_key)
            target_picked_points = pickPoints(target_point_cloud)

            # Conditions of alignment
            if not (len(source_picked_points) >= 3 and len(target_picked_points) >= 3):
                print('\nYou have chosen less than 3 points in the pointcloud, please redo it.')
                continue            
            elif not (len(source_picked_points) == len(target_picked_points)):
                print(f'\nYou have chosen {len(source_picked_points)} and {len(target_picked_points)} points, which needed to be equal, please redo them.')
                continue       
            else:
                break    

        corr = np.zeros((len(source_picked_points), 2))
        corr[:, 0] = source_picked_points
        corr[:, 1] = target_picked_points

        # Estimate rough transformation using correspondences
        print("Compute a rough transform using the correspondences given by user")
        p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        T_target_to_source = p2p.compute_transformation(source_point_cloud, target_point_cloud,
                                                o3d.utility.Vector2iVector(corr))
        
        # Aligning using ICP
        print('T_target_to_source = \n' + str(T_target_to_source))
        reg_p2p = ICPTransformations(source_point_cloud, target_point_cloud, threshold, T_target_to_source, show_images)
        transforms[collection_key] = reg_p2p.transformation
        print(reg_p2p.transformation)

    # Saving the dataset
    print(Fore.YELLOW + '\nSaving odometry transformations'  + Style.RESET_ALL)
    saveICPTransforms(dataset, transforms, filename_results_json, od)



if __name__ == '__main__':
   main() 