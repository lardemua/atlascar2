#!/usr/bin/python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from camera_perception.msg import detect2d
import copy
import numpy as np
import yaml
from yaml.loader import SafeLoader
from pathlib import Path
import math
import time
from converter import sensor_params
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray 
import tf2_ros
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped, do_transform_point
from visualization_msgs.msg import MarkerArray, Marker
from math import atan2

fps = 0.001
max_time = rospy.Duration.from_sec(1/fps)

counter = 0

def get_bounding_box_corners(center, dimensions):
    # Calculate half-dimensions
    half_width = dimensions[0] / 2.0
    half_height = dimensions[1] / 2.0
    half_depth = dimensions[2] / 2.0

    # Compute coordinates of the eight corners
    corners = [
        (center[0] - half_width, center[1] + half_height, center[2] + half_depth),  
        (center[0] + half_width, center[1] + half_height, center[2] + half_depth),  
        (center[0] - half_width, center[1] - half_height, center[2] + half_depth),  
        (center[0] + half_width, center[1] - half_height, center[2] + half_depth), 
        (center[0] - half_width, center[1] + half_height, center[2] - half_depth),  
        (center[0] + half_width, center[1] + half_height, center[2] - half_depth),  
        (center[0] - half_width, center[1] - half_height, center[2] - half_depth), 
        (center[0] + half_width, center[1] - half_height, center[2] - half_depth)   
    ]
    return corners

def get_color_range(data):
    # Define a different color for each object class or instance
    frac, intNum = math.modf(180/(len(data)))
    intNum = int(intNum)
    frac = int(round(math.prod((frac, (len(data)))))) 
    output_cls = {}
    first_val = -1
    second_val = 0
    for name in data:
        if frac > 0:
            second_val = first_val + intNum + 1
            output_cls[data[name]] = [first_val+1, second_val]
            first_val = second_val
            frac -=1
        else:
            second_val = first_val + intNum
            output_cls[data[name]] = [first_val+1, second_val]
            first_val = second_val
    return output_cls

def isAllEmpty(dictionary):
    counter = 0
    for key in dictionary:
        counter += len(dictionary[key])
    return counter == 0

def most_frequent_yolo_class(label_freq_dict):
    max_freq = 0
    most_freq_class = None
    for yolo_class, frequency in label_freq_dict.items():
        if frequency > max_freq:
            max_freq = frequency
            most_freq_class = yolo_class
    return most_freq_class

mod_path = Path(__file__).parent
with open(mod_path / 'bdd100k.yaml') as f:
    data = yaml.load(f, Loader=SafeLoader)


objDect_cls = {}
for name in data['object detection']:
    objDect_cls[data['object detection'][name]] = cv2.cvtColor(np.array([[[int(255/(math.sqrt(name))),255,255]]], np.uint8), cv2.COLOR_HSV2BGR).squeeze().tolist()

class OrientationSmoothingFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.orientation_history = []

    def update(self, yaw):
        self.orientation_history.append(yaw)
        if len(self.orientation_history) > self.window_size:
            self.orientation_history = self.orientation_history[-self.window_size:]
        return sum(self.orientation_history) / len(self.orientation_history)

class BasicReceiver:
    def __init__(self):
        topic_input = '/panorama_img'
        topic_detection2d = '/detection2d_left'
        topic_pc = '/obstacle_detector/cloud_clusters'
        topic_jsk_sub = '/obstacle_detector/jsk_bboxes'
        topic_jsk_pub = '/fused_detection'
        self.bridge = CvBridge()
        self.original_image = None
        self.BBoxes = None
        self.pc_points = []
        self.lidar_boxes = []
        self.subscriber_input = rospy.Subscriber(topic_input, Image, self.inputCallback)
        self.subscriber_detection2d = rospy.Subscriber(topic_detection2d, detect2d, self.detection2dCallback)
        self.subscriber_pc = rospy.Subscriber(topic_pc, PointCloud2, self.pcCallback)
        self.susbcriber_jsk = rospy.Subscriber(topic_jsk_sub, BoundingBoxArray, self.jskCallback)
        self.publisher_jsk = rospy.Publisher(topic_jsk_pub, MarkerArray, queue_size=1)
        rospy.on_shutdown(self.shutdown_callback)
        
    def inputCallback(self, msg):
        self.original_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.origin_stamp = msg.header.stamp
    def detection2dCallback(self, msg):
        if self.origin_stamp - msg.stamp < max_time:
            self.BBoxes = msg
        else:
            self.BBoxes = None
    def pcCallback(self, msg):        
        self.pc_points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)

    def jskCallback(self, msg):
        self.lidar_boxes = msg.boxes    

     
   


if __name__ == '__main__':
    rospy.init_node('image_plotter', anonymous=True)
    
    teste = BasicReceiver()
    params = sensor_params()
    orientation_smoother = OrientationSmoothingFilter(5)
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
   
    namespace = rospy.get_namespace()
    new_boxes = MarkerArray()
    boxes_class = {}
    boxes_orientation = {}
    pose_odom_prev_x = 0
    pose_odom_prev_y = 0
    Homography = np.array([[ 6.54919231e-01, -6.69753943e-02,  2.48957192e+02],
                                            [-2.78402893e-02,  9.83161219e-01, -1.47572196e+00],
                                            [-1.05676793e-03,  3.13490230e-04,  1.00000000e+00]])
    # Homography = np.array([[ 4.48590236e-01, -1.37643857e-01,  8.47104211e+02],
    #                                         [-6.89298447e-02, 9.09414937e-01,  1.60873693e+01],
    #                                         [-4.29944417e-04,  4.20886946e-05,  1.00000000e+00]])
    rate = rospy.Rate(2.5)
    window_name = "Left Camera"
    while not rospy.is_shutdown():
        time_a = time.time()
        if not(teste.original_image is None):
            image = teste.original_image
            image = copy.copy(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
     
            c1_yolo = []
            c2_yolo = []
            class_yolo = []
            # Draw bounding boxes
            if not(teste.BBoxes is None):
                bboxes = teste.BBoxes
                bbox_list = bboxes.BBoxList
                bbox_classes = bboxes.ClassList
                fontFace=cv2.FONT_HERSHEY_COMPLEX
                thickness= 1
                fontScale=0.5
                for idx, bbox in enumerate(bbox_list):
                    c1 = (bbox.Px1, bbox.Py1)
                    c2 = (bbox.Px2, bbox.Py2)
                    c1_yolo.append(c1)
                    c2_yolo.append(c2)
                    centroid = ((bbox.Px1+bbox.Px2)/2, (bbox.Py1+bbox.Py2)/2)
                    text = bbox_classes[idx].data
                    class_yolo.append(text)
               
                    color = objDect_cls[text]
                    image = cv2.rectangle(image, c1, c2, color = color, thickness=2, lineType=cv2.LINE_AA)
                    top_center = [int((c1[0]+c2[0])/2), c1[1]]
                    label_size = cv2.getTextSize(text=text, fontFace=fontFace, thickness=thickness, fontScale=fontScale)
                    org = (top_center[0]-int(label_size[0][0]/2),top_center[1]-int(label_size[0][1]/2))
                    image = cv2.putText(image, text=text, org=org, fontFace=fontFace, thickness=thickness, fontScale=fontScale, color=color)
                    # image = cv2.circle(image, (int(centroid[0]),int(centroid[1])),radius=10,color=(0,0,255), thickness=-1)

            # Draw point cloud points on image    
            cluster_pixels = []
            for point in teste.pc_points:              
                
                # lidar_points = np.array([[point[0]],[point[1]],[point[2]],[1]])
                # print('ponto:' , point)
                # print('lidar:', lidar_points)
         
                pose_lidar_cluster = PointStamped()        
                pose_lidar_cluster.point.x = point[0]
                pose_lidar_cluster.point.y = point[1]
                pose_lidar_cluster.point.z = point[2]

                try:                    
                    pixels_left = do_transform_point(pose_lidar_cluster, tf_buffer.lookup_transform('top_left_camera_optical', 'top_laser', rospy.Time(0)))
                    pixels_right = do_transform_point(pose_lidar_cluster, tf_buffer.lookup_transform('top_right_camera_optical', 'top_laser', rospy.Time(0)))
                except (tf2_ros.TransformException, rospy.ROSException) as e:
                    rospy.logwarn("Failed to transform point from source frame to target frame: {}".format(e))

                pixels_left = np.dot(np.hstack((params.K_camera_left_resized,np.array([[0],[0],[0]]))), np.array([[pixels_left.point.x], [pixels_left.point.y], [pixels_left.point.z], [1]]))
                pixels_left = pixels_left[:2] / pixels_left[2] 
                pixels_right = np.dot(np.hstack((params.K_camera_right_resized,np.array([[0],[0],[0]]))), np.array([[pixels_right.point.x], [pixels_right.point.y], [pixels_right.point.z], [1]]))
                pixels_right = pixels_right[:2] / pixels_right[2]
      
                    

                pixels_right_homography = np.dot(Homography, np.vstack((pixels_right, 1)))
         
                pixels_right_homography = pixels_right_homography[:2] / pixels_right_homography[2]
                # pixels_right_homography[0] = pixels_right_homography[0] * params.scale_x
                # pixels_right_homography[1] = pixels_right_homography[1] * params.scale_y
                if pixels_left[0] <= 400:
                    cluster_pixels.append(pixels_left)
                if pixels_right_homography[0] > 400:
                    cluster_pixels.append(pixels_right_homography)

                       
                        
                image = cv2.circle(image, (int(pixels_left[0]), int(pixels_left[1])),radius=1,color=(0,255,0) , thickness=-1)
                image = cv2.circle(image, (int(pixels_right_homography[0]), int(pixels_right_homography[1])),radius=1,color=(0,0,255) , thickness=-1)

            new_boxes.markers = []
            for box in teste.lidar_boxes:
         
                # if not any(box.label == marker.id for marker in new_boxes.markers):
                # print(box.label)
                pose_1 = PointStamped()
                pose_1.point.x = box.pose.position.x
                pose_1.point.y = box.pose.position.y 

                try:                    
                    pose_2 = do_transform_point(pose_1, tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time(0)))
                except (tf2_ros.TransformException, rospy.ROSException) as e:
                    rospy.logwarn("Failed to transform point from source frame to target frame: {}".format(e))


                if box.label not in boxes_orientation:
                    boxes_orientation[box.label] = {'x': pose_2.point.x, 'y': pose_2.point.y, 'moved_flag': False, 'previous_smoothed_yaw': None} 
     

                direction = np.array([pose_2.point.x, pose_2.point.y]) - np.array([boxes_orientation[box.label]['x'], boxes_orientation[box.label]['y']])
                distance = np.linalg.norm(np.array([pose_2.point.x, pose_2.point.y]) - np.array([boxes_orientation[box.label]['x'], boxes_orientation[box.label]['y']]))
                # print(distance)
                if distance > 0.15:     
                    yaw = atan2(direction[1], direction[0])
                    smoothed_yaw = orientation_smoother.update(yaw)
                    boxes_orientation[box.label]['moved_flag'] = True
             
                else:
                    if boxes_orientation[box.label]['moved_flag']:
                        smoothed_yaw = boxes_orientation[box.label]['previous_smoothed_yaw']                      
                    else:
                        yaw = atan2(direction[1], direction[0])
                        smoothed_yaw = yaw
                #     if not moved_flag:
                #         smoothed_yaw = -1
          
                boxes_orientation[box.label]['previous_smoothed_yaw'] = smoothed_yaw  
                # print("distance:", distance)        
                # print("flag:", moved_flag)
                # print("smoothed_yaw:", smoothed_yaw)
                    # print("yaw:", yaw)
                    # print("smoothed_yaw:", smoothed_yaw)
                
                new_box = Marker()
                new_box.header = box.header
                new_box.color.a = 0.8
                new_box.id = box.label
                new_box.type = Marker.TEXT_VIEW_FACING
                new_box.action = Marker.ADD
                new_box.color.r = 0
                new_box.color.g = 0
                new_box.color.b = 0
                new_box.pose.position.x = box.pose.position.x
                new_box.pose.position.y = box.pose.position.y
                new_box.pose.position.z = box.dimensions.z + 1
                new_box.pose.orientation.w = 1
                new_box.pose.orientation.z = smoothed_yaw
                new_box.scale = box.dimensions 
                   
                if len(boxes_class) == 0:        
                    new_box.text = "unknown"
                
                # print(boxes_orientation)
                # else:
                #     new_boxes.markers[idx].header = box.header
                #     new_boxes.markers[idx].pose = box.pose
                #     new_boxes.markers[idx].scale = box.dimensions
              
                boxes_orientation[box.label]['x'] = pose_2.point.x
                boxes_orientation[box.label]['y'] = pose_2.point.y
                

                corners = get_bounding_box_corners((box.pose.position.x,box.pose.position.y,box.pose.position.z),(box.dimensions.x,box.dimensions.y,box.dimensions.z))  
                
                pose_pixels_list = []
                for corner in corners:
                    pose_lidar = PointStamped()        
                    pose_lidar.point.x = corner[0]
                    pose_lidar.point.y = corner[1]
                    pose_lidar.point.z = corner[2]
            

                    try:                    
                        pose_camera_left = do_transform_point(pose_lidar, tf_buffer.lookup_transform('top_left_camera_optical', 'base_footprint', rospy.Time(0)))
                        pose_camera_right = do_transform_point(pose_lidar, tf_buffer.lookup_transform('top_right_camera_optical', 'base_footprint', rospy.Time(0)))
                    except (tf2_ros.TransformException, rospy.ROSException) as e:
                        rospy.logwarn("Failed to transform point from source frame to target frame: {}".format(e))

                   
                    pose_pixels_left = np.dot(np.hstack((params.K_camera_left_resized,np.array([[0],[0],[0]]))), np.array([[pose_camera_left.point.x], [pose_camera_left.point.y], [pose_camera_left.point.z], [1]]))
                    pose_pixels_left = pose_pixels_left[:2] / pose_pixels_left[2]
                    pose_pixels_right = np.dot(np.hstack((params.K_camera_right_resized,np.array([[0],[0],[0]]))), np.array([[pose_camera_right.point.x], [pose_camera_right.point.y], [pose_camera_right.point.z], [1]]))
                    pose_pixels_right = pose_pixels_right[:2] / pose_pixels_right[2]
                    pose_pixels_right_homography = np.dot(Homography, np.vstack((pose_pixels_right, 1)))
         
                    pose_pixels_right_homography = pose_pixels_right_homography[:2] / pose_pixels_right_homography[2]
             
                    if pose_pixels_left[0] <= 400:
                        pose_pixels_list.append(pose_pixels_left)
                    if pose_pixels_right_homography[0] > 400:
                        pose_pixels_list.append(pose_pixels_right_homography)
       
                    # image = cv2.circle(image, (int(pose_pixels_left[0]), int(pose_pixels_left[1])),radius=5,color=(255,0,0) , thickness=-1)
              
                u_values_list = [arr[0] for arr in pose_pixels_list]
                u_values_array = np.concatenate(u_values_list)
                u_min = int(np.min(u_values_array))
                u_max = int(np.max(u_values_array))
                v_values_list = [arr[1] for arr in pose_pixels_list]
                v_values_array = np.concatenate(v_values_list)
                v_min = int(np.min(v_values_array))
                v_max = int(np.max(v_values_array))
                
                u_values_list_1 = [arr[0] for arr in cluster_pixels]
                u_values_array_1 = np.concatenate(u_values_list_1)
                u_list = []
                for v in u_values_array_1:
                    if u_min <= v <= u_max:
                        u_list.append(v)
                if len(u_list) > 0:        
                    u_min_1 = int(np.min(u_list))
                    u_max_1 = int(np.max(u_list))

                v_values_list_1 = [arr[1] for arr in cluster_pixels]
                v_values_array_1 = np.concatenate(v_values_list_1)
                v_list = []
                for v in v_values_array_1:
                    if v_min <= v <= v_max:
                        v_list.append(v)
                if len(v_list) > 0: 
                    v_min_1 = int(np.min(v_list))
                    v_max_1 = int(np.max(v_list))
                


                image = cv2.rectangle(image, (u_min, v_min), (u_max, v_max), color=(0,100,255), thickness=2)
                image = cv2.rectangle(image, (u_min_1, v_min_1), (u_max_1, v_max_1), color=(0,255,0), thickness=2)
                IoU_l = 0
                
                for i,yolo in enumerate(c1_yolo):
                    
          
                    A_inter = max(yolo[0], u_min_1)            
                    B_inter = max(yolo[1], v_min_1)
                    C_inter = min(c2_yolo[i][0], u_max_1)
                    D_inter = min(c2_yolo[i][1], v_max_1)
                 
                    if C_inter < A_inter or D_inter <  B_inter:
                        inter_area = 0
                    else:
                        inter_area = ((C_inter-A_inter) * (D_inter-B_inter))
                    
                    reunion_1 = ((c2_yolo[i][0]-yolo[0]) * (c2_yolo[i][1]-yolo[1]))
                    reunion_2 = ((u_max_1-u_min_1) * (v_max_1-v_min_1))
                    reunion_area = reunion_1 + reunion_2 - inter_area
                    IoU = inter_area/reunion_area                  
                    
                    
                    if IoU > 0.4:
                        IoU_l = IoU
                     
                        if box.label not in boxes_class:
                            boxes_class[box.label] = {}
                        if class_yolo[i] not in boxes_class[box.label]:
                            boxes_class[box.label][class_yolo[i]] = 1
                        else:
                            boxes_class[box.label][class_yolo[i]] += 1


       
                # print(boxes_class)
                if box.label in boxes_class:
                    freq_dict = boxes_class[box.label]
                    new_box.text = most_frequent_yolo_class(freq_dict)
                    if new_box.text == "other":
                        new_box.text = "pedestrian"
                    

                  
             

                     
                new_boxes.markers.append(new_box)  
                fontFace=cv2.FONT_HERSHEY_COMPLEX                       
                org = (u_min,v_min-5)
    
                image = cv2.putText(image, text='IoU = ' + str(round(IoU_l,2)), org=org, color=[255,0,0], fontFace=fontFace, thickness=2, fontScale=0.8)
                    
            
       
        
            

            cv2.imshow(window_name, image)

            # counter += 1
            cv2.waitKey(1)
            teste.publisher_jsk.publish(new_boxes)
            rate.sleep()
        # time_b = time.time()
        # print(f"Tempo de receção: {time_b-time_a}")
    cv2.destroyAllWindows()
