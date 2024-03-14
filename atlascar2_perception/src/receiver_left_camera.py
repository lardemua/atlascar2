#!/usr/bin/python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from atlascar2_perception.msg import detect2d
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

mod_path = Path(__file__).parent
with open(mod_path / 'bdd100k.yaml') as f:
    data = yaml.load(f, Loader=SafeLoader)


objDect_cls = {}
for name in data['object detection']:
    objDect_cls[data['object detection'][name]] = cv2.cvtColor(np.array([[[int(255/(math.sqrt(name))),255,255]]], np.uint8), cv2.COLOR_HSV2BGR).squeeze().tolist()

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
        self.publisher_jsk = rospy.Publisher(topic_jsk_pub, MarkerArray, queue_size=10)
        
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
    
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)
    namespace = rospy.get_namespace()
    new_boxes = MarkerArray()
    
    
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
                    print(objDect_cls)
                    color = objDect_cls[text]
                    image = cv2.rectangle(image, c1, c2, color = color, thickness=2, lineType=cv2.LINE_AA)
                    top_center = [int((c1[0]+c2[0])/2), c1[1]]
                    label_size = cv2.getTextSize(text=text, fontFace=fontFace, thickness=thickness, fontScale=fontScale)
                    org = (top_center[0]-int(label_size[0][0]/2),top_center[1]-int(label_size[0][1]/2))
                    image = cv2.putText(image, text=text, org=org, fontFace=fontFace, thickness=thickness, fontScale=fontScale, color=color)
                    # image = cv2.circle(image, (int(centroid[0]),int(centroid[1])),radius=10,color=(0,0,255), thickness=-1)

            # Draw point cloud points on image    
           
            # for point in teste.pc_points:              
                
            #     lidar_points = np.array([[point[0]],[point[1]],[point[2]],[1]])
            #     # print('ponto:' , point)
            #     # print('lidar:', lidar_points)
            #     camera_points = np.dot(params.M_lidar_camera_inv, lidar_points)
                
            #     pixels = np.dot(params.P_camera_left, camera_points)
               
            #     pixels = pixels[:2] / pixels[2]              
                        
                # image = cv2.circle(image, (int(pixels[0]), int(pixels[1])),radius=1,color=(0,255,0) , thickness=-1)
            new_boxes.markers = []
            for box in teste.lidar_boxes:
                
                new_box = Marker()
                new_box.header = box.header
                new_box.pose = box.pose
                new_box.scale = box.dimensions
                new_box.id = box.label
                new_box.type = 1
                new_box.action = 0
                new_box.lifetime = rospy.Duration.from_sec(0.01)
                new_box.color.a = 0.8
                new_box.color.r = 0
                new_box.color.g = 0
                new_box.color.b = 1
                new_box.ns = ""

                corners = get_bounding_box_corners((box.pose.position.x,box.pose.position.y,box.pose.position.z),(box.dimensions.x,box.dimensions.y,box.dimensions.z))  
                
                pose_pixels_list = []
                for corner in corners:
                    pose_lidar = PointStamped()        
                    pose_lidar.point.x = corner[0]
                    pose_lidar.point.y = corner[1]
                    pose_lidar.point.z = corner[2]
            

                    try:                    
                        pose_camera = do_transform_point(pose_lidar, tf_buffer.lookup_transform('top_left_camera_optical', 'odom', rospy.Time(0)))
                    except (tf2_ros.TransformException, rospy.ROSException) as e:
                        rospy.logwarn("Failed to transform point from source frame to target frame: {}".format(e))

                    # intrinsic_parameters = params.P_camera_left[:, :3]
                    # homography_extended = np.vstack((params.homography, [0, 0, 0]))
                    # transformation_matrix = np.dot(intrinsic_parameters, homography_extended.T)


                    pose_pixels = np.dot(params.P_camera_left, np.array([[pose_camera.point.x], [pose_camera.point.y], [pose_camera.point.z], [1]]))
                    # pose_pixels = np.dot(params.homography, pose_pixels)
                    pose_pixels = pose_pixels[:2] / pose_pixels[2]
                    pose_pixels_list.append(pose_pixels)
                    image = cv2.circle(image, (int(pose_pixels[0]), int(pose_pixels[1])),radius=5,color=(255,0,0) , thickness=-1)
              
                u_values_list = [arr[0] for arr in pose_pixels_list]
                u_values_array = np.concatenate(u_values_list)
                u_min = int(np.min(u_values_array))
                u_max = int(np.max(u_values_array))
                v_values_list = [arr[1] for arr in pose_pixels_list]
                v_values_array = np.concatenate(v_values_list)
                v_min = int(np.min(v_values_array))
                v_max = int(np.max(v_values_array))
                
                
                image = cv2.rectangle(image, (u_min, v_min), (u_max, v_max), color=(0,100,255), thickness=2)
                IoU_l = 0
                
                for i,yolo in enumerate(c1_yolo):
                    
          
                    A_inter = max(yolo[0], u_min)            
                    B_inter = max(yolo[1], v_min)
                    C_inter = min(c2_yolo[i][0], u_max)
                    D_inter = min(c2_yolo[i][1], v_max)
                 
                    if C_inter < A_inter or D_inter <  B_inter:
                        inter_area = 0
                    else:
                        inter_area = ((C_inter-A_inter) * (D_inter-B_inter))
                    
                    reunion_1 = ((c2_yolo[i][0]-yolo[0]) * (c2_yolo[i][1]-yolo[1]))
                    reunion_2 = ((u_max-u_min) * (v_max-v_min))
                    reunion_area = reunion_1 + reunion_2 - inter_area
                    IoU = inter_area/reunion_area                  
                    
                    
                    if IoU > 0.3:
                        IoU_l = IoU
                        
                        new_box.ns = class_yolo[i]                        
                        new_boxes.markers.append(copy.deepcopy(new_box))    

                                          
                fontFace=cv2.FONT_HERSHEY_COMPLEX                       
                org = (u_min,v_min-5)
    
                image = cv2.putText(image, text='IoU = ' + str(round(IoU_l,2)), org=org, color=[255,0,0], fontFace=fontFace, thickness=2, fontScale=0.8)
                    
            
       
        
            teste.publisher_jsk.publish(new_boxes)

            cv2.imshow(window_name, image)

            counter += 1
            cv2.waitKey(1)
        # time_b = time.time()
        # print(f"Tempo de receção: {time_b-time_a}")
    cv2.destroyAllWindows()