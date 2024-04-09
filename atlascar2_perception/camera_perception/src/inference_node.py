#!/usr/bin/python3
from inference_class import Inference
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from inference_manager.msg import detect2d, BBox
from std_msgs.msg import String
import argparse
import sys
import cv2
import time

def bbox_iou(bbox1, bbox2):
    # Compute intersection coordinates
    intersection_x1 = max(bbox1.Px1, bbox2.Px1)
    intersection_y1 = max(bbox1.Py1, bbox2.Py1)
    intersection_x2 = min(bbox1.Px2, bbox2.Px2)
    intersection_y2 = min(bbox1.Py2, bbox2.Py2)

    # Compute intersection area
    intersection_area = max(0, intersection_x2 - intersection_x1) * max(0, intersection_y2 - intersection_y1)

    # Compute union area
    bbox1_area = (bbox1.Px2 - bbox1.Px1) * (bbox1.Py2 - bbox1.Py1)
    bbox2_area = (bbox2.Px2 - bbox2.Px1) * (bbox2.Py2 - bbox2.Py1)
    union_area = bbox1_area + bbox2_area - intersection_area

    # Compute IoU
    iou = intersection_area / union_area if union_area > 0 else 0

    return iou

class InferenceNode:
    def __init__(self, infer_function_name:str, model_path:str, model_loader:str, source:str):
        # ---------------------------------------------------
        #   Model and inference module
        # ---------------------------------------------------

        # The inference module must have a output_organizer and a transforms 
        # function and be in the inference_modules folder
        # infer_function_name = 'yolopv2_module'
        # model_path = '../../../models/yolopv2.pt'
        # self.inference = Inference(model_path, infer_function_name)

        # topic_input = '/cameras/frontcamera'

        topic_detection2d_left = 'detection2d_left'
        topic_detection2d_right = 'detection2d_right'
        subscriber_stream = rospy.Subscriber(source, Image, self.InferenceCallback)
        self.detection2d_pub_left = rospy.Publisher(topic_detection2d_left,detect2d, queue_size=10)
        self.detection2d_pub_right = rospy.Publisher(topic_detection2d_right,detect2d, queue_size=10)
        self.bridge = CvBridge()
        self.first_run = True
        self.inference_ready = False
        self.infer_function_name = infer_function_name
        self.model_loader = model_loader
        self.model_path = model_path
        self.source = source

    def InferenceCallback(self,msg):
        if self.first_run:
            self.first_run = False
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.inference = Inference(self.model_path, self.infer_function_name, self.model_loader, image)
            self.inference_ready = True
            
        elif self.inference_ready:
            # print("A fazer algo!")
            time_a = time.time()
            time_source = msg.header.stamp
            now = rospy.get_rostime()
            time_late = (now-time_source).to_sec()
            
            # if time_late < 0.005: #In the same machine
            if time_late < 0.5:# multiple machines
            # if time_late < 0.15:# multiple machines multiple models
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                image_stamp = msg.header.stamp
                image_frameId = msg.header.frame_id
                
                roi1 = image[:, 0:1001, :]
                roi2 = image[:, 500:image.shape[1], :]

                coords = []
                strings = []
                
                for idx, image_roi in enumerate([roi1, roi2]):
                    self.inference.load_image(image_roi)
                    start_time = rospy.get_rostime()
                    detections_2d, seg = self.inference.infer()
                    end_time = rospy.get_rostime()
            
                    if not(detections_2d is None):
                        (det2d_class_list, det2d_list) = detections_2d
                        detect2d_msg = detect2d()                        

                        bbox_roi2 = []
                        string_roi2 = []
                        for k, i in enumerate(det2d_list):
                            string = String()
                            string.data = det2d_class_list[k]
                            coord = BBox()
                            if idx == 1:
                                if i[0][0] > 0:
                                    coord.Px1 = i[0][0] + 500
                                    coord.Px2 = i[1][0] + 500
                                    coord.Py1 = i[0][1]                           
                                    coord.Py2 = i[1][1]
                                    bbox_roi2.append(coord) 
                                    string_roi2.append(string)                                   
                            else:
                                if i[1][0] < 1000:
                                    coord.Px1 = i[0][0]
                                    coord.Px2 = i[1][0]
                                    coord.Py1 = i[0][1]                           
                                    coord.Py2 = i[1][1]
                                    
                                    coords.append(coord)
                                    strings.append(string)
                                                     
                        for s, box in enumerate(bbox_roi2):
                            if all(abs(cord.Px1 - box.Px1) > 5 and abs(cord.Px2 - box.Px2) > 5 for cord in coords):
                                coords.append(box)
                                strings.append(string_roi2[s])
                      
                       



                detect2d_msg.BBoxList = coords
                detect2d_msg.ClassList = strings
                detect2d_msg.stamp = image_stamp
                detect2d_msg.frame_id = image_frameId
                detect2d_msg.start_stamp = start_time
                detect2d_msg.end_stamp = end_time
                # if self.source == '/top_right_camera/image_raw':
                #     self.detection2d_pub_right.publish(detect2d_msg)
                # else:
                self.detection2d_pub_left.publish(detect2d_msg)  
                      
                      
            # time_b = time.time()
            # print(f"Tempo geral: {time_b-time_a}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                            prog = 'inference_node',
                            description='This node receives an image as input and\
                            outputs the result of the inference')
    
    parser.add_argument('-fn', '--function_name', type=str, 
                        dest='infer_function', required=True, 
                        help='Name of the module with the output_organizer and \
                            transforms functions')
    
    parser.add_argument('-mp', '--model_path', type=str, 
                        dest='model_path', required=True, 
                        help='Model directory')
    
    parser.add_argument('-ml', '--model_loader', type=str, 
                        dest='model_loader', required=True, 
                        help='Name of the module that loads the model')
    
    parser.add_argument('-sr', '--source', type=str, 
                        dest='source', required=True, 
                        help='Topic with the image messages to process')
    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(parser.parse_args(args=arglist))
    
    rospy.init_node('inference_node', anonymous=False)
    teste = InferenceNode(infer_function_name = args['infer_function'], 
                          model_path = args['model_path'], 
                          model_loader = args['model_loader'],
                          source = args['source']
                          )
    rospy.spin()
