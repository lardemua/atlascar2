#!/usr/bin/python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from camera_perception.msg import detect2d
import copy
import numpy as np
import yaml
from yaml.loader import SafeLoader
from pathlib import Path
import math
import time


fps = 0.001
max_time = rospy.Duration.from_sec(1/fps)

counter = 0

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
        topic_input = '/top_left_camera/image_raw'
        topic_detection2d = 'detection2d_left'
        self.bridge = CvBridge()
        self.original_image = None
        self.BBoxes = None
        self.subscriber_input = rospy.Subscriber(topic_input, Image, self.inputCallback)
        self.subscriber_detection2d = rospy.Subscriber(topic_detection2d, detect2d, self.detection2dCallback)
    def inputCallback(self, msg):
        self.original_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.origin_stamp = msg.header.stamp
    def detection2dCallback(self, msg):
        if self.origin_stamp - msg.stamp < max_time:
            self.BBoxes = msg
        else:
            self.BBoxes = None


if __name__ == '__main__':
    teste = BasicReceiver()
    rospy.init_node('image_plotter', anonymous=True)
    namespace = rospy.get_namespace()

    window_name = "Right Camera"
    while not rospy.is_shutdown():
        time_a = time.time()
        if not(teste.original_image is None):
            image = teste.original_image
            image = copy.copy(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
     

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
                    text = bbox_classes[idx].data
                    color = objDect_cls[text]
                    image = cv2.rectangle(image, c1, c2, color = color, thickness=2, lineType=cv2.LINE_AA)
                    top_center = [int((c1[0]+c2[0])/2), c1[1]]
                    label_size = cv2.getTextSize(text=text, fontFace=fontFace, thickness=thickness, fontScale=fontScale)
                    org = (top_center[0]-int(label_size[0][0]/2),top_center[1]-int(label_size[0][1]/2))
                    image = cv2.putText(image, text=text, org=org, fontFace=fontFace, thickness=thickness, fontScale=fontScale, color=color)
         
            img = copy.deepcopy(image)
            # cv2.imwrite('/home/gribeiro/Tese/videos_tese/Output/'+str(counter)+'.png', img)
     
            cv2.imshow(window_name, image)

            counter += 1
            cv2.waitKey(1)
        # time_b = time.time()
        # print(f"Tempo de receção: {time_b-time_a}")
    cv2.destroyAllWindows()