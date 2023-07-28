#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

bridge = CvBridge()
model = torch.hub.load("ultralytics/yolov5", "yolov5x", "/home/gaurav/yolov5/yolov5s.pt") 

def main():
    rospy.init_node('image')
    rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)

    rospy.spin() 

def image_callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("/head_camera/rgb/image_raw", img)
        cv2.waitKey(1)  
        person_Detection(img)
    except CvBridgeError as e:
        print(e)

def person_Detection(img):
    results = model(img)
    persons = results.pred[0] 
    persons = persons[persons[:, 5] == 0] 

    detection_img = img.copy()
    arrPersons = []
    for person in persons:
        xmin, ymin, xmax, ymax, confidence, class_id = person.tolist()
        arrPersons.append(person.tolist())
        cv2.rectangle(detection_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
    
    for arrPerson in arrPersons:
        center_x = int((arrPerson[0] + arrPerson[2]) / 2)
        center_y = int((arrPerson[1] + arrPerson[3]) / 2)

        cv2.imshow("Detection Results", detection_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
