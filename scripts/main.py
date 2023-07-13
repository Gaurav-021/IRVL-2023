#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import torch
import numpy as np
import time
import actionlib


bridge = CvBridge()
model = torch.hub.load("ultralytics/yolov5", "yolov5x", "/home/gaurav/yolov5/yolov5s.pt") 

x = None


def main():
    rospy.init_node('image')
    rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)
    rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, pointcloud_callback)
    rospy.spin() 


# /head_camera/depth_registered/image_raw
def image_callback(msg):
    
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("/head_camera/rgb/image_raw", img)
        cv2.waitKey(1)  
        person_Detection(img)
    except CvBridgeError as e:
        print(e)

def person_Detection(img):
    global x, pcl, pcl_received
    if pcl_received == True:
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

            if pcl_received:
                    pcl_arr = pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True, uvs=[(center_x, center_y)])
                    pcl_points = np.array(list(pcl_arr))
                    if pcl_points.size > 0:
                        center_3d = pcl_points

                        # Print the 3D coordinates
                        # print("3D Coordinates:", arrPerson, center_3d) # prints arrPerson (id)
                        print("3D Coordinates:", center_3d)
                        move_robot(center_3d[0])
                        # movebase_client(center_3d[0])

        print()
        pcl_received = False

def pointcloud_callback(msg):
    global pcl, pcl_received
    pcl = msg
    pcl_received = True

def move_robot(coordinates):
    distance = coordinates[2] + 2.0  
    goal_x = coordinates[0] + distance  
    goal_y = coordinates[1]  

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    # client.wait_for_result()

def task1(distance):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    move_cmd = Twist()
    move_cmd.linear.x = (distance - 1.5)/4
    pub.publish(move_cmd)
    now = time.time()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass