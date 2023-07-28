#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import Image, PointCloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import torch
import numpy as np
import actionlib
from math import radians
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class Robot:
    def __init__(self):
        rospy.init_node('person_detection_robot')
        rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/odom', Odometry, self.get_odom)

        self.bridge = CvBridge()
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5x", "/home/gaurav/yolov5/yolov5s.pt")
        self.pcl = None
        self.pcl_received = False
        self.pose_arr = []
        self.coordinates_arr = []
        self.within_threshold = False

        self.center_3d = [[10, 10, 10]]

    def get_odom(self, msg):
        self.pose_arr = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def image_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # cv2.imshow("/head_camera/rgb/image_raw", self.img)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def pointcloud_callback(self, msg):
        self.pcl = msg
        self.pcl_received = True

    def person_detection(self):
        if self.pcl_received:
            detection_img = self.img.copy()
            results = self.model(detection_img)
            persons = results.pred[0]
            persons = persons[persons[:, 5] == 0]

            
            self.coordinates_arr = []  # Clear the coordinates array for new detections

            if len(persons) > 0:
                for person in persons:
                    xmin, ymin, xmax, ymax, confidence, class_id = person.tolist()
                    self.coordinates_arr.append(person.tolist())
                    cv2.rectangle(detection_img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)

                for coordinates in self.coordinates_arr:
                    center_x = int((coordinates[0] + coordinates[2]) / 2)
                    center_y = int((coordinates[1] + coordinates[3]) / 2)

                    # cv2.imshow("Detection Results", detection_img)
                    # cv2.waitKey(1)

                    if self.pcl_received:
                        pcl_arr = pc2.read_points(self.pcl, field_names=("x", "y", "z"), skip_nans=True, uvs=[(center_x, center_y)])
                        pcl_points = np.array(list(pcl_arr))
                        if pcl_points.size > 0:
                            self.center_3d = pcl_points
                            print("3D Coordinates:", self.center_3d)
                            # self.move_robot(self.center_3d[0])

                print()
                self.pcl_received = False

    def move_robot(self):
        
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        print("move_robot")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.person_detection()
            coordinates = self.center_3d[0]
            print(self.center_3d[0])
            self.within_threshold = coordinates[2] < 3
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 12 * (not self.within_threshold) + coordinates[2] * self.within_threshold
            print(5 * (not self.within_threshold) + coordinates[2] * self.within_threshold)
            goal.target_pose.pose.position.y = 0 * (not self.within_threshold) + 0.5 * self.within_threshold
            print(0 * (not self.within_threshold) + 0.5 * self.within_threshold)

            goal_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # No rotation (keep current orientation)
            goal.target_pose.pose.orientation = goal_orientation
            
            # if coordinates[2] < 4:
            #     distance = coordinates[2] + 2
            #     goal_x = coordinates[0] - distance  # Adjust the goal position to be behind the person
            #     goal_y = coordinates[1]

            #     goal.target_pose.header.stamp = rospy.Time.now()
            #     goal.target_pose.pose.position.x = self.pose_arr[0] + distance  # goal_x
            #     goal.target_pose.pose.position.y = 0.5

            #     # Set the orientation to face towards the goal
            #     goal_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # No rotation (keep current orientation)
            #     goal.target_pose.pose.orientation = goal_orientation

            client.send_goal(goal)
            client.wait_for_result()
            time.sleep(5)
            rate.sleep()
            

if __name__ == '__main__':
    try:
        robot = Robot()
        robot.move_robot()
    except rospy.ROSInterruptException:
        pass
