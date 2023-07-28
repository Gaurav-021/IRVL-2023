#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import torch
import numpy as np
import time
from math import radians
from nav_msgs.msg import Odometry

bridge = CvBridge()
model = torch.hub.load("ultralytics/yolov5", "yolov5x", "/home/gaurav/yolov5/yolov5s.pt")

pcl = None
pcl_received = False
robot_position = None


def main():
    rospy.init_node('image')
    rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)
    rospy.Subscriber('/head_camera/depth_registered/points', PointCloud2, pointcloud_callback)
    rospy.Subscriber('/odom', Odometry, get_odom)

    rospy.spin()


def image_callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("/head_camera/rgb/image_raw", img)
        cv2.waitKey(1)
        person_Detection(img)
    except CvBridgeError as e:
        print(e)


def get_odom(msg):
    global robot_position
    robot_position = msg.pose.pose.position


def person_Detection(img):
    global pcl, pcl_received
    if pcl_received:
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

                    print("3D Coordinates:", center_3d)
                    move_robot(center_3d[0])

        print()
        pcl_received = False


def pointcloud_callback(msg):
    global pcl, pcl_received
    pcl = msg
    pcl_received = True


def move_robot(coordinates):
    global robot_position

    if robot_position is not None:
        distance_to_human = coordinates[2]

        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        cmd_vel_msg = Twist()

        desired_distance = 2.0
        speed = 0.2

        if distance_to_human > desired_distance + 0.2:
            # Move closer to the human
            cmd_vel_msg.linear.x = speed
        elif distance_to_human < desired_distance - 0.2:
            # Move away from the human
            cmd_vel_msg.linear.x = -speed

        cmd_vel_pub.publish(cmd_vel_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
