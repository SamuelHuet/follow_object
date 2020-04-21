#!/usr/bin/env python
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

import rospy
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionFeedback
from simple_navigation_goals import simple_navigation_goals
from geometry_msgs.msg import PoseWithCovarianceStamped

SIZE_FACTOR = 60
IMAGE_SIZE = 500
POINT_SIZE = 3

PATH = os.path.dirname(os.path.abspath(__file__))
TIMER_START = 0
SUBSCRIBED_LIDAR_TOPIC = 0
SUBSCRIBED_POSE_TOPIC = 0

INITIAL_X = 0
INITIAL_Y = 0
FRAME_ID = 0

class Sonar:

    def __init__(self):
        self.lidar_data = [0]*360
        self.img = None

    def new_data(self, data):
        self.lidar_data = data

    def create_template(self):
        self.img = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE), "#FFFFFF")

    def show(self):
        self.img.show()

    def to_cathesian(self, offset_theta=0, y_mirrored=True, x_mirrored=False):
        lidar_data_x_carthesian = [0]*360
        lidar_data_y_carthesian = [0]*360
        for i in range(360):
            if np.isinf(self.lidar_data[i]):
                lidar_data_x_carthesian[i] = 501
                lidar_data_y_carthesian[i] = 501
            else:
                if y_mirrored is True:
                    lidar_data_x_carthesian[i] = (-(self.lidar_data[i] *
                                                    SIZE_FACTOR *
                                                    np.cos(np.radians(i+offset_theta)))) + (IMAGE_SIZE/2)
                else:
                    lidar_data_x_carthesian[i] = (self.lidar_data[i] *
                                                  SIZE_FACTOR *
                                                  np.cos(np.radians(i+offset_theta))) + (IMAGE_SIZE/2)
                if x_mirrored is True:
                    lidar_data_y_carthesian[i] = (-(self.lidar_data[i] *
                                                  SIZE_FACTOR *
                                                  np.sin(np.radians(i+offset_theta)))) + (IMAGE_SIZE/2)
                else:
                    lidar_data_y_carthesian[i] = (self.lidar_data[i] *
                                                  SIZE_FACTOR *
                                                  np.sin(np.radians(i+offset_theta))) + (IMAGE_SIZE/2)

        return lidar_data_x_carthesian, lidar_data_y_carthesian

    def draw_points(self, lidar_data_x, lidar_data_y):
        border = (POINT_SIZE-1)/2
        for i in range(360):
            self.img.paste((0, 0, 0), (lidar_data_x[i]-border, lidar_data_y[i]-border,
                                       lidar_data_x[i]+border, lidar_data_y[i]+border))

    def round_data(self, lidar_data_x_old, lidar_data_y_old):
        lidar_data_x = []
        lidar_data_y = []
        for i in range(360):
            lidar_data_x.append(int(round(lidar_data_x_old[i])))
            lidar_data_y.append(int(round(lidar_data_y_old[i])))
        return lidar_data_x, lidar_data_y

    def saveimg(self):
        self.img.save(PATH + "/LIDAR.PNG")


    def opencv_clutch(self, picture, lidar):
        center = [0.0, 0.0]
        ref_robot_coor_objet = [0.0 ,0.0, 0.0]
        img = cv2.imread(PATH + "/" + lidar, 0)
        template = cv2.imread(PATH + "/" + picture, 0)
        method = eval('cv2.TM_CCOEFF')
        res = cv2.matchTemplate(img, template, method)
        left_top = cv2.minMaxLoc(res)[3]
        center[0] = left_top[0] + 30
        center[1] = left_top[1] + 28
        ref_robot_coor_objet[0] = (center[0] - float(IMAGE_SIZE/2)) / float(SIZE_FACTOR)
        ref_robot_coor_objet[1] = (center[1] - float(IMAGE_SIZE/2)) / float(SIZE_FACTOR)
        ref_robot_coor_objet[2] = (np.sqrt(((center[0]-(float(IMAGE_SIZE/2)))**2)+((center[1]-(float(IMAGE_SIZE/2)))**2))) / float(SIZE_FACTOR)

        x = ref_robot_coor_objet[0]
        y = ref_robot_coor_objet[1]
        R = ref_robot_coor_objet[2]
        R = R-0.7
        theta = 2*np.arctan(y/(x+np.sqrt(x**2 +y**2)))

        x = R*np.cos(theta)
        y = R*np.sin(theta)
        ref_robot_coor_objet[0] = x
        ref_robot_coor_objet[1] = y

        return ref_robot_coor_objet

    def move_to_point(self, coord, move):
        global TIMER_START
        global SUBSCRIBED_LIDAR_TOPIC
        global INITIAL_X
        global INITIAL_Y
        global FRAME_ID
        move.cancel_goal()
        if coord[2] > 1.0:
            move.go_to(coord[1], -coord[0], np.pi, frame="base_scan", blocking=False)
            TIMER_START = rospy.Time.now()
        else:
            if (rospy.Time.now()-TIMER_START) >= rospy.Duration.from_sec(3):
                SUBSCRIBED_LIDAR_TOPIC.unregister()
                move.cancel_all_goals()
                rospy.loginfo(rospy.get_caller_id() + "   Coming back to home")
                move.go_to(INITIAL_X, INITIAL_Y, 0, frame=FRAME_ID, blocking=True)
                rospy.signal_shutdown("Scenario terminated")
            else:
                move.cancel_all_goals()



def callback(msg, args):
    move = args
    rospy.loginfo(rospy.get_caller_id() + "   Got lidar data")
    sonar = Sonar()
    sonar.new_data(msg.ranges)
    sonar.create_template()
    x, y = sonar.to_cathesian(offset_theta=-90, y_mirrored=True, x_mirrored=True)
    x, y = sonar.round_data(x, y)
    sonar.draw_points(x, y)
    sonar.saveimg()
    rospy.loginfo(rospy.get_caller_id() + "   Image created")
    coord = sonar.opencv_clutch('cercle.png', 'LIDAR.PNG')
    rospy.loginfo(rospy.get_caller_id() + "   X = " + str(coord[0]) + "  Y = " + str(coord[1]))
    rospy.loginfo(rospy.get_caller_id() + "   Distance to target = " + str(coord[2]))
    sonar.move_to_point(coord, move)


def save_initial_pose(data):
    global SUBSCRIBED_POSE_TOPIC
    global INITIAL_X
    global INITIAL_Y
    global FRAME_ID
    FRAME_ID = data.header.frame_id
    INITIAL_X = data.pose.pose.position.x
    INITIAL_Y = data.pose.pose.position.y
    rospy.loginfo(rospy.get_caller_id() + "   Saving initial pose")
    SUBSCRIBED_POSE_TOPIC.unregister()



def listener():
    global TIMER_START
    global SUBSCRIBED_LIDAR_TOPIC
    global SUBSCRIBED_POSE_TOPIC

    rospy.init_node('listenerLidar', anonymous=True)
    move = simple_navigation_goals.SimpleNavigationGoals()
    rospy.on_shutdown(move._shutdown)
    TIMER_START = rospy.Time.now()

    SUBSCRIBED_POSE_TOPIC = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, save_initial_pose)
    SUBSCRIBED_LIDAR_TOPIC = rospy.Subscriber('/scan', LaserScan, callback, (move))
    rospy.spin()


if __name__ == '__main__':
    listener()
