#!/usr/bin/env python
import cv2, os
import numpy as np
from PIL import Image

import rospy
from sensor_msgs.msg import LaserScan
from simple_navigation_goals import simple_navigation_goals
from geometry_msgs.msg import PoseWithCovarianceStamped

SIZE_FACTOR = 60
IMAGE_SIZE = 500
POINT_SIZE = 3
MIN_CONFIANCE = 0.3


class Follower:

    def __init__(self):
        self.lidar_data = [0]*360
        self.img = None
        self.TIMER_START = 0
        self.initial_x = 0
        self.initial_y = 0
        self.frame_id = 0
        self.SUBSCRIBED_LIDAR_TOPIC = 0
        self.SUBSCRIBED_POSE_TOPIC = 0
        self.PATH = os.path.dirname(os.path.abspath(__file__))
        self.go_to_x = 0
        self.go_to_y = 0
        self.listener()

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
        self.img.save(self.PATH + "/" + "LIDAR.PNG")

    def opencv_clutch(self, picture, lidar):
        confiance = 1

        center = [0.0, 0.0]
        ref_robot_coor_objet = [0.0, 0.0, 0.0]
        img = cv2.imread(self.PATH + "/" + lidar, 0)
        template = cv2.imread(self.PATH + "/" + picture, 0)
        method = eval('cv2.TM_CCOEFF_NORMED')
        res = cv2.matchTemplate(img, template, method)
        left_top = cv2.minMaxLoc(res)[3]
        confiance = cv2.minMaxLoc(res)[1]
        center[0] = left_top[0] + 30
        center[1] = left_top[1] + 28
        ref_robot_coor_objet[0] = (center[0] - float(IMAGE_SIZE/2)) / float(SIZE_FACTOR)
        ref_robot_coor_objet[1] = (center[1] - float(IMAGE_SIZE/2)) / float(SIZE_FACTOR)
        ref_robot_coor_objet[2] = (np.sqrt(((center[0]-(float(IMAGE_SIZE/2)))**2)+((center[1]-(float(IMAGE_SIZE/2)))**2))) / float(SIZE_FACTOR)

        x = ref_robot_coor_objet[0]
        y = ref_robot_coor_objet[1]
        R = ref_robot_coor_objet[2]
        R = R-0.7
        theta = 2*np.arctan(y/(x+np.sqrt(x**2 + y**2)))

        x = R*np.cos(theta)
        y = R*np.sin(theta)
        ref_robot_coor_objet[0] = x
        ref_robot_coor_objet[1] = y

        if confiance > MIN_CONFIANCE:
            self.go_to_x = x
            self.go_to_y = y

        return ref_robot_coor_objet

    def move_to_point(self, coord):
        self.move.cancel_goal()
        if coord[2] > 1.0:
            self.move.go_to(self.go_to_y, -self.go_to_x, np.pi, frame="base_scan", blocking=False)
            self.TIMER_START = rospy.Time.now()
        else:
            if (rospy.Time.now()-self.TIMER_START) >= rospy.Duration.from_sec(3):
                self.SUBSCRIBED_LIDAR_TOPIC.unregister()
                self.move.cancel_all_goals()
                rospy.loginfo(rospy.get_caller_id() + "   Coming back to home")
                self.move.go_to(self.initial_x, self.initial_y, 0, frame=self.frame_id, blocking=True)
                rospy.signal_shutdown("Scenario terminated")
            else:
                self.move.cancel_all_goals()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "   Got lidar data")
        self.new_data(data.ranges)
        self.create_template()
        x, y = self.to_cathesian(offset_theta=-90, y_mirrored=True, x_mirrored=True)
        x, y = self.round_data(x, y)
        self.draw_points(x, y)
        self.saveimg()
        rospy.loginfo(rospy.get_caller_id() + "   Image created")
        coord = self.opencv_clutch('cercle.png', 'LIDAR.PNG')
        rospy.loginfo(rospy.get_caller_id() + "   X = " + str(coord[0]) + "  Y = " + str(coord[1]))
        rospy.loginfo(rospy.get_caller_id() + "   Distance to target = " + str(coord[2]))
        self.move_to_point(coord)

    def save_initial_pose(self, data):
        self.frame_id = data.header.frame_id
        self.initial_x = data.pose.pose.position.x
        self.initial_y = data.pose.pose.position.y
        self.go_to_x = data.pose.pose.position.x
        self.go_to_y = data.pose.pose.position.y
        rospy.loginfo(rospy.get_caller_id() + "   Saving initial pose")
        self.SUBSCRIBED_POSE_TOPIC.unregister()

    def listener(self):
        rospy.init_node('listenerLidar', anonymous=True)
        self.move = simple_navigation_goals.SimpleNavigationGoals()
        rospy.on_shutdown(self.move._shutdown)
        self.TIMER_START = rospy.Time.now()
        self.SUBSCRIBED_POSE_TOPIC = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.save_initial_pose)
        self.SUBSCRIBED_LIDAR_TOPIC = rospy.Subscriber('/scan', LaserScan, self.callback)
        rospy.spin()


if __name__ == '__main__':
    Follower()
