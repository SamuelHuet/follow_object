#!/usr/bin/env python
import cv2
from matplotlib import pyplot as plt
import random
from PIL import Image
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import os


SIZE_FACTOR = 60
IMAGE_SIZE = 500
POINT_SIZE = 3

center = [0, 0]
ref_robot_coor_objet = [0 ,0]


class Sonar:

    def __init__(self, data):
        self.path = os.path.dirname(os.path.abspath(__file__))
        self.lidar_data = data
        self.img = None

    def generate_fake_data(self):
        self.lidar_data = [random.randint(0, 10) for i in range(360)]

    def create_template(self):
        self.img = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE), "#FFFFFF")
        # self.img.paste((0, 0, 0), (245, 245, 255, 255))

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

    def saveimg(self, ):
        self.img.save(self.path + "/LIDAR.PNG")

    def to_line(self):

        lidar_data_x = []
        lidar_data_y = []
        for i in range(360):
            lidar_data_x.append(i)
            if np.isinf(self.lidar_data[i]):
                lidar_data_y.append(0)
            else:
                lidar_data_y.append(self.lidar_data[i]*50)
        return lidar_data_x, lidar_data_y

    def pixel_to_metters(self, x, y):
        x = x/SIZE_FACTOR
        y = y/SIZE_FACTOR
        return x, y


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "   Got lidar data")
    sonar = Sonar(msg.ranges)
    sonar.create_template()
    x, y = sonar.to_cathesian(offset_theta=-90, y_mirrored=True, x_mirrored=True)
    x, y = sonar.round_data(x, y)
    sonar.draw_points(x, y)
    sonar.saveimg()
    rospy.loginfo(rospy.get_caller_id() + "   Image created")
    print(opencv_clutch('cercle.png', 'LIDAR.PNG'))


def listener():
    rospy.init_node('listenerLidar', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


def opencv_clutch(picture, lidar):
    img = cv2.imread(lidar,0)
    #img2 = img.copy()
    template = cv2.imread(picture,0)
    #w, h = template.shape[::-1]
    #img = img2.copy()
    method = eval('cv2.TM_CCOEFF')
    res = cv2.matchTemplate(img,template,method)
    left_top = cv2.minMaxLoc(res)[3]
    center[0] = left_top[0] + 30
    center[1] = left_top[1] + 28
    ref_robot_coor_objet [0] = (center [0] - 250) / 60
    ref_robot_coor_objet [1] = (center [1] - 250) / 60
    distance = (np.sqrt(((center[0]-250)**2)+((center[1]-250)**2))) / 60
    return ref_robot_coor_objet, distance






if __name__ == '__main__':
    listener()
