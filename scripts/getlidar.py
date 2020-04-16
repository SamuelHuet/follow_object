#!/usr/bin/env python
import random
from PIL import Image
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


class Sonar:

    def __init__(self, data):
        self.lidar_data = data
        self.img = None

    def generate_fake_data(self):
        self.lidar_data = [random.randint(0, 10) for i in range(360)]

    def create_template(self):
        self.img = Image.new("RGB", (500, 500), "#FFFFFF")
        self.img.paste((0, 0, 0), (245, 245, 255, 255))

    def show(self):
        self.img.show()

    def to_polar(self):
        lidar_data_x = []
        lidar_data_y = []
        for i in range(360):
            if np.isinf(self.lidar_data[i]):
                lidar_data_x.append(250)
                lidar_data_y.append(250)
            else:
                lidar_data_x.append((self.lidar_data[i]*50 * np.cos(i)) + 250)
                lidar_data_y.append((self.lidar_data[i]*50 * np.sin(i)) + 250)
        return lidar_data_x, lidar_data_y

    def draw_points(self, lidar_data_x, lidar_data_y):
        for i in range(360):
            self.img.paste((0, 0, 0), (lidar_data_x[i]-2, lidar_data_y[i]-2,
                                       lidar_data_x[i]+2, lidar_data_y[i]+2))

    def round_data(self, lidar_data_x_old, lidar_data_y_old):
        lidar_data_x = []
        lidar_data_y = []
        for i in range(360):
            lidar_data_x.append(int(round(lidar_data_x_old[i])))
            lidar_data_y.append(int(round(lidar_data_y_old[i])))
        return lidar_data_x, lidar_data_y

    def saveimg(self, ):
        self.img.save("LIDAR.PNG")

    def to_line(self):

        lidar_data_x = []
        lidar_data_y = []
        for i in range(360):
            lidar_data_x.append(i+1)
            if np.isinf(self.lidar_data[i]):
                lidar_data_y.append(0)
            else:
                lidar_data_y.append(self.lidar_data[i]*50)
        return lidar_data_x, lidar_data_y



def callback(msg):
    # print(len(msg.ranges))
    sonar = Sonar(msg.ranges)
    sonar.create_template()
    # sonar.generate_fake_data()
    x, y = sonar.to_line()
    # x, y = sonar.to_polar()
    x, y = sonar.round_data(x, y)
    sonar.draw_points(x, y)
    # toto.show()
    #sonar.show()
    sonar.saveimg()
    rospy.loginfo(rospy.get_caller_id() + "   Image created")

def listener():
    rospy.init_node('listenerLidar', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
