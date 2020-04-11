#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

def callback(msg):
    x = []
    for i in range(0, 360):
        x.append(i)
        # print("ANGLE [{}] : {}".format(i, msg.ranges[i]))

    plt.title("Captation")
    plt.plot(x, msg.ranges)
    plt.xlabel('Degres')
    plt.ylabel('Distance')
    plt.savefig('LIDAR.PNG')

def listener():

    rospy.init_node('listenerLidar', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
