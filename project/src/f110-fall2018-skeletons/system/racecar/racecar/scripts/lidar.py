#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Duration

def plot_x(msg):
    global counter
    if (counter % 100 == 0):
        plt.clf()
        plt.plot(msg.ranges)
        plt.draw()
        plt.pause(0.01)
    counter += 1

if __name__ == '__main__':
    counter = 0
    rospy.init_node("plotter")
    rospy.Subscriber("/scan", LaserScan, plot_x)
    plt.ion()
    rospy.spin()
