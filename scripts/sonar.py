#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs import point_cloud2
import numpy as np


class Sonar():
    def __init__(self):
        rospy.Subscriber("/range_front", Range, self.callback)

        self.pub = rospy.Publisher('/range_front/filtered', Range, queue_size=10)

    def callback(self, data):
        if data.range == data.max_range:
            data.range = np.inf
#            data.range = data.max_range + 1
        self.pub.publish(data)
    
if __name__ == '__main__':
    rospy.init_node('sonar', anonymous=True)
    Sonar()
    rospy.spin()
