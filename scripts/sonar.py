#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs import point_cloud2
import numpy as np


class Sonar():
    def __init__(self):
        self.make_filter("front")
        #self.make_filter("left1")
        #self.make_filter("left2")
        #self.make_filter("left3")
        #self.make_filter("left4")
        #self.make_filter("right1")
        #self.make_filter("right2")
        #self.make_filter("right3")
        #self.make_filter("right4")

    def make_filter(self, topic):
        pub = rospy.Publisher("/range_filtered/" + topic, Range, queue_size=10)

        def callback(data):
            if data.range == data.max_range:
                data.range = np.inf
            pub.publish(data)

        rospy.Subscriber("/range/" + topic, Range, callback)
    
if __name__ == '__main__':
    rospy.init_node('sonar', anonymous=True)
    Sonar()
    rospy.spin()
