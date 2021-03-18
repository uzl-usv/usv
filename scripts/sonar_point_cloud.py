#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs import point_cloud2
import numpy as np


class SonarPointCloud():
    def __init__(self):
        rospy.Subscriber("/range_front", Range, self.callback)

        self.pub = rospy.Publisher('/obstacles', PointCloud2, queue_size=10)

    def callback(self, data):
        header = Header(frame_id = data.header.frame_id,
                        stamp = rospy.Time.now())

        alpha = data.field_of_view / 2
        x = data.range
        y = x * np.tan(alpha)

        points = [[x, 0, 0], [x, -y, 0], [x, y, 0]]

        msg = point_cloud2.create_cloud_xyz32(header, points)
        
        self.pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('us_point_cloud', anonymous=True)
    SonarPointCloud()
    rospy.spin()
