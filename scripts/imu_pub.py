#!/usr/bin/env python
import rospy
from usv_tf.msg import Heading
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import quaternion_from_euler


class ImuPub():
    def __init__(self):
        rospy.Subscriber("/diffboat/heading", Heading, self.callback)

        self.pub = rospy.Publisher('imu_for_transform', Imu, queue_size=10)
        self.last_heading = 0
        self.last_time = 0

    def callback(self, data):
        msg = Imu()
        #0 degrees in the east
        #convert to radians
        heading = np.deg2rad(data.mag_heading + 90)

        time = data.header.stamp.to_sec()
        time_diff = time - self.last_time
        heading_diff = heading - self.last_heading

        x,y,z,w = quaternion_from_euler(0, 0, heading)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link" # TODO
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
        msg.angular_velocity.z = heading_diff / time_diff

        self.pub.publish(msg)
        self.last_heading = heading
        self.last_time = time
    
if __name__ == '__main__':
    rospy.init_node('imu_pub', anonymous=True)
    ImuPub()
    rospy.spin()
