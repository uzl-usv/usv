#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist

def callback(pub, data):
    odom = Odometry()
    pose = PoseWithCovariance()
    twist = TwistWithCovariance()
    #no shift in relation to boat position
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    #no rotation in relation to boat position
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    twist.twist = data
    odom.pose = pose
    odom.twist = twist
    odom.header.frame_id = "base_link"
    odom.child_frame_id = "base_link"
    pub.publish(odom)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odometry_pub', anonymous=True)

    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.Subscriber("velocity", Twist, lambda data: callback(pub, data))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

