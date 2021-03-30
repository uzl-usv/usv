#!/usr/bin/env python
import rospy
import json
import sys
import random
from std_msgs.msg import Header
from boot.msg import Safety
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Range, NavSatFix, PointCloud2
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from geopy.distance import vincenty
from nav_msgs.msg import MapMetaData
from std_srvs.srv import Empty, EmptyResponse
from actionlib.simple_action_client import SimpleActionClient
from tf.transformations import quaternion_from_euler
import numpy as np
import math
from datetime import datetime


class MeasurementsPub:
    def __init__(self):
        self.measure_dist = rospy.get_param("usv/measurements/dist", 5)
        self.width = rospy.get_param("usv/measurements/width", 15)
        self.height = rospy.get_param("usv/measurements/height", 15)

        self.origin_lat = rospy.get_param("usv/origin/lat", -30.048638)
        self.origin_lon = rospy.get_param("usv/origin/lon", -51.23669)
        self.start_lat = rospy.get_param("usv/measurements/start/lat", -30.047311)
        self.start_lon = rospy.get_param("usv/measurements/start/lon", -51.234663)
        self.home_lat = rospy.get_param("usv/home/lat", -30.047358)
        self.home_lon = rospy.get_param("usv/home/lon", -51.233064)

        self.file_name = rospy.get_param("usv/measurements/file_name")

        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()

        self.area_pub = rospy.Publisher("/measurement_area", PolygonStamped, queue_size=10)
        self.point_pub = rospy.Publisher('/measurements', PointCloud2, queue_size=10)

        self.measurements = []
        self.points = []
        self.returning_home = False

        rospy.Subscriber("/range_depth", Range, self.depth_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/diffboat/safety", Safety, self.safety_callback)

        rospy.Service("next_goal", Empty, self.service_next_goal)

        self.has_map = False
        self.info = MapMetaData()

        self.load()

        rospy.on_shutdown(self.save)

    def grid_cell(self, wx, wy):
        mx = int((wx - self.info.origin.position.x) / self.info.resolution)
        my = int((wy - self.info.origin.position.y) / self.info.resolution)

        return self.grid[mx, my]

    def load(self):
        try:
            with open(self.file_name, "r") as read_file:
                json_data = json.load(read_file)
                self.measurements = json_data["measurements"]
                self.goal = json_data["goal"]
        except Exception as e:
            print(e)

    def save(self):
        print("saving to", self.file_name)
        with open(self.file_name, "w") as write_file:
            json_data = {
                "measurements" : self.measurements,
                "goal": self.goal
            }
            json.dump(json_data, write_file)

    def send_goal(self):
        x, y, d = self.goal

        if self.returning_home:
            return
        if y > self.origin[1]+self.height:
            self.return_home()
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        heading = 0 if d == 1 else np.pi

        qx, qy, qz, qw = quaternion_from_euler(0, 0, heading)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        self.publish_area()
#        self.move_base.cancel_all_goals()
        self.move_base.send_goal(goal, self.goal_callback)

    def next_goal(self, goal):
        x,y,w = goal
        if (w == 1 and x+self.measure_dist > self.origin[0]+self.width) or (w == -1 and x-self.measure_dist < self.origin[0]):
            y += self.measure_dist
            w *= -1
        else:
            x += self.measure_dist*w

        if self.grid_cell(x, y) > 0:
            return self.next_goal((x, y, w))
        else:
            return (x, y, w)

    def goal_callback(self, status, result):
        rospy.loginfo("Goal callback %s %s", status, result)

        x, y, w = self.goal

        if status == GoalStatus.SUCCEEDED:
            self.measurements.append({
                  "depth": self.depth,
                  "timestamp": datetime.now().replace(microsecond=0).isoformat(),
                  "x": x,
                  "y": y
                })

            self.points.append([x, y, self.depth])
            header = Header(frame_id = "map", stamp = rospy.Time.now())
            msg = point_cloud2.create_cloud_xyz32(header, self.points)
            self.point_pub.publish(msg)

            self.goal = self.next_goal(self.goal)
            
            rospy.loginfo("Next goal %s", self.goal)

            self.send_goal()

    def service_next_goal(self, req):
        rospy.logwarn("Skipping goal")
        self.goal = self.next_goal(self.goal)
        rospy.loginfo("Next goal %s", self.goal)
        self.send_goal()
        return EmptyResponse()

    def return_home(self):
        rospy.loginfo("Returning home")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        x, y = self.gps_to_cell(self.home_lat, self.home_lon)
        print("x: ",x, "y: ", y)
        heading = 0

        qx, qy, qz, qw = quaternion_from_euler(0, 0, heading)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        self.move_base.send_goal(goal)
        self.returning_home = True
        self.move_base.wait_for_result()

    def safety_callback(self, data):
        if data.battery <= 20: 
            rospy.logerr("Battery low")
            self.return_home()
        if data.water == 1:
            rospy.logerr("Water in boat")
            self.return_home()

    #calculates cell from gps data
    def gps_to_cell(self, lat, lon):
        #calculate position in map frame
        if self.has_map == True and self.info.resolution != 0:
            #distance in metres between current position and bottom left corner of the map
            if not(lat < self.origin_lat or lon < self.origin_lon):
                dist_lat = vincenty((self.origin_lat, self.origin_lon), (lat, self.origin_lon)).m
                dist_lon = vincenty((self.origin_lat, self.origin_lon), (self.origin_lat, lon)).m
                x = dist_lon
                y = dist_lat

                return (x+self.info.origin.position.x, y+self.info.origin.position.y)
            else:
                return -1

    #save depth at current position
    def depth_callback(self,data):
        self.depth = data.range

    def publish_area(self):
        msg = PolygonStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        origin_x, origin_y = self.origin
        msg.polygon.points = [Point32(x, y, 0) for (x, y) in [
            (origin_x, origin_y),
            (origin_x+self.width, origin_y),
            (origin_x+self.width, origin_y+self.height),
            (origin_x, origin_y+self.height)
            ]]
        self.area_pub.publish(msg)

    #get map metadata
    def map_callback(self, data):
        self.info = data.info

        width = self.info.width
        height = self.info.height

        self.grid = np.matrix(np.array(data.data).reshape((height,width))).transpose()

        self.has_map = True

        self.origin = self.gps_to_cell(self.start_lat, self.start_lon)
        print(self.origin)
        if not self.measurements:
            self.goal = (self.origin[0], self.origin[1], 1)

        self.send_goal()
    
def listener():
    rospy.init_node('measurements_pub', anonymous=True, disable_signals=True)
    MeasurementsPub()
    rospy.spin()

if __name__ == '__main__':
    listener()
