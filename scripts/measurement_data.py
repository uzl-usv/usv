#!/usr/bin/env python
import rospy
import json
import sys
import random
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Range, NavSatFix, PointCloud2
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PolygonStamped, Point32
from geopy.distance import vincenty
from nav_msgs.msg import MapMetaData
from actionlib.simple_action_client import SimpleActionClient
from tf.transformations import quaternion_from_euler
import numpy as np
import math


class MeasurementsPub:
    def __init__(self, s):
        self.aera_pub = rospy.Publisher("/measurement_area", PolygonStamped, queue_size=10)
        self.origin = (-140, -40)
        self.goal = (self.origin[0], self.origin[1], 1)
        self.measure_dist = 5
        self.width = 25
        self.height = 10

        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        self.point_pub = rospy.Publisher('/measurements', PointCloud2, queue_size=10)
        self.points = []

        #get position
        #rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        #rospy.Subscriber("/tf", TransformStamped[], self.position_callback)
        #get range
        rospy.Subscriber("/range_depth", Range, self.depth_callback)
        #get map data
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

#        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.goal_callback)

        #No known position and map in the beginning
        self.fileName = s
        self.hasPosition = False
        self.position = -1
        self.hasMap = False
        self.info = MapMetaData()
        self.visited = None
        self.depth = None
        try:
            with open(self.fileName + "_depth.json", "r") as read_file:
                try:
                    self.depth = json.load(read_file)
                    self.hasMap = True
                except Exception as e:
                    print("got %s on json.load()" % e)
        except Exception as e:
            print(e)

        try:
            with open(self.fileName + "_visited.json", "r") as read_file:
                try:
                    self.visited = json.load(read_file)
                except Exception as e:
                    print("got %s on json.load()" % e)
        except Exception as e:
            print(e)

        rospy.on_shutdown(self.save)

    def grid_cell(self, wx, wy):
        mx = int((wx - self.info.origin.position.x) / self.info.resolution)
        my = int((wy - self.info.origin.position.y) / self.info.resolution)
        return self.grid[mx,my]

    def save(self):
        with open(self.fileName + "_depth.json", "w") as write_file:
            json.dump(self.depth, write_file)
        with open(self.fileName + "_visited.json", "w") as write_file:
            json.dump(self.visited, write_file)

    def next(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        x, y, d = self.goal
        heading = 0 if d == 1 else np.pi

        qx, qy, qz, qw = quaternion_from_euler(0, 0, heading)
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        print("grid", self.grid_cell(x, y))
    
        self.move_base.send_goal(goal, self.goal_callback)

    def goal_callback(self, status, result):
        print("goal reached", status, result)

        x,y,w = self.goal

        self.points.append([x,y,-random.randint(1,10)])
        header = Header(frame_id = "map", stamp = rospy.Time.now())
        msg = point_cloud2.create_cloud_xyz32(header, self.points)
        self.point_pub.publish(msg)

        if (w == 1 and x+self.measure_dist > self.origin[0]+self.width) or (w == -1 and x-self.measure_dist < self.origin[0]):
            y += self.measure_dist
            w *= -1
        else:
            x += self.measure_dist*w
        self.goal = (x,y,w)
        if y <= self.origin[1]+self.height:
            self.next()

    #get current position in map
    def gps_callback(self, data):
        #calculate position in grid from position of the boat
        if self.hasMap and self.info.resolution != 0:
            #gps coordinates of the bottom left corner of the map
            orig_lat = -30.0486235837
            orig_lon = -51.2365778088
            #distance in metres between current position and bottom left corner of the map
            if not(data.latitude < orig_lat or data.longitude < orig_lon):
                dist_lat = vincenty((orig_lat,orig_lon), (data.latitude,orig_lon)).m
                dist_lon = vincenty((orig_lat,orig_lon), (orig_lat,data.longitude)).m
                '''
                sig_lat = -1
                sig_lon = -1
                if data.latitude < orig_lat:
                    sig_lat = 1
                if data.longitude < orig_lon:
                    sig_lon = 1
                '''
                x = dist_lat
                y = dist_lon
                print("\nDistanz in x-Richtung: ", dist_lat)
                print("\nDistanz in y-Richtung: ", dist_lon)
                #calculate position in grid
                print("\nPosition:")
                self.position = int(np.floor(x/self.info.resolution)+(np.floor(y/self.info.resolution))*self.info.width)
                print(self.position)
                self.hasPosition = True
            else:
                self.position = -1
    
    '''
    def position_callback(self, data):
        #calculate position in grid from position of the boat
        if self.hasMap:
            #real world coordinates of the bottom left corner of the map
            orig_x = self.info.origin.position.x
            orig_y = self.info.origin.position.y
            #distance in metres between current position and bottom left corner of the map
            dist_x = orig_x-#TODO: get transform from tf
            dist_y = orig_y-#TODO: get transform from tf
            #calculate position in grid
            self.position = np.floor(x/self.info.resolution)+(np.floor(y/self.info.resolution))*self.info.width
            self.hasPosition = True
    '''
    #save depth at current position
    def depth_callback(self,data):
        self.depth = data.range
        return

        if self.hasMap and self.hasPosition and self.position >= 0:
            self.depth[self.position] = data.range
            self.visited[self.position] = 1

    #get map metadata
    def map_callback(self, data):
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

        self.grid = np.array(data.data).reshape((data.info.width, data.info.height))
        self.aera_pub.publish(msg)

        self.info = data.info
        self.next()

        width = self.info.width
        height = self.info.height
        if not self.hasMap:
            self.visited = [0] * (height*width)
            for i in range(height*width):
                if data.data[i] > 50:
                    self.visited[i] = 1
                else:
                    self.visited[i] = -1
            self.depth = [0] * (height*width)
            self.hasMap = True
        
    
def listener():
    rospy.init_node('measurements_listener', anonymous=True)
    MeasurementsPub(sys.argv[1])
    rospy.spin()

if __name__ == '__main__':
    listener()
