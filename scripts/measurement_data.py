#!/usr/bin/env python
import rospy
import json
import sys
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geopy.distance import vincenty
from nav_msgs.msg import MapMetaData
import numpy as np
import math


class MeasurementsPub:
    def __init__(self, s):
        #get position
        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        #rospy.Subscriber("/tf", TransformStamped[], self.position_callback)
        #get range
        rospy.Subscriber("/range_front", Range, self.depth_callback)
        #get map data
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        #No known position and map in the beginning
        self.fileName = s
        self.hasPosition = False
        self.position = 0
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

    def save(self):
        with open(self.fileName + "_depth.json", "w") as write_file:
            json.dump(self.depth, write_file)
        with open(self.fileName + "_visited.json", "w") as write_file:
            json.dump(self.visited, write_file)

    def gps_callback(self, data):
        #calculate position in grid from position of the boat
        if self.hasMap and self.info.resolution != 0:
            #gps coordinates of the bottom left corner of the map
            orig_lat = -30.0486235837
            orig_lon = -51.2365778088
            #distance in metres between current position and bottom left corner of the map
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
            #print("\nX: %s", x)
            #print("\nY: %s", y)
            self.position = int(np.floor(x/self.info.resolution)+(np.floor(y/self.info.resolution))*self.info.width)
            print(self.position)
            self.hasPosition = True
    
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
    def depth_callback(self,data):
        if self.hasMap and self.hasPosition and self.position >= 0:
            self.depth[self.position] = data.range
            self.visited[self.position] = 1

    def map_callback(self, data):
        self.info = data.info
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
