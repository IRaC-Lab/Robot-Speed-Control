#!/usr/bin/env python3
import rospy
import cv2
import sys
import numpy as np
import datetime
import csv
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from matplotlib.path import Path
import matplotlib.pyplot as plt

cdr = 0
pdr = 0
    
class DepthImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.seg_coords = None
        self.sub = rospy.Subscriber(topic, Image, self.depthImageCallback)
        self.pub_1 = rospy.Publisher('scaling_factor', Float32, queue_size=3)
        self.pub_2 = rospy.Publisher('distance', Float32, queue_size=3)

    def depthImageCallback(self, data):
        global pdr, cdr, scaling_factor
        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            seg_depth_values = []
            if self.seg_coords and len(self.seg_coords) % 2 == 0:
                for s in range(0, len(self.seg_coords), 2):
                    depth_values = cv_depth_image[self.seg_coords[s+1], self.seg_coords[s]]
                    seg_depth_values.append(depth_values)

                distance = np.min(seg_depth_values)
                distance_meter = distance / 1000
                print("Shortest distance in the selected regions: {:.2f} meters".format(distance_meter))
                self.pub_2.publish(distance_meter)
                if distance_meter <= 3.15:
                    cdr = 1
                    scaling_factor = 0.017
                else:
                    cdr = 2
                    scaling_factor = 1.0
                if cdr != pdr:
                    print(scaling_factor)
                    self.pub_1.publish(scaling_factor)
                    pdr = cdr
            elif not self.seg_coords:
                distance_meter = None 
                self.pub_2.publish(distance_meter)
                print("No depth information available in the selected regions")
                cdr = 2
                scaling_factor = 1.0
                if cdr != pdr:
                    print(scaling_factor)
                    self.pub_1.publish(scaling_factor)
                    pdr = cdr
            else:
                pass
        
        except CvBridgeError as e:
            print(e)
            
    def set_seg_coords(self, seg_coords):
        self.seg_coords = seg_coords

class BoxCoordListener:
    def __init__(self, topic, depth_topic):
        self.topic = topic
        self.sub = rospy.Subscriber(topic, Int32MultiArray, self.boxCoordCallback)
        self.depth_image_listener = DepthImageListener(depth_topic)
        
    def boxCoordCallback(self, data):
        seg_coords = []
        if data.data:
            seg_coords = data.data
            self.depth_image_listener.set_seg_coords(seg_coords)
        else:
            self.depth_image_listener.set_seg_coords([])
            print("Received empty seg_coords data")

if __name__ == '__main__':
    rospy.init_node("depth_and_seg_coord_listener")
    topic = '/seg_coords'
    depth_topic = '/resize_multi_depth_image'
    listener = BoxCoordListener(topic, depth_topic)
    rospy.spin()

