#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import argparse
import random
import rospy
import timeit
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.srv import GetModelProperties, GetModelState
from gazebo_msgs.msg import ModelState

from moveit_commander import PlanningSceneInterface
from moveit_commander import PlanningScene

from math import pi
from visualization_msgs.msg import Marker
from queue import Queue

np.set_printoptions(suppress=True)
np.set_printoptions(linewidth=np.inf)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur_control", anonymous=True)

group_name_m = "arm"
move_group_m = moveit_commander.MoveGroupCommander(group_name_m)
group_name_g = "gripper"
move_group_g = moveit_commander.MoveGroupCommander(group_name_g)
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)

scene = moveit_commander.PlanningSceneInterface()

def add_static_obstacle():
    object1_name = "ur_base_modify"
    object1_pose = geometry_msgs.msg.PoseStamped()
    object1_pose.header.frame_id = "world"
    object1_pose.pose.orientation.w = 1.0
    object1_pose.pose.position.x = 0 
    object1_pose.pose.position.y = 0 
    object1_pose.pose.position.z = -0.02
    
    object2_name = "test_box"
    object2_pose = geometry_msgs.msg.PoseStamped()
    object2_pose.header.frame_id = "world"
    object2_pose.pose.orientation.w = 1.0
    object2_pose.pose.position.x = 0
    object2_pose.pose.position.y = 0.5
    object2_pose.pose.position.z = -0.44
    
    scene.add_box(object1_name, object1_pose, size=(0.5, 0.5, 0.02))
    scene.add_box(object2_name, object2_pose, size=(0.5, 0.45, 0.72))
    
def subscriber():
    global result
    point_sub = rospy.Subscriber('scaling_factor', Float32, callback, queue_size = 3)
    rospy.on_shutdown(shutdown_hock)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pnp_loop()
        rate.sleep()

def callback(data):
    global num
    print(data.data)
    num = num-1
    scaling_factor = data.data
    scaling_factor = float(scaling_factor)
    move_group_m.set_max_velocity_scaling_factor(scaling_factor)
    move_group_m.stop()
    
def shutdown_hock():
    rospy.loginfo("Shutting down the node")
    rospy.signal_shutdown("Ctrl + C received")

def waypoint_1():
    global pose_goal, plan
    global joint_goal_m, current_pose
    joint_goal_m = move_group_m.get_current_joint_values()
    joint_goal_m = [math.radians(-58), math.radians(-137), math.radians(-27), math.radians(-107), math.radians(91), math.radians(32)]
    move_group_m.go(joint_goal_m, wait=True)
    move_group_m.stop()
    
def waypoint_2():
    global joint_goal_m, current_pose
    joint_goal_m = move_group_m.get_current_joint_values()
    joint_goal_m = [math.radians(-58), math.radians(-207), math.radians(68), math.radians(-132), math.radians(91), math.radians(32)]
    move_group_m.go(joint_goal_m, wait=True)
    move_group_m.stop()
    
def waypoint_3():
    global joint_goal_m, current_pose
    joint_goal_m = move_group_m.get_current_joint_values()
    joint_goal_m = [math.radians(-95), math.radians(-166), math.radians(33), math.radians(-137), math.radians(90), math.radians(-5)]
    move_group_m.go(joint_goal_m, wait=True)
    move_group_m.stop()

def waypoint_4():
    global joint_goal_m, current_pose
    joint_goal_m = move_group_m.get_current_joint_values()
    joint_goal_m = [math.radians(-95), math.radians(-207), math.radians(68), math.radians(-133), math.radians(90), math.radians(-5)]
    move_group_m.go(joint_goal_m, wait=True)
    move_group_m.stop()
    
def move_init():
    global joint_goal_m, current_pose
    joint_goal_m = move_group_m.get_current_joint_values()
    joint_goal_m = [math.radians(0), math.radians(-90), math.radians(0), math.radians(-90), math.radians(0), math.radians(0)]
    move_group_m.go(joint_goal_m, wait=True)
    move_group_m.stop()
    
def gripper_close():
    joint_goal = [0.285, -0.285, 0.285, 0.285, -0.285, 0.285]
    move_group_g.go(joint_goal, wait=True)
    move_group_g.stop()

def gripper_open():
    joint_goal = [0, 0, 0, 0, 0, 0]
    move_group_g.go(joint_goal, wait=True)
    move_group_g.stop()
    
scaling_factor = 1.0
num = 0
def pnp_loop():
    global num
    num += 1
    if num == 1:
        waypoint_1()
        print(f"current_num: {num}")
    elif num == 2:
        waypoint_2()
        print(f"current_num: {num}")
    elif num == 3:
        gripper_close()
        print(f"current_num: {num}")
        rospy.sleep(1)
    elif num == 4:
        waypoint_1()
        print(f"current_num: {num}")


    elif num == 5:
        waypoint_3()
        print(f"current_num: {num}")
    elif num == 6:
        waypoint_4()
        print(f"current_num: {num}")
    elif num == 7:
        gripper_open()
        print(f"current_num: {num}")
        rospy.sleep(1)
    elif num == 8:
        waypoint_3()
        print(f"current_num: {num}")


    elif num == 9:
        waypoint_3()
        print(f"current_num: {num}")
    elif num == 10:
        waypoint_4()
        print(f"current_num: {num}")
    elif num == 11:
        gripper_close() 
        print(f"current_num: {num}")
        rospy.sleep(1)
    elif num == 12:
        waypoint_3()
        print(f"current_num: {num}")
       

    elif num == 13:
        waypoint_1()
        print(f"current_num: {num}")
    elif num == 14:
        waypoint_2()
        print(f"current_num: {num}")
    elif num == 15:
        gripper_open()
        print(f"current_num: {num}")
        rospy.sleep(1)
    elif num == 16:
        waypoint_1()
        print("waypoint_1()")
       
    elif num == 17:
        num = 0

if __name__ == '__main__':
    try:
        add_static_obstacle()
        move_init()
        gripper_open()
        subscriber()
    except rospy.ROSInterruptException:
        pass

