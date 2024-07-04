#!/usr/bin/env python3
import rospy
import math
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

resized_img1 = None
resized_img2 = None

def image_combine(resized_img1, resized_img2):
    print("image_combine")
    combined_image = cv2.hconcat([resized_img1, resized_img2])
    resized_center_x, resized_center_y = 960, 240
    distance_value = combined_image[resized_center_y, resized_center_x]
    print(f"Distance at center pixel: {distance_value} units")
    
    cv_image_normalized = cv2.normalize(combined_image, None, 0, 255, cv2.NORM_MINMAX)
    cv_image_normalized = cv_image_normalized.astype('uint8')
    jet_img = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
    cv2.imshow("Resized Multi Depth Image", jet_img)
    cv2.waitKey(1)
    return combined_image

def sync_callback(image_msg1, image_msg2):
    print("sync_callback")
    global resized_img1, resized_img2
    bridge = cv_bridge.CvBridge()
    
    try:
        depth_image1 = bridge.imgmsg_to_cv2(image_msg1, 'passthrough')
        depth_image2 = bridge.imgmsg_to_cv2(image_msg2, 'passthrough')
        rgb_fov = 69.4
        depth_fov = 85.2
        rgb_fov_rad = math.radians(rgb_fov / 2)
        depth_fov_rad = math.radians(depth_fov / 2)
        
        #Image1
        height, width = depth_image1.shape[:2]
        center_x, center_y = width // 2, height // 2
        crop_width = int(math.tan(rgb_fov_rad) / math.tan(depth_fov_rad) * width)
        crop_height = height
        start_x = center_x - crop_width // 2
        start_y = center_y - crop_height // 2
        end_x = start_x + crop_width
        end_y = start_y + crop_height
        cropped_img1 = depth_image1[start_y:end_y, start_x:end_x]
        resized_img1 = cv2.resize(cropped_img1, (640, 480), interpolation=cv2.INTER_AREA)
        
        #Image2
        height, width = depth_image2.shape[:2]
        center_x, center_y = width // 2, height // 2
        crop_width = int(math.tan(rgb_fov_rad) / math.tan(depth_fov_rad) * width)
        crop_height = height
        start_x = center_x - crop_width // 2
        start_y = center_y - crop_height // 2
        end_x = start_x + crop_width
        end_y = start_y + crop_height
        cropped_img2 = depth_image2[start_y:end_y, start_x:end_x]
        resized_img2 = cv2.resize(cropped_img2, (640, 480), interpolation=cv2.INTER_AREA)
        
    except Exception as e:
        rospy.logerr("Error processing depth images: %s", e)

def depth_listener():
    print("depth_listener")
    rospy.init_node('depth_image_processor')
    
    depth_sub1 = Subscriber('/camera1/depth/image_raw', Image)
    depth_sub2 = Subscriber('/camera2/depth/image_raw', Image)
    
    ats = ApproximateTimeSynchronizer([depth_sub1, depth_sub2], queue_size=2, slop=0.05)
    ats.registerCallback(sync_callback)
    
    image_pub = rospy.Publisher('/resize_multi_depth_image', Image, queue_size=10)
    bridge = cv_bridge.CvBridge()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        global resized_img1, resized_img2
        if resized_img1 is not None and resized_img2 is not None:
            combined_image = image_combine(resized_img1, resized_img2)
            ros_image_msg = bridge.cv2_to_imgmsg(combined_image, encoding="passthrough")
            image_pub.publish(ros_image_msg)
        rate.sleep()
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        depth_listener()
    except rospy.ROSInterruptException:
        pass

