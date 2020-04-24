#!/usr/bin/env python
# Author: Feng Jin

import argparse
import os
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import rosbag
import rospy
import random
from rospy.numpy_msg import numpy_msg
from node1_radarinterface.msg import RadarScan
from visualization_msgs.msg import Marker
from colormap import marker_color_map

veryfirstframe  = True
sub_frame_idx   = 0
frame_num       = 0
color_idx       = 0
N               = 5
first_targetidx = 0
second_targetidx = 0
msg_list        = []

def publish_new_marker(source_msg):
    global frame_num, color_idx, sub_frame_idx, veryfirstframe, N, first_targetidx, second_targetidx, msg_list

    universal_id = 0
    # New frame comes
    if frame_num != source_msg.frame_num:
        frame_num                   = source_msg.frame_num

        # Fullfill the new marker
        marker                      = Marker()
        marker.header.frame_id      = "display_markers"
        marker.header.stamp         = rospy.get_rostime()
        marker.type                 = Marker.SPHERE
        # Otherwise, the old markers may be overwrite by the marker with the same id
        # marker.id                   = universal_id
        marker.lifetime             = rospy.Duration(0) # Forever
        marker.pose.orientation.x   = 0
        marker.pose.orientation.y   = 0
        marker.pose.orientation.z   = 0
        marker.pose.orientation.w   = 0
        marker.scale.x              = 0.1
        marker.scale.y              = 0.1
        marker.scale.z              = 0.1
        marker.color.a              = 1

        marker.id                   = universal_id
        universal_id                += 1
        marker.action               = marker.DELETEALL
        new_marker_pub.publish(marker)
        raw_input("Press Enter to continue...")

        scan_targetidx              = 360
        # universal_id                = 0
        for current_msg in msg_list:
            if current_msg.target_idx < 253:
                if scan_targetidx != current_msg.target_idx:
                    scan_targetidx          = current_msg.target_idx
                    # Publish first centroid
                    marker.id               = universal_id
                    universal_id            += 1
                    # Fill the centorid marker
                    # Transer to ground Cartesian coordiantes due to tilt angle and tripod height
                    tilt_angle              = -10.0 # degrees
                    height                  = 1.8   # meters
                    rotation_matrix         = np.array([[1.0, 0.0, 0.0],\
                                            [0.0, np.cos(np.deg2rad(tilt_angle)), -np.sin(np.deg2rad(tilt_angle))],\
                                            [0.0, np.sin(np.deg2rad(tilt_angle)), np.cos(np.deg2rad(tilt_angle))]])
                    results                 = np.matmul(rotation_matrix, np.array([current_msg.posX, current_msg.posY, current_msg.posZ]))
                    marker.pose.position.x  = results[0]
                    marker.pose.position.y  = results[1]
                    marker.pose.position.z  = results[2] + height
                    # Centorid color
                    marker.color.r          = 255
                    marker.color.g          = 255
                    marker.color.b          = 0
                    marker.action           = marker.ADD
                    new_marker_pub.publish(marker)

                marker.id               = universal_id
                universal_id            += 1
                # Publish the points
                # Transer to Cartesian coordiantes from Polar coordinates
                x = current_msg.range * np.cos(current_msg.elev) * np.sin(current_msg.angle)
                y = current_msg.range * np.cos(current_msg.elev) * np.cos(current_msg.angle)
                z = current_msg.range * np.sin(current_msg.elev)
                # Transer to ground Cartesian coordiantes due to tilt angle and tripod height
                tilt_angle              = -10.0 # degrees
                height                  = 1.8   # meters
                rotation_matrix         = np.array([[1.0, 0.0, 0.0],\
                                    [0.0, np.cos(np.deg2rad(tilt_angle)), -np.sin(np.deg2rad(tilt_angle))],\
                                    [0.0, np.sin(np.deg2rad(tilt_angle)), np.cos(np.deg2rad(tilt_angle))]])
                results                 = np.matmul(rotation_matrix, np.array([x, y, z]))
                marker.pose.position.x  = results[0]
                marker.pose.position.y  = results[1]
                marker.pose.position.z  = results[2] + height

                target_idx              = current_msg.target_idx
                marker.color.r = marker_color_map[target_idx][0]
                marker.color.g = marker_color_map[target_idx][1]
                marker.color.b = marker_color_map[target_idx][2]

                marker.action           = marker.ADD
                new_marker_pub.publish(marker)      
        msg_list                    = []
        raw_input("Press Enter to continue...")  
    else:
        msg_list.append(source_msg)
        
if __name__ == '__main__':
    global new_marker_pub

    rospy.init_node('display', anonymous=True)
    
    new_marker_pub  = rospy.Publisher('/display/display_markers', Marker, queue_size = 100)
    rospy.Subscriber('/node1_radarinterface/radar_scan', RadarScan, publish_new_marker)

    rospy.spin()
