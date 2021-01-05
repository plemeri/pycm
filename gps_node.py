import os
import numpy as np
import rospy

from tf import TransformBroadcaster as TB
from geodesy.utm import fromLatLong as proj
from geometry_msgs.msg import PoseStamped
from hellocm_msgs.msg import GPS_Out
import time

from pycm import CM, VDS
from pycm.config import *

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

def init_cm():
    cm = CM(IP_ADDRESS, PORT, key='Vhcl')
    cm.init_subscribe()
    cm.read()
    cm.read()
    return cm

def callback(msg):
    cm.read()
    vhcl = cam = cm.quantity.qdict.Vhcl

    quat = euler_to_quaternion(vhcl.Yaw.data, vhcl.Roll.data, vhcl.Pitch.data)
    pos = proj(msg.latitude, msg.longitude, msg.altitude)

    pos = np.array([pos.easting, pos.northing, pos.altitude])
    pos[:2] -= center[:2]

    print(quat)
    print(pos)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'

    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    pose.pose.position.z = pos[2]

    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    pub.publish(pose)
    tf_pub.sendTransform(tuple(pos), tuple(quat), rospy.Time.now(), 'Fr1A', 'map')


if __name__ == '__main__':
    cm = init_cm()
    port = 2211
    center = proj(37.58, 126.89, 0)
    center = np.array([center.easting, center.northing, center.altitude])
    rospy.init_node('camera_node')
    tf_pub = TB()

    sub = rospy.Subscriber('/gps_out', GPS_Out, callback)
    pub = rospy.Publisher('/localizer_pose', PoseStamped, queue_size=10)
    rospy.spin()