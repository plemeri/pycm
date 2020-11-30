import os
import rospy

import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist
from jsk_recognition_msgs.msg import BoundingBox as BB, BoundingBoxArray as BBA

from pycm import CM
from pycm.config import *

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return {'x': qx, 'y': qy, 'z': qz, 'w': qw}

def parse_obj(obj_dict):
    ref = obj_dict.RefPnt
    near = obj_dict.NearPnt
    point = Point(x=ref.ds.x.data, y=ref.ds.y.data, z=ref.ds.z.data)
    # point = Point(x=near.ds.x.data + 4.35, y=near.ds.y.data, z=near.ds.z.data + 0.65)
    quat = Quaternion(**euler_to_quaternion(ref.r_zyx.z.data, ref.r_zyx.y.data, ref.r_zyx.x.data))

    do_dict = {
        'pose': Pose(position=point, orientation=quat),
        'dimensions': Vector3(0.1, 0.1, 0.1),
        'velocity': Twist(linear=Vector3(ref.dv.x.data, ref.dv.y.data, ref.dv.z.data))
    }
    
    return do_dict

def init_cm():
    cm = CM(IP_ADDRESS, PORT, key='Sensor.Object.RadarL')

    for nobj in range(20):
        cm.quantity.qdict.Sensor.Object.RadarL.Obj['T' + str(nobj).zfill(2)] = init_obj_dict(nobj)
        # cm.init_subscribe()

    cm.quantity._fill_msg_list(cm.quantity.qdict)
    cm.init_subscribe()
    cm.read()
    cm.read()
    return cm

def init_node():
    rospy.init_node('cm_obj_node')
    pub_jsk = rospy.Publisher('/obj_bbox', BBA, queue_size=10)
    return pub_jsk

if __name__ == '__main__':
    cm = init_cm()
    pub_jsk = init_node()

    while True:
        readable = cm.read()
        if readable is False:
            print('no')
            continue
        jsk_list = []
        for nobj in range(20):
            obj = cm.quantity.qdict.Sensor.Object.RadarL.Obj['T' + str(nobj).zfill(2)]
            if obj.obsv.data == 1:
                do_dict = parse_obj(obj)
                do_dict['id'] = nobj
                jsk_list.append(BB(header=Header(stamp=rospy.Time.now(), frame_id='RadarL'), pose=do_dict['pose'], dimensions=do_dict['dimensions'], label=nobj))


        pub_jsk.publish(BBA(header=Header(stamp=rospy.Time.now(), frame_id='RadarL'), boxes=jsk_list))
        # print('.', end='')