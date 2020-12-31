import os
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import time
import copy
import json


from pycm import CM, VDS
from pycm.config import *

IMG_DIR = './data/img'
OBJ_DIR = './data/obj_annotation'
TRF_DIR = './data/trf_annotation'
PREFIX = 'route1_'
SEQ = 0
SAVE_ALL = True
INTERVAL = 1

if SAVE_ALL is False:
    import keyboard

if os.path.isdir(IMG_DIR) is False:
    os.makedirs(IMG_DIR)

if os.path.isdir(OBJ_DIR) is False:
    os.makedirs(OBJ_DIR)

if os.path.isdir(TRF_DIR) is False:
    os.makedirs(TRF_DIR)    

camera_info = None
cam = None

def init_cm():
    cm = CM(IP_ADDRESS, PORT, key='Sensor.Camera')
    cm.init_subscribe()
    cm.read()
    cm.read()
    return cm

def callback(cm, img):
    global bbox_list
    global img_list
    global SEQ

    readable = cm.read()

    w, h = camera_info.width, camera_info.height
    img = img.data
    img = np.frombuffer(img, dtype=np.uint8)
    img = img.reshape((h, w, 3))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    if readable is False:
        print('no')
    
    cam = cm.quantity.qdict.Sensor.Camera

    boxes = []
    clses = []
    sizes = []
    faces = []
    for key in cam.keys():
        for i in range(50):
            obj = cam[key].Obj[str(i)]

            bl = np.array([-obj.MBR.BL_Y.data, -obj.MBR.BL_Z.data, obj.MBR.BL_X.data, 1])
            tr = np.array([-obj.MBR.TR_Y.data, -obj.MBR.TR_Z.data, obj.MBR.TR_X.data, 1])
            if (bl == 0).sum() > 0 or (tr == 0).sum() > 0:
                continue

            P = np.array(camera_info.P).reshape((3, 4))

            bl = np.matmul(P, bl)
            bl /= bl[-1]

            tr = np.matmul(P, tr)
            tr /= tr[-1]

            box = [min(tr[0], bl[0]), min(tr[1], bl[1]), max(tr[0], bl[0]), max(tr[1], bl[1])]
            ratio = (box[2] - box[0]) / (box[3] - box[1])
            cls = obj.Type.data
        
            boxes.append(box)
            if cls == 5:
                cls = cls * 10 + obj.LightState.data
            clses.append(cls)
            sizes.append(obj.nVisPixels.data)
            faces.append(obj.Facing.data)

        break

    if SEQ % INTERVAL == 0:
            cv2.imwrite(os.path.join(IMG_DIR, PREFIX + str(SEQ // INTERVAL).zfill(6) + '.png'), img)
        
    obj = {'bbox': [], 'cls': []}
    trf = {'bbox': [], 'cls': [], 'size': []}

    for box, cls, size, face in zip(boxes, clses, sizes, faces):
        if cls // 10 == 0:
            print(box)
            obj['bbox'].append(box)
            obj['cls'].append(int(cls))
            color = (255, 0, 0)

            img = cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, 2)
            img = cv2.putText(img, 'obj', (int(box[0]), int(box[1])), 0, 0.5, color, 2)
        else:
            light = img[int(box[1]):int(box[3]), int(box[0]):int(box[2]), :]

            b, g, r = light[:, :, 0], light[:, :, 1], light[:, :, 2]

            has_r = np.any((r > 150) & (g < 50) & (b < 50))
            has_g = np.any((r < 50) & (g > 150) & (b > 150))
            has_y = np.any((r > 150) & (g > 150) & (b < 50))

            is_light = np.any([has_r, has_g, has_y]) and (box[2] - box[0]) / (box[3] - box[1]) > 1.5

            if is_light:
                trf['bbox'].append(box)
                trf['cls'].append(int(cls % 10))
                trf['size'].append(size)
                color = (0, 0, 255)
            else:
                color = (100, 100, 100)

            img = cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, 2)
            img = cv2.putText(img, 'trflight', (int(box[0]), int(box[1])), 0, 0.5, color, 2)
    
    if SAVE_ALL is True or keyboard.is_pressed('ctrl'):
        if SEQ % INTERVAL == 0:
            if len(obj['bbox']) > 0:
                json.dump(obj, open(os.path.join(OBJ_DIR, PREFIX + str(SEQ // INTERVAL).zfill(6) + '.json'), 'w'))

            if len(trf['bbox']) > 0:
                if len(trf['bbox']) > 2:
                    del trf['bbox'][2:]
                    del trf['cls'][2:]
                    del trf['size']
                json.dump(trf, open(os.path.join(TRF_DIR, PREFIX + str(SEQ // INTERVAL).zfill(6) + '.json'), 'w'))
        
            cv2.imshow('ipg', img)
            cv2.waitKey(1)
            
        SEQ += 1

def callback_camera_info(cam_info):
    global camera_info
    camera_info = cam_info


if __name__ == '__main__':
    cm = init_cm()
    port = 2211
    rospy.init_node('camera_node')
    sub1 = rospy.Subscriber('/vds_node_localhost_' + str(port) + '/image_raw', Image, lambda img: callback(cm, img), tcp_nodelay=True)
    sub2 = rospy.Subscriber('/vds_node_localhost_' + str(port) + '/camera_info', CameraInfo, callback_camera_info)
    rospy.spin()