import os
import rospy
from autoware_msgs.msg import ControlCommandStamped as ctrl
from pycm import CM
from pycm.config import *

readable = False

def callback(cm, msg):
    global readable
    readable = cm.read()

    if readable is True:
        os.system('clear')
        print('velocity:', cm.quantity.qdict.Car.v.data * 3.6)
        print('Ax:', cm.quantity.qdict.UDP.Ax.data)
        print('WheelAng:', cm.quantity.qdict.UDP.WheelAng.data)
        print('GearNo:', cm.quantity.qdict.UDP.GearNo.data)
        print('VC_SwitchOn:', cm.quantity.qdict.UDP.VC_SwitchOn.data)

        cm.DVA_write('UDP.Ax', msg.cmd.linear_acceleration)
        cm.DVA_write('UDP.WheelAng', msg.cmd.steering_angle)

if __name__ == '__main__':
    rospy.init_node('cmd_node')

    cm = CM(IP_ADDRESS, PORT)
    cm.read()
    cm.read()

    sub = rospy.Subscriber('/ctrl_cmd', ctrl, lambda msg: callback(cm, msg))
    rospy.spin()
    cm.DVA_release()
