import os
import pygame
from pycm import CM
from pycm.config import *

pygame.init()
pygame.joystick.init()
print("joystick count", pygame.joystick.get_count())

g29 = pygame.joystick.Joystick(0)
g29.init()

print('name:', g29.get_name())
print('axes:', g29.get_numaxes())

cm = CM(IP_ADDRESS, PORT)

cm.read()
cm.read()

pub = ""
acc = 0
steer = 0

prev_up = 0
prev_down = 0
prev_quit = 0

gear = [-9, -1, 0, 1]
gidx = 2

readable = False
while True:
    readable = cm.read()

    if readable is True:
        os.system('clear')
        print('velocity:', cm.quantity.qdict.Car.v.data * 3.6)
        print('Ax:', cm.quantity.qdict.UDP.Ax.data)
        print('WheelAng:', cm.quantity.qdict.UDP.WheelAng.data)
        print('GearNo:', cm.quantity.qdict.UDP.GearNo.data)
        print('VC_SwitchOn:', cm.quantity.qdict.UDP.VC_SwitchOn.data)

        pygame.event.pump()
        angle = g29.get_axis(0) * -6
        acc = (g29.get_axis(2) - 1) * -5
        brake = (g29.get_axis(3) - 1) * -5

        up = g29.get_button(4)
        down = g29.get_button(5)
        quit = g29.get_button(6)

        if prev_quit == 1 and quit == 0:
            break

        if prev_up == 1 and up == 0:
            gidx += 1
            gidx = min(gidx, len(gear) - 1)
            cm.DVA_write('UDP.GearNo', gear[gidx])

        elif prev_down == 1 and down == 0:
            gidx -= 1
            gidx = max(gidx, 0)
            cm.DVA_write('UDP.GearNo', gear[gidx])

        prev_up = up
        prev_down = down
        prev_quit = quit

        cm.DVA_write('UDP.Ax', acc - brake)
        cm.DVA_write('UDP.WheelAng', angle)

cm.DVA_release()


