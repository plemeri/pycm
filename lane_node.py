from pycm import CM
from pycm.config import *
import os

cm = CM(IP_ADDRESS, PORT, key='Sensor.Line.Front')

cm.read()
cm.read()

while True:
    cm.read()
    nleft, nright = cm.quantity.get('Sensor.Line.Front.nLine_Left'), cm.quantity.get('Sensor.Line.Front.nLine_Left')
    nleft, nright = int(nleft.data), int(nright.data)

    lcolor = []
    lheight = []
    lid = []
    ltype = []
    lwidth = []

    for i in range(1, nleft + 1):
        lcolor.append(cm.quantity.get('Sensor.Line.Front.LLines.' + str(i) + '.ColorCode').data)
        lheight.append(cm.quantity.get('Sensor.Line.Front.LLines.' + str(i) + '.Height').data)
        lid.append(cm.quantity.get('Sensor.Line.Front.LLines.' + str(i) + '.Id').data)
        ltype.append(cm.quantity.get('Sensor.Line.Front.LLines.' + str(i) + '.Type').data)
        lwidth.append(cm.quantity.get('Sensor.Line.Front.LLines.' + str(i) + '.Width').data)

    print(lcolor, lheight, lid, ltype, lwidth)

cm.DVA_release()

