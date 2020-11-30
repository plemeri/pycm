from pycm import CM
from pycm.config import *
import os

cm = CM(IP_ADDRESS, PORT, key='')

cm.read()
cm.read()
pub = ""
acc = 0
steer = 0
print(cm.quantity.msg_list)
# while True:
    # cm.read()
    # os.system("clear")
    # print(cm.quantity)

# cm.DVA_release()

