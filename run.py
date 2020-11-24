from pycm import CM
from pycm.config import *
import time
import socket

cm = CM(IP_ADDRESS, PORT)

cm.read()
cm.read()


for i in range(100):
    cm.read()
    print(cm.quantity)
