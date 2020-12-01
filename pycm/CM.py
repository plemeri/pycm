import socket
import logging
from .Quantities import Quantities as Qty
import numpy as np
import time
import sys

class CM():
    def __init__(self, ip, port, key="", max_trial=1000000, log_level=logging.WARN):
        self.logger = logging.getLogger("pycm")
        self.logger.setLevel(log_level)

        self.ip = ip
        self.port = port
        self.max_trial = max_trial
        self.trial = 0

        self.socket = None
        self.quantity = Qty(key)
        self.connect()
        self.is_connected = self.init_subscribe()

        if self.is_connected is True:
            print("pycm init completed")
        else:
            print("pycm init failed")
        
    def count_trial(self):
        self.trial += 1
        c = ['/', '-', '\\'][self.trial % 3]
        return c

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.connect((self.ip, self.port))
        print("TCP socket connected")

    def init_subscribe(self):
        subscribable = True

        msg = ""
        for msg_ in self.quantity.msg_list:
            msg += msg_ + " "
        smsg = "QuantSubscribe {" + msg[:-1] + "}\r"

        if (self.socket == None):
            self.logger.error("Not connected")
            return

        self.socket.send(smsg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")[0]
        
        c = self.count_trial()

        if self.trial > self.max_trial:
            sys.exit(1)

        if rsp != "O0":
            subscribable = False
            print("Registering subscription for quantites", c, '\r', end='')
        else:
            print("Registerd to subscription for quantities")
        return subscribable

    def read(self):
        readable = True

        if self.is_connected is False:
            self.init_subscribe()
            readable = False
            return readable

        for msg in self.quantity.msg_list:
            try:
                self.socket.send(self.quantity.format_msg(msg).encode())
                str_rx = self.socket.recv(1000).decode()
            except:
                readable = False
                self.is_connected = False
                break
            rx_list = str_rx.split("\r\n\r\n")

            if (len(rx_list) != 2):
                self.logger.error("Wrong read")
                readable = False
                self.is_connected = False
                break
            else:
                rx = rx_list[0]
            
            if rx[0] == 'O':
                self.quantity.get(msg).data = float(rx[1:])
            else:
                self.logger.info(rx[1:])
            
        return readable
                
                

    def DVA_write(self, quantity, value, duration=-1, mode="Abs"):
        """ set the value of a variable using DVAWrite <Name> <Value> <Duration> <Mode> ...
        Parameters
        ----------
        quant : Quantity
            Quantity to set.
        value : Float
            New quantity value
        duration : Float
            Duration in milliseconds
        mode : string
            One of Abs, Off, Fac, AbsRamp, ...; default Abs(olute Value)
        """
        assert mode in ["Abs", "Off", "Fac", "FacOff", "AbsRamp", "OffRamp", "FacRamp", "FacOffRamp"]
        msg = "DVAWrite " + quantity + " " + \
            str(value)+" "+str(duration)+" "+mode+"\r"
        self.socket.send(msg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        self.logger.info("Write quantity " +
                         quantity + ": " + str(rsp))

    def DVA_release(self):
        """ Call this method when you are done using DVA """
        self.send("DVAReleaseQuants\r")

    def send(self, msg):
        """ send the giving message to CarMaker
        Paramters
        ---------
        msg : string
            a string contains the message ending with \ r
        """
        self.socket.send(msg.encode())
        return self.socket.recv(200)


class VDS:
    def __init__(self, ip="localhost", port=2210, log_level=logging.INFO):
        self.logger = logging.getLogger("pycm")
        self.logger.setLevel(log_level)
        self.ip = ip
        self.port = port
        self.socket = None
        self.cameras = []
        self.connected = False

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
        data = self.socket.recv(64)
        if(data.decode().find("*IPGMovie") != -1):
            print("IPG Movie is Connected...")
            self.connected = True

    def read(self):
        """
        Read the streamed images.

        Returns
        -------
        img : numpy array
            a numpy array representing the image

        """
        if not self.connected:
            self.logger.error("Connect first by calling .connect()")
            return
        # Get Image header and fill data
        data = self.socket.recv(64)
        splitdata = data.decode().split(" ")
        imgtype = splitdata[2]
        img_size = splitdata[4]
        data_len = int(splitdata[5])
        imag_h = int(img_size.split('x')[1])
        image_w = int(img_size.split('x')[0])
        lastdata = b''
        size = 0
        while(size != data_len):
            data = self.socket.recv(1024)
            try:
                strdata = data.decode()
                if strdata[0] == '*' and strdata[1] == 'V':
                    splitdata = data.decode().split(" ")
                    imgtype = splitdata[2]
                    img_size = splitdata[4]
                    data_len = int(splitdata[5])
                    imag_h = int(img_size.split('x')[1])
                    image_w = int(img_size.split('x')[0])
                    lastdata = b''
                    size = 0
                    continue
            except:
                pass
            lastdata += data
            size = np.frombuffer(lastdata, dtype=np.uint8).size
        datalist = np.frombuffer(lastdata, dtype=np.uint8)
        if(imgtype == "rgb"):
            img = datalist.reshape((imag_h, image_w, 3))
        elif(imgtype == "grey"):
            img = datalist.reshape((imag_h, image_w))
        else:
            self.logger.error("rgb and gray are supported for now")

        return img
