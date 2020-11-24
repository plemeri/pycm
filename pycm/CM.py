import socket
import logging
from .Quantities import Quantities as Qty
import numpy as np

class CM():
    def __init__(self, ip, port, key="", log_level=logging.WARN):
        self.logger = logging.getLogger("pycm")
        self.logger.setLevel(log_level)

        self.ip = ip
        self.port = port

        self.socket = None
        self.quantity = Qty(key)
        self.connect()
        self.init_subscribe()

        print("pycm init completed")

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.connect((self.ip, self.port))
        print("TCP socket connected")

    def init_subscribe(self):
        msg = ""
        for msg_ in self.quantity.msg_list:
            msg += msg_ + " "
        smsg = "QuantSubscribe {" + msg[:-1] + "}\r"
        if (self.socket == None):
            self.logger.error("Not connected")
            return

        self.socket.send(smsg.encode())
        rsp = self.socket.recv(200)
        rsp = rsp.decode().split("\r\n\r\n")
        print("Subscribe for quantites: " + str(rsp))
        # TODO Handle error

    def read(self):
        # By IPG recommendation, read one quantity at a time.
        for msg in self.quantity.msg_list:
            self.socket.send(self.quantity.format_msg(msg).encode())
            str_rx = self.socket.recv(1000).decode()
            rx_list = str_rx.split("\r\n\r\n")

            if (len(rx_list) != 2):
                self.logger.error("Wrong read")
                return
            else:
                rx = rx_list[0]
            
            if rx[0] == 'O':
                self.quantity.get(msg).data = float(rx[1:])
            elif rx[0] == 'E':
                self.logger.info(rx[1:])
                

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
