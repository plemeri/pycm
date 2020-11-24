from easydict import EasyDict as ed
import functools
from . import config

class Quantities:
    def __init__(self, key):
        self.qdict = dict()
        self._init_qdict()
        self.msg_list = []
        self._fill_msg_list(self.qdict)
        self.msg_list = [msg for msg in self.msg_list if key in msg]
        print(len(self.msg_list))

    @staticmethod
    def format_msg(msg):
        return "expr {$Qu(" + msg + ")}\r"


    def get(self, msg):
        return functools.reduce(getattr, [self.qdict] + msg.split('.'))

    def _fill_msg_list(self, qdict):
        if type(qdict) == ed:
            for key in qdict.keys():
                if key not in ['data', 'msg', 'type']:
                    self._fill_msg_list(qdict[key])
                elif key == 'msg':
                    self.msg_list.append(qdict[key])
                else:
                    continue
        else:
            raise AttributeError

    def __str__(self):
        out = ""
        for msg in self.msg_list:
            out += msg + ' -> ' + str(self.get(msg).data) + '\n'
        return out

    def _init_qdict(self):
        self.qdict= config.init_qdict
        self.qdict = ed(self.qdict)
