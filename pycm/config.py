IP_ADDRESS = 'localhost' 
PORT = 16600

def init_qdict():
    qdict = {
                'Car':{
                    'ax':{'data': None, 'msg': 'Car.ax', 'type': 'm/s'},
                    'ay':{'data': None, 'msg': 'Car.ay', 'type': 'm/s'},
                    'az':{'data': None, 'msg': 'Car.az', 'type': 'm/s'},
                    'tx':{'data': None, 'msg': 'Car.tx', 'type': 'm'},
                    'ty':{'data': None, 'msg': 'Car.ty', 'type': 'm'},
                    'tz':{'data': None, 'msg': 'Car.tz', 'type': 'm'},
                    'v':{'data': None, 'msg': 'Car.v', 'type': 'm/s'},
                    'Yaw':{'data': None, 'msg': 'Car.Yaw', 'type': 'rad'},
                    'YawAcc':{'data': None, 'msg': 'Car.YawAcc', 'type': 'rad/s^2'},
                    'YawRate':{'data': None, 'msg': 'Car.YawRate', 'type': 'rad/s'},
                    'YawVel':{'data': None, 'msg': 'Car.YawVel', 'type': 'rad/s'},
                    'Road':{
                        'Lane':{

                        }
                    }
                },
                'Driver':{
                    'Lat':{
                        'dy':{'data': None, 'msg': 'Driver.Lat.dy', 'type': 'm'},
                        'passive':{'data': None, 'msg': 'Driver.Lat.passive', 'type': 'bool'},
                        },
                    'Long':{
                        'dv':{'data': None, 'msg': 'Driver.Long.dv', 'type': 'm'},
                        'passive':{'data': None, 'msg': 'Driver.Long.passive', 'type': 'bool'},
                    }
                },
                'LineDetect':{
                    'nPtsL':{'data': None, 'msg': 'LineDetect.nPtsL', 'type': 'int'},
                    'nPtsR':{'data': None, 'msg': 'LineDetect.nPtsR', 'type': 'int'},
                    'pL':{},
                    'pR':{}
                },
                'Sensor':{
                    'Camera':{
                        'CA00':{
                            'nObj':{'data': None, 'msg': 'Sensor.Camera.CA00.nObj', 'type': 'int'},
                            'Obj':{}
                        },
                        'CA01':{
                            'nObj':{'data': None, 'msg': 'Sensor.Camera.CA01.nObj', 'type': 'int'},
                            'Obj':{}
                        },
                        'CA02':{
                            'nObj':{'data': None, 'msg': 'Sensor.Camera.CA02.nObj', 'type': 'int'},
                            'Obj':{}
                        },
                        'CA03':{
                            'nObj':{'data': None, 'msg': 'Sensor.Camera.CA03.nObj', 'type': 'int'},
                            'Obj':{}
                        }
                    },
                    'Line':{
                        'Front':{
                            'LLines':{},
                            'RLines':{},
                            'nLine_Left':{'data': None, 'msg': 'Sensor.Line.Front.nLine_Left', 'type': 'int'},
                            'nLine_Right':{'data': None, 'msg': 'Sensor.Line.Front.nLine_Right', 'type': 'int'},
                            'TimeStamp':{'data': None, 'msg': 'Sensor.Line.Front.TimeStamp', 'type': 'float'},
                        }
                    },
                    'Object':{
                        'nSensors':{'data': None, 'msg': 'Sensor.Object.nSensors', 'type': 'float'},
                        'RadarL':{
                            'DrvLaneCurv': {'data': None, 'msg': 'Sensor.Object.RadarL.DrvLaneCurv', 'type': 'float'},
                            'Obj':{},
                            'relvTgt':{
                                'dtct':{'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.dtct', 'type': 'float'},
                                'NearPnt':{
                                    'alpha':{'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.alpha', 'type': 'float'},
                                    'ds':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.ds.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.ds.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.ds.z', 'type': 'float'},
                                    },
                                    'ds_p': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.ds_p', 'type': 'float'},
                                    'dv':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.dv.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.dv.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.dv.z', 'type': 'float'},
                                    },
                                    'dv_p': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.NearPnt.dv_p', 'type': 'float'},
                                },
                                'ObjId': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.ObjId', 'type': 'float'},
                                'RefPnt':{
                                    'alpha':{'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.alpha', 'type': 'float'},
                                    'ds':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.ds.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.ds.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.ds.z', 'type': 'float'},
                                    },
                                    'ds_p': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.ds_p', 'type': 'float'},
                                    'dv':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.dv.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.dv.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.dv.z', 'type': 'float'},
                                    },
                                    'dv_p': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.dv_p', 'type': 'float'},
                                    'r_zyx':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.r_zyx.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.r_zyx.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.r_zyx.z', 'type': 'float'},
                                    },
                                    'theta': {'data': None, 'msg': 'Sensor.Object.RadarL.relvTgt.RefPnt.theta', 'type': 'float'}
                                }
                            },
                            'TimeStamp': {'data': None, 'msg': 'Sensor.Object.RadarL.TimeStamp', 'type': 'float'}
                        },
                        'RadarR':{
                            'DrvLaneCurv': {'data': None, 'msg': 'Sensor.Object.RadarR.DrvLaneCurv', 'type': 'float'},
                            'Obj':{},
                            'relvTgt':{
                                'dtct':{'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.dtct', 'type': 'float'},
                                'NearPnt':{
                                    'alpha':{'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.alpha', 'type': 'float'},
                                    'ds':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.ds.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.ds.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.ds.z', 'type': 'float'},
                                    },
                                    'ds_p': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.ds_p', 'type': 'float'},
                                    'dv':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.dv.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.dv.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.dv.z', 'type': 'float'},
                                    },
                                    'dv_p': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.NearPnt.dv_p', 'type': 'float'},
                                },
                                'ObjId': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.ObjId', 'type': 'float'},
                                'RefPnt':{
                                    'alpha':{'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.alpha', 'type': 'float'},
                                    'ds':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.ds.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.ds.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.ds.z', 'type': 'float'},
                                    },
                                    'ds_p': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.ds_p', 'type': 'float'},
                                    'dv':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.dv.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.dv.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.dv.z', 'type': 'float'},
                                    },
                                    'dv_p': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.dv_p', 'type': 'float'},
                                    'r_zyx':{
                                        'x': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.r_zyx.x', 'type': 'float'},
                                        'y': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.r_zyx.y', 'type': 'float'},
                                        'z': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.r_zyx.z', 'type': 'float'},
                                    },
                                    'theta': {'data': None, 'msg': 'Sensor.Object.RadarR.relvTgt.RefPnt.theta', 'type': 'float'}
                                }
                            },
                            'TimeStamp': {'data': None, 'msg': 'Sensor.Object.RadarR.TimeStamp', 'type': 'float'}
                        }
                    }
                },
                'TrfLight':{
                    'CtrlTL000_InG':{
                        'Mode':{'data': None, 'msg': 'TrfLight.CtrlTL000_InG.Mode', 'type': 'float'},
                        'State':{'data': None, 'msg': 'TrfLight.CtrlTL000_InG.State', 'type': 'float'},
                        'tRemain':{'data': None, 'msg': 'TrfLight.CtrlTL000_InG.tRemain', 'type': 'float'},
                    },
                    'CtrlTL000_inR':{
                        'Mode':{'data': None, 'msg': 'TrfLight.CtrlTL000_inR.Mode', 'type': 'float'},
                        'State':{'data': None, 'msg': 'TrfLight.CtrlTL000_inR.State', 'type': 'float'},
                        'tRemain':{'data': None, 'msg': 'TrfLight.CtrlTL000_inR.tRemain', 'type': 'float'},
                    },
                    'Ped_inG':{
                        'Mode':{'data': None, 'msg': 'TrfLight.Ped_inG.Mode', 'type': 'float'},
                        'State':{'data': None, 'msg': 'TrfLight.Ped_inG.State', 'type': 'float'},
                        'tRemain':{'data': None, 'msg': 'TrfLight.Ped_inG.tRemain', 'type': 'float'},
                    },
                    'Ped_inR':{
                        'Mode':{'data': None, 'msg': 'TrfLight.Ped_inR.Mode', 'type': 'float'},
                        'State':{'data': None, 'msg': 'TrfLight.Ped_inR.State', 'type': 'float'},
                        'tRemain':{'data': None, 'msg': 'TrfLight.Ped_inR.tRemain', 'type': 'float'},
                    },
                },
                'UDP':{
                    'Ax':{'data': None, 'msg': 'UDP.Ax', 'type': 'm/s^2'},
                    'GearNo':{'data': None, 'msg': 'UDP.GearNo', 'type': 'float'},
                    'Length':{'data': None, 'msg': 'UDP.Length', 'type': 'float'},
                    'VC_SwitchOn':{'data': None, 'msg': 'UDP.VC_SwitchOn', 'type': 'float'},
                    'WheelAng':{'data': None, 'msg': 'UDP.WheelAng', 'type': 'rad'},
                },
                'Vhcl':{
                    'Yaw':{'data': None, 'msg': 'Vhcl.Yaw', 'type': 'float'},
                    'Roll':{'data': None, 'msg': 'Vhcl.Roll', 'type': 'float'},
                    'Pitch':{'data': None, 'msg': 'Vhcl.Pitch', 'type': 'float'},
                }}

    for i in range(30):
        qdict['LineDetect']['pL'][str(i)] = {
            'x':{'data': None, 'msg': 'LineDetect.pL.' + str(i) + '.x', 'type': 'int'},
            'y':{'data': None, 'msg': 'LineDetect.pL.' + str(i) + '.y', 'type': 'int'},
            'z':{'data': None, 'msg': 'LineDetect.pL.' + str(i) + '.z', 'type': 'int'}
        }
        qdict['LineDetect']['pR'][str(i)] = {
            'x':{'data': None, 'msg': 'LineDetect.pR.' + str(i) + '.x', 'type': 'int'},
            'y':{'data': None, 'msg': 'LineDetect.pR.' + str(i) + '.y', 'type': 'int'},
            'z':{'data': None, 'msg': 'LineDetect.pR.' + str(i) + '.z', 'type': 'int'}
        }

    for cam in qdict['Sensor']['Camera'].keys():
        for i in range(50):
            qdict['Sensor']['Camera'][cam]['Obj'][str(i)] = {
                'Confidence':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.Confidence', 'type': 'float'},
                'Facing':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.Facing', 'type': 'float'},
                'LightState':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.LightState', 'type': 'float'},
                'MBR':{
                    'BL_X':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.MBR.BL_X', 'type': 'float'},
                    'BL_Y':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.MBR.BL_Y', 'type': 'float'},
                    'BL_Z':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.MBR.BL_Z', 'type': 'float'},
                    'TR_X':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.MBR.TR_X', 'type': 'float'},
                    'TR_Y':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.MBR.TR_Y', 'type': 'float'},
                    'TR_Z':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.MBR.TR_Z', 'type': 'float'}
                },
                'nVisPixels':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.nVisPixels', 'type': 'float'},
                'ObjID':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.ObjID', 'type': 'float'},
                'SignMain':{
                    'Val0':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.SignMain.Val0', 'type': 'float'},
                    'Val1':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.SignMain.Val1', 'type': 'float'}
                },
                'Type':{'data': None, 'msg': 'Sensor.Camera.' + cam + '.Obj.' + str(i) + '.Type', 'type': 'float'}
            }

    for i in range(1, 11):
        qdict['Sensor']['Line']['Front']['LLines'][str(i)] = {
            'ColorCode':{'data': None, 'msg': 'Sensor.Line.Front.LLines.' + str(i) + '.ColorCode', 'type': 'float'},
            'Height':{'data': None, 'msg': 'Sensor.Line.Front.LLines.' + str(i) + '.Height', 'type': 'float'},
            'Id':{'data': None, 'msg': 'Sensor.Line.Front.LLines.' + str(i) + '.Id', 'type': 'float'},
            'Type':{'data': None, 'msg': 'Sensor.Line.Front.LLines.' + str(i) + '.Type', 'type': 'float'},
            'Width':{'data': None, 'msg': 'Sensor.Line.Front.LLines.' + str(i) + '.Width', 'type': 'float'},
        }
        qdict['Sensor']['Line']['Front']['RLines'][str(i)] = {
            'ColorCode':{'data': None, 'msg': 'Sensor.Line.Front.RLines.' + str(i) + '.ColorCode', 'type': 'float'},
            'Height':{'data': None, 'msg': 'Sensor.Line.Front.RLines.' + str(i) + '.Height', 'type': 'float'},
            'Id':{'data': None, 'msg': 'Sensor.Line.Front.RLines.' + str(i) + '.Id', 'type': 'float'},
            'Type':{'data': None, 'msg': 'Sensor.Line.Front.RLines.' + str(i) + '.Type', 'type': 'float'},
            'Width':{'data': None, 'msg': 'Sensor.Line.Front.RLines.' + str(i) + '.Width', 'type': 'float'},
        }

    return qdict

def init_obj_dict(nobj):
    obj_dict = {
        'dtct':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.dtct', 'type': 'float'},
        'InLane':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.InLane', 'type': 'float'},
        'NearPnt':{
            'alpha': {'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.alpha', 'type': 'float'},
            'ds':{
                'x':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.ds.x', 'type': 'float'},
                'y':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.ds.y', 'type': 'float'},
                'z':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.ds.y', 'type': 'float'}
            },
            'ds_p':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.ds_p', 'type': 'float'},
            'dv':{
                'x':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.dv.x', 'type': 'float'},
                'y':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.dv.y', 'type': 'float'},
                'z':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.dv.y', 'type': 'float'}
            },
            'dv_p':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.NearPnt.dv_p', 'type': 'float'}
        },
        'obsv': {'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.obsv', 'type': 'float'},
        'RefPnt':{
            'alpha': {'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.alpha', 'type': 'float'},
            'ds':{
                'x':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.ds.x', 'type': 'float'},
                'y':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.ds.y', 'type': 'float'},
                'z':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.ds.y', 'type': 'float'}
            },
            'ds_p':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.ds_p', 'type': 'float'},
            'dv':{
                'x':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.dv.x', 'type': 'float'},
                'y':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.dv.y', 'type': 'float'},
                'z':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.dv.y', 'type': 'float'}
            },
            'dv_p':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.dv_p', 'type': 'float'},
            'r_zyx':{
                'x':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.r_zyx.x', 'type': 'float'},
                'y':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.r_zyx.y', 'type': 'float'},
                'z':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.r_zyx.y', 'type': 'float'}
            },
            'theta':{'data': None, 'msg': 'Sensor.Object.RadarL.Obj.T' + str(nobj).zfill(2) + '.RefPnt.theta', 'type': 'float'}
        }
    }
    return obj_dict