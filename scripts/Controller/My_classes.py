from typing import NewType
from numpy.lib.function_base import disp
from sksurgerynditracker.nditracker import NDITracker
import socket
import time
import math
import numpy as np
import scipy.io as io
from tqdm import tqdm
import time
import pyquaternion
from pyquaternion import Quaternion
# from autolab_core import RigidTransform
# import cv2
import cv2
from autolab_core import RigidTransform
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from AHATNDITracker import *


class control_enum:
    create_axis=5
    create_axis_fb=6
    receive_axis=15
    get_hololens_pos=20
    receive_hololens_pos=25
    set_position_6d=30
    set_single_object=50
    

    

class code_generater:
    def __init__(self,control,para):
        self.control=control
        self.para=para
        self.control_dictionary=control_enum()

    def encode_mess(self):
        if self.control==self.control_dictionary.create_axis or self.control==self.control_dictionary.create_axis_fb:
            str_this = self.int2str(self.control,4)+self.float2str(self.para[0],16,16)+self.float2str(self.para[1],16,16)+self.float2str(self.para[2],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
        
        elif self.control==self.control_dictionary.get_hololens_pos:
            str_this=self.int2str(self.control,4)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
        
        elif self.control==self.control_dictionary.set_position_6d:
            str_this = self.int2str(self.control,4)+self.float2str(self.para[0],16,16)+self.float2str(self.para[1],16,16)+self.float2str(self.para[2],16,16)\
               +self.float2str(self.para[3],16,16)+self.float2str(self.para[4],16,16)+self.float2str(self.para[5],16,16)+self.float2str(self.para[6],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
        
        elif self.control==self.control_dictionary.set_single_object:
            ## here
            str_this = self.int2str(self.control,4)+\
                self.int2str(self.para[0],4)+\
                self.float2str(self.para[1],16,16)+\
                self.float2str(self.para[2],16,16)+\
                self.float2str(self.para[3],16,16)+\
                self.float2str(self.para[4],16,16)+\
                self.float2str(self.para[5],16,16)+\
                self.float2str(self.para[6],16,16)+\
                self.float2str(self.para[7],16,16)+"0"*1024
            str_this=str_this[0:1024]
            return str_this
            



    def float2str(self,num,int_,deci_):
        sgn_code="0" if num < 0 else "1"
        num = num if num >= 0 else -num
        int_part=math.floor(num)
        deci_part=math.floor((num-int_part)*10**deci_)
        str_this=sgn_code+"{:0>{width}}".format(int_part, width=int_)+"{:0>{width}}".format(deci_part, width=deci_)
        
        return str_this
    def int2str(self,num,int_):
        return "{:0>{width}}".format(num, width=int_) 

class code_decoder:
    def __init__(self,str_):
        self.message=str_
        self.control_dictionary=control_enum()

    def decode_mess(self,display=False):
        control_seq=self.message[0:4]
        control=self.str2int(control_seq)
        if control == self.control_dictionary.create_axis or control == self.control_dictionary.create_axis_fb or control==self.control_dictionary.receive_axis:
            x_pos=self.str2float(self.message[4:37],16,16)
            y_pos=self.str2float(self.message[37:70],16,16)
            z_pos=self.str2float(self.message[70:103],16,16)
            if display:
                print("control:  %d"%control)
                print("x_pos:    %f"%x_pos)
                print("y_pos:    %f"%y_pos)
                print("z_pos:    %f"%z_pos)
            return control,[x_pos,y_pos,z_pos]
        
        elif control==self.control_dictionary.receive_hololens_pos:
            x_pos=self.str2float(self.message[4:37],16,16)
            y_pos=self.str2float(self.message[37:70],16,16)
            z_pos=self.str2float(self.message[70:103],16,16)
            qw=self.str2float(self.message[103:136],16,16)
            qx=self.str2float(self.message[136:169],16,16)
            qy=self.str2float(self.message[169:202],16,16)
            qz=self.str2float(self.message[202:235],16,16)
            pos=[x_pos,y_pos,z_pos]
            quat=[qw,qx,qy,qz]
            if display:
                print("control:  %d"%control)
                print("x_pos:    %f"%x_pos)
                print("y_pos:    %f"%y_pos)
                print("z_pos:    %f"%z_pos)
                print("qw   :    %f"%qw)
                print("qx   :    %f"%qx)
                print("qy   :    %f"%qy)
                print("qz   :    %f"%qz)
            return control,[x_pos,y_pos,z_pos,qw,qx,qy,qz]
    
    def str2int(self,substring):
        
        return int(substring)
        
    def str2float(self,substring_,int_,deci_):
        sgn_part=substring_[0:1]
        int_part=substring_[1:1+int_]
        deci_part=substring_[1+int_:1+int_+deci_]
        sgn_=1 if sgn_part == "1" else -1
        # print(int_part)
        # print(deci_part)
        num=sgn_*(int(int_part)+int(deci_part)*10**(-deci_))
        return num

class socket_connecter:
    def __init__(self,ip_address,port):
        self.tcp_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server_addr=(ip_address,port)
        
    def connect(self):
        # print("trying to connect to :")
        # print(self.server_addr)
        self.tcp_socket.connect(self.server_addr)
        # print("connect success")

    def send(self,mess):
        self.tcp_socket.send(mess.encode("UTF-8"))

    def recv(self,length=1024):
        mess=self.tcp_socket.recv(1024)
        mess_= mess.decode("UTF-8")
        return mess_



class HololensTrackingObject:
    def __init__(self,TrackerFile,IP,PortUp=3000,PortDown=4000):
        self.TrackerFile=TrackerFile
        self.IP=IP 
        self.PortUp=PortUp
        self.PortDown=PortDown
        self.UpSocket=socket_connecter(self.IP,self.PortUp)
        self.DownSocket=socket_connecter(self.IP,self.PortDown)
        self.Connected=False
        
    def ConnectSocketChannel(self):
        if not self.Connected:
            self.UpSocket.connect()
            self.DownSocket.connect()
            self.Connected=True
            print("Connected")
        
    def SetPosition(self,TargetID,TrasnformMatrix):
        Quat=list(Quaternion(matrix=TrasnformMatrix[0:3,0:3]))
        ct=control_enum()
        code=code_generater(ct.set_single_object,[TargetID,TrasnformMatrix[0,3]/1000,-TrasnformMatrix[1,3]/1000,TrasnformMatrix[2,3]/1000,Quat[0],-Quat[1],Quat[2],-Quat[3]])
        cd=code.encode_mess()
        self.UpSocket.send(cd)
        
    
    def GetHololensPosition(self):
        ct=control_enum()
        code=code_generater(ct.get_hololens_pos,[])
        cd=code.encode_mess()
        self.UpSocket.send(cd)
        mess_back=self.DownSocket.recv()
        msb_decoder=code_decoder(mess_back)
        control_re,info_re=msb_decoder.decode_mess(False)
        return info_re

        
        
class TrackingTarget:
    def __init__(self,Name,TargetName,InitiateStatus,InitiatePosition,TrackingPoints,ROMFile):
        self.Name=Name
        self.TargetName=TargetName
        self.Status=InitiateStatus
        self.Position=InitiatePosition
        self.Rotationquat=[1,0,0,0]
        self.TrackingPoints=TrackingPoints
        self.ROMFile=ROMFile
        self.PointsinTracker=np.zeros((4,3))
        self.MatrixModeltoTracker=np.zeros((4,4))
        self.HasPoint=[False,False,False,False]
        self.PointError=[]
        self.SumErr=0


    def RegistrationModelTracker(self):
        print("Data display *******")
        print(self.TrackingPoints)
        print(self.PointsinTracker)
        self.MatrixModeltoTracker,self.SumErr,self.PointError=UVD_RigidTransform(self.TrackingPoints,self.PointsinTracker)


    def SetPoint(self,id,point):
        self.PointsinTracker[id]=point
        if not self.HasPoint[id]:
            self.HasPoint[id]=True
        if all(self.HasPoint):
            self.RegistrationModelTracker()
    
    def fetchresult(self):
        if all(self.HasPoint):
            return (True,self.MatrixModeltoTracker,self.PointError,self.SumErr)
        else:
            return (False,self.HasPoint)
        

    




