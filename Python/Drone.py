# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 10:04:10 2016

@author: Nick
"""
import serial
import struct
import numpy as np
import time

class Drone():
    '''
    I should make this a multithreaded object with one thread constantly parsing
    input data and separating the stream from command responses while the main thread handles them.
    '''
    def __init__(self,portName):
        self.port=serial.Serial(port=portName,baudrate=115200,timeout=2)
        self.setStream(False)
        time.sleep(0.1)
        self.port.reset_input_buffer()
        time.sleep(0.1)
        self.port.write(b'isitadrone?/')
        read=self.port.readline()
        if read!=b'yesitis!\r\n':
            self.port.close()
            self.port=None
            raise NameError('Port not recognized as correct device! Did not receive proper response. Received: %s'%read)
    def setStream(self,Bool):
        if not isinstance(Bool,bool):
            raise TypeError("Bool must be boolean")
        if Bool: s=b'streamon/'
        else: s=b'streamoff/'
        self.port.write(s)
    def parseDataStream(self):
        skippedBytes=0
        while True:
            b=self.port.read()
            if ord(b)==0x7e:
                break
            skippedBytes+=1
        b=self.port.read()
        packetlength=struct.unpack('B',b)[0]          
        b=self.port.read(packetlength)
        c=''     
        xorflag=False
        for i in range(len(b)):        
            if xorflag==False and ord(b[i])==0x7d:
                xorflag=True
            elif xorflag==True: #and ((ord(b[i])^32==0x7e) or (ord(b[i])^32==0x7d)):
                c+=chr(ord(b[i])^32)
                xorflag=False
            else:
                c+=b[i]
                #xorflag=False
        blist=[] 
        for i in range(len(c)//4):
            blist.append(struct.unpack('f',c[i*4:i*4+4])[0])
        if len(blist)!=13:
            raise ValueError("Length of blist doesn't seem right!")
        return np.array(blist[:4]),np.array(blist[4:7]),np.array(blist[7:10]),np.array(blist[10:13]),skippedBytes
    