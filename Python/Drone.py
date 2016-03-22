# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 10:04:10 2016

@author: Nick
"""
import serial
import struct
import numpy as np
import threading
import queue

class Drone():
    def __init__(self,portName):
        self.port=serial.Serial(port=portName,baudrate=115200,timeout=2)
        self.thread=threading.Thread(target=self._threadedHandler)
        self.thread.daemon=True
        self.exit=False
        self.exitLock=threading.Lock()
        self.dataQueue=queue.LifoQueue(maxsize=1)
        self.commandOutQueue=queue.Queue()
        self.commandInQueue=queue.Queue()
        self.errorQueue=queue.Queue()
        self.thread.start()

        if not self.identify():
            self.port.close()
            self.port=None
            self.exit=True
            self.thread.join()
            raise NameError('Port not recognized as correct device! Did not receive proper response.')
    
    def __del__(self):
        self.close()
        
    def close(self):
        self.exit=True
        self.thread.join()
        self.port.close()

        
    def identify(self):
        self.commandOutQueue.put(b'isitadrone?/')
        read=self.commandInQueue.get()
        if read!=b'yesitis!':
            return False
        else:
            return True
            
        
    def setStream(self,Bool):
        if not isinstance(Bool,bool):
            raise TypeError("Bool must be boolean")
        if Bool: s=b'streamon/'
        else: s=b'streamoff/'
        self.commandOutQueue.put(s)
    def parseStream(self):
        skippedBytes=[]
        while True:
            b=self.port.read()
            if b[0]==0x7e:
                break
            skippedBytes.append(b)
        packetType=self.port.read()[0]
        packetLength=self.port.read()[0]          
        b=self.port.read(packetLength)
        c=b''     
        xorflag=False
        for i in range(len(b)):        
            if xorflag==False and b[i]==0x7d:
                xorflag=True
            elif xorflag==True: #and ((ord(b[i])^32==0x7e) or (ord(b[i])^32==0x7d)):
                c+=bytes([b[i]^32])
                xorflag=False
            else:
                c+=bytes([b[i]])
                #xorflag=False
        if packetType==1:
            blist=[] 
            for i in range(len(c)//4):
                blist.append(struct.unpack('f',c[i*4:i*4+4])[0])
            if len(blist)!=13:
                raise ValueError("Length of blist doesn't seem right!")
            return packetType,np.array(blist[:4]),np.array(blist[4:7]),np.array(blist[7:10]),np.array(blist[10:13]),skippedBytes
        elif packetType==2:
            return packetType,c
        else: raise ValueError('No Valid Packet recognized!',packetType)
    def _threadedHandler(self):
        while True:
            if self.port.in_waiting>0:
                a=self.parseStream()
                if a[0]==1:
                    try:
                        self.dataQueue.put(a[1:],block=False)
                    except queue.Full:
                        pass
                elif a[0]==2:
                    self.commandInQueue.put(a[1])
            if not self.commandOutQueue.empty():
                self.port.write(self.commandOutQueue.get())
            if self.exit:
                return
        