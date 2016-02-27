# -*- coding: utf-8 -*-
"""
Created on Sat Oct 10 17:30:31 2015

@author: Nick
"""
from PyQt4 import QtCore,QtGui,uic
import PyQt4.Qwt5 as Qwt

import sys
import serial, platform
import struct

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow,self).__init__()        
        uic.loadUi('C:/Users/Nick/Documents/nav6/qt/ui.ui',self)
        self.show()
        

class droneUI():
    def __init__(self):
        self.window=MyWindow()
        
        self.window.Compass.setNeedle(Qwt.QwtCompassMagnetNeedle())
    
        #Signals
        self.window.connectButton.released.connect(lambda: self.droneConnect(str(self.window.portsComboBox.currentText()))) #connects to port listed in box when connect button is pressed.
        self.window.refreshPortsButton.released.connect(self.updateSerialPortsList) #updates list of serial ports when refresh button is pressed.
        self.window.terminalSendButton.released.connect(self.sendTerminal)  #sends terminal data when send button is pressed
        self.window.terminalSendBox.returnPressed.connect(self.sendTerminal) #sends terminal data when enter is pressed
        
        self.updateSerialPortsList()
        

    
    def sendTerminal(self):      
        self.ser.write(str(self.window.terminalSendBox.displayText()).encode())
        self.window.terminalSendBox.clear()
        self.updateTerminalBox()
        
    def updateSerialPortsList(self):
        def list_serial_ports():
            # A function that tries to list serial ports on most common platforms
            system_name = platform.system()
            if system_name == "Windows":
                # Scan for available ports.
                available = []
                for i in range(256):
                    try:
                        s = serial.Serial(i)
                        available.append(i)
                        s.close()
                    except serial.SerialException:
                        pass
                return available
            elif system_name == "Darwin":
                # Mac
                return glob.glob('/dev/tty*') + glob.glob('/dev/cu*')
            else:
                # Assume Linux or something else
                return glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyUSB*')
        ports=list_serial_ports()
        self.window.portsComboBox.clear()
        for i in range(len(ports)):
            self.window.portsComboBox.addItem("COM"+str(ports[i]+1))
        
    def updateTerminalBox(self):
        a=''        
        while self.ser.inWaiting()>0:
            a+=self.ser.read()
        self.window.terminalDisplayBox.setText(a)

class Drone():
    def __init__(self,portName):
        self.port=serial.Serial(port=portName,baudrate=115200,timeout=2)
        self.port.write(b'isitadrone?/')
        if self.port.readline()!=b'yesitis!\r\n':
    def serialIn(comport,queueobject):
    cond=True
    ser=serial.Serial(port=comport,timeout=.5,baudrate=115200) 
    times=np.zeros((20))
    times[-1]=time.clock()
    j=0
    
    while cond:
        while True:
            b=ser.read()
            if ord(b)==0x7e:
                break
        b=ser.read()
        packetlength=struct.unpack('B',b)[0]          
        b=ser.read(packetlength)
        c=''     
        xorflag=False
        for i in range(len(b)):        
            if xorflag==False and ord(b[i])==0x7d:
                xorflag=True
            else:
                if xorflag==True and ((ord(b[i])^32==0x7e) or (ord(b[i])^32==0x7d)):
                    c+=chr(ord(b[i])^32)
                    xorflag=False
                else:
                    c+=b[i]
                    xorflag=False
        blist=[] 
        for i in range(len(c)//4):
            blist.append(c[i*4:i*4+4])
        for i in range(len(blist)):
            blist[i]=struct.unpack('f',blist[i])[0]
    
        try:
            queueobject.put(blist,block=False)
        except:
            pass
        times[:-1]=times[1:]
 
        times[-1]=time.clock()
        j+=1 

        if j==20:
            j=0            
          #  print "FPS",'%.0f'%(1/np.ediff1d(times).mean())
        try:
            out=exitqueue.get(block=False)
        except:
            continue      
        if out=='exit':
            ser.close()  
            print "serial closing"
            return        self.port.close()
            self.port=None
            raise NameError('Port not recognized as correct device! Did not receive proper response.')
    def setStream(self,Bool):
        if not isinstance(Bool,bool):
            raise TypeError("Bool must be boolean")
        if Bool: s=b'streamon'
        else: s=b'streamoff'
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
            elif xorflag==True and ((ord(b[i])^32==0x7e) or (ord(b[i])^32==0x7d)):
                c+=chr(ord(b[i])^32)
                xorflag=False
            else:
                c+=b[i]
                xorflag=False
        blist=[] 
        for i in range(len(c)//4):
            blist.append(struct.unpack('f',c[i*4:i*4+4])[0])
        return blist,skippedBytes
            


if __name__=='__main__':
    
    app=QtGui.QApplication(sys.argv)
    app.setStyle("plastique")
    ui=droneUI()
    
    sys.exit(app.exec_())