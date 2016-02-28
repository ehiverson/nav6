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




if __name__=='__main__':
    
    app=QtGui.QApplication(sys.argv)
    app.setStyle("plastique")
    ui=droneUI()
    
    sys.exit(app.exec_())