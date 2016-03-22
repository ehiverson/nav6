# -*- coding: utf-8 -*-
"""
Created on Sat Oct 10 17:30:31 2015

@author: Nick
"""
from PyQt4 import QtCore,QtGui,uic
import PyQt4.Qwt5 as Qwt

import sys
import serial
from Drone import Drone

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow,self).__init__()        
        uic.loadUi('C:/Users/Nick/Documents/nav6/qt/ui.ui',self)
        self.show()
        

class droneUI():
    def __init__(self):
        self.window=MyWindow()
        self.drone=None
        
        self.window.Compass.setNeedle(Qwt.QwtCompassMagnetNeedle())
    
        #Signals
        self.window.connectButton.clicked.connect(lambda: self.connect(str(self.window.portsComboBox.currentText()))) #connects to port listed in box when connect button is pressed.
        self.window.terminalSendButton.clicked.connect(self.sendTerminal)  #sends terminal data when send button is pressed
        self.window.terminalSendBox.returnPressed.connect(self.sendTerminal) #sends terminal data when enter is pressed
        
        self.portRefreshTimer=QtCore.QTimer()
        self.portRefreshTimer.setInterval(3000)
        self.portRefreshTimer.timeout.connect(self.updateSerialPortsList)
        self.portRefreshTimer.start()
        

    
    def sendTerminal(self):      
        self.ser.write(str(self.window.terminalSendBox.displayText()).encode())
        self.window.terminalSendBox.clear()
        self.updateTerminalBox()
        
    def updateSerialPortsList(self):
        def list_serial_ports():
            """ Lists serial port names
        
                :raises EnvironmentError:
                    On unsupported or unknown platforms
                :returns:
                    A list of the serial ports available on the system
            """
            if sys.platform.startswith('win'):
                ports = ['COM%s' % (i + 1) for i in range(256)]
            elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
                # this excludes your current terminal "/dev/tty"
                ports = glob.glob('/dev/tty[A-Za-z]*')
            elif sys.platform.startswith('darwin'):
                ports = glob.glob('/dev/tty.*')
            else:
                raise EnvironmentError('Unsupported platform')
        
            result = []
            for port in ports:
                try:
                    s = serial.Serial(port)
                    s.close()
                    result.append(port)
                except (OSError, serial.SerialException):
                    pass
            return result
        ports=list_serial_ports()
        self.window.portsComboBox.clear()
        for i in range(len(ports)):
            self.window.portsComboBox.addItem(ports[i])
        
    def updateTerminalBox(self):
        a=''        
        while self.ser.inWaiting()>0:
            a+=self.ser.read()
        self.window.terminalDisplayBox.setText(a)
    
    def connect(self,portname):
        self.drone=Drone(portname)
        self.window.connectButton.setText('Disconnect')
        self.window.connectButton.clicked.disconnect()
        self.window.connectButton.clicked.connect(self.disconnect)
    def disconnect(self):
        self.drone.close()
        self.drone=None
        self.window.connectButton.setText('Connect')
        self.window.connectButton.clicked.disconnect()
        self.window.connectButton.clicked.connect(self.connect)



if __name__=='__main__':
    
    app=QtGui.QApplication(sys.argv)
    app.setStyle("plastique")
    ui=droneUI()
    
    sys.exit(app.exec_())