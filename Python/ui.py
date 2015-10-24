# -*- coding: utf-8 -*-
"""
Created on Sat Oct 10 17:30:31 2015

@author: Nick
"""
from PyQt4 import QtCore,QtGui,uic

import sys
import serial, platform

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow,self).__init__()        
        uic.loadUi('C:/Users/Nick/Documents/Github/nav6/qt/ui.ui',self)
        self.show()
        
# A function that tries to list serial ports on most common platforms
def list_serial_ports():
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
if __name__=='__main__':
    app=QtGui.QApplication(sys.argv)
    ports=list_serial_ports()
    print ports
    window=MyWindow()
    for i in range(len(ports)):
        window.SerialPortComboBox.addItem("COM"+str(ports[i]+1))
    window.Compass.setNeedle(QwtCompassMagnetNeedle())
    sys.exit(app.exec_())