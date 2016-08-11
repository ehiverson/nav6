# -*- coding: utf-8 -*-
"""
Created on Sun Aug  7 16:39:25 2016

@author: Nick
"""
from PyQt4 import QtCore, QtGui,uic
import sys

class MyMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        uic.loadUi('untitled.ui', self)
        
if __name__=='__main__':
    app=QtGui.QApplication(sys.argv)
    win=MyMainWindow()
    for i in range(6):
        win.verticalLayout.addWidget(QtGui.QCheckBox(('hey')))
    win.show()
    sys.exit(app.exec())