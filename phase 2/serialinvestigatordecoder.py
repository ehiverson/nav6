# -*- coding: utf-8 -*-
"""
Created on Thu Oct 20 09:00:01 2016

@author: Nick
"""

import serial
import time
import pickle
import io



evbuffer=[]
def readev():
    ev=[]
    for i in range(3):
        _=readline()
        try:
            if _==b'rxbuffer\r\n': return -1
            ev.append(int(_.decode()[:-2]))
        except:
            print(_)
            raise
    return ev

ser=serial.Serial("COM12",baudrate=115200,timeout=2)
def readline():
    eol=b'\r\n'
    leneol = len(eol)
    line = bytearray()
    while True:
        c = ser.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:
                break
        else:
            break
    return bytes(line)
ser.write(b'go/')
ser.flushInput()
while True:
    _=readline()
    print(_[:-2])
    if _==b'stats\r\n': break
    time.sleep(0.1)
print('evind,rxind,txind')
for i in range(3):
    print(readline().decode()[:-2])
_=readline()
if _!=b'evbuffer\r\n':
    print(_)
    raise ValueError


while True:
    _=readev()
    if _==-1: break
    else:evbuffer.append(_)
rxbuffer=readline()[:-2]
_=readline()
if _!=b'txbuffer\r\n':
    raise ValueError(_)
txbuffer=readline()[:-2]
_=readline()
if _!=b'stop\r\n':
    raise ValueError(_)
dic={'events':evbuffer,'tx':txbuffer,'rx':rxbuffer}
with open('serialdump','wb') as f:
    pickle.dump(dic,f)
    