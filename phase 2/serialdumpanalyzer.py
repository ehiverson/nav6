# -*- coding: utf-8 -*-
"""
Created on Thu Oct 20 22:20:55 2016

@author: Nick
"""

#CTS and RTS plotting still hasn't been implemented, may not be necessary

import pickle
import matplotlib.pyplot as plt
import numpy as np
with open('serialdump','rb') as f:
    dic=pickle.load(f)
#Events: stored is sets of three. (timestamp,bufferidentifier,index(if applicable))
#Eventidentifiers: 1 rx, 2 tx, 3 ctsrise, 4 ctsfall, 5 rtsrise, 6 rtsfall


class mavEvent():
    def __init__(self,timestamp,typ):
        self.timestamp=timestamp
        self.type=typ
    def setData(self,data):
        self.data=data
        self.length=len(data)*10*1000000/57600

events=[]
for j in range(len(dic['events'])):
    i=dic['events'][j]
    if i[1]==1 or i[1]==2:
        if i[1]==1: key='rx'
        elif i[1]==2: key='tx'
        _=mavEvent(i[0],i[1])
        if j==len(dic['events'])-1:
            _.setData(dic[key][i[2]:])
        else:
            _.setData(dic[key][i[2]:dic['events'][j+1][2]])
        events.append(_)      
    else:
        events.append(mavEvent(i[0],i[1]))


timestamps=[i.timestamp for i in events]
rxEvents=[i for i in events if i.type==1]
txEvents=[i for i in events if i.type==2]
ctsRiseEvents=[i for i in events if i.type==3]
ctsFallEvents=[i for i in events if i.type==4]
rtsRiseEvents=[i for i in events if i.type==5]
rtsFallEvents=[i for i in events if i.type==6]
indList=[rxEvents,txEvents]

fig=plt.figure()
ax=fig.add_subplot(1,1,1)

colors=['g','r','b','c']
for i in events:
    ax.barh(i.type,i.length,0.8,i.timestamp,color=colors[i.type-1],edgecolor='w',picker=True)

ax.set_xlim(0,timestamps[-1])
ax.set_ylim(1,5)
ax.set_yticks([1,2,3,4])
ax.set_yticklabels(['Rx','Tx','CTS','RTS'])
def onpick(event):
    art=event.artist
    if art.xy[1]==0: #RX event
        _=[i for i in rxEvents if i[0]==int(art.xy[0])][0]
        print(_)
        _=dic['rx'][_[2]]
    elif art.xy[1]==1: #TX event
        _=[i for i in txEvents if i[0]==int(art.xy[0])][0]
    print(_)
fig.canvas.mpl_connect('pick_event',onpick)
