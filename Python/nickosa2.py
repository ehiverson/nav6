# -*- coding: utf-8 -*-
"""
Created on Sun Jun 28 12:17:13 2015

@author: Nick


"""

#enables an optional second plotting window.
magplot=True

'''
Replaces the calibrated magnetic data with raw data. At the end of execution
the script will plot a scatter plot of all the measured magnetic points. An offset from
the origin represents hard iron error. Ellipticity represents soft iron error.
The script will also print out new calibration values to be used.
'''
magcalibrate=False

#If true the script will, at the end of execution, plot a scatter plot of all 
#the measured magnetic field points. If they form a sphere that is centered at
#the origin the the magnetometer is properly calibrated.
magcalcheck=False

import serial
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.pylab import ion
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d import Axes3D
from time import sleep
import numpy as np
import math
import scipy.linalg as la
from mpu9150mathfuncs import *
from pylab import *
import time


#Set up plotting axes. 
fig = plt.figure()
axis = fig.gca(projection='3d')
axis.set_aspect("equal")
ra=1
axis.set_xlim([-ra,ra])
axis.set_ylim([-ra,ra])
axis.set_zlim([-ra,ra])
axis.plot([-ra, ra], [0,0],zs=[0,0],color='k')
axis.plot([0, 0], [-ra,ra],zs=[0,0],color='k')
axis.plot([0, 0], [0,0],zs=[-ra,ra],color='k')
axis.plot([0, 0], [ra,ra],zs=[-ra,ra],color='k')
axis.plot([-ra, ra], [ra,ra],zs=[0,0],color='k')
win=get_current_fig_manager()
win.window.wm_geometry("+0+0")
axis.view_init(25,90)
plt.show(block=False)

#Set up optional plots.
if magplot:
    fig2=plt.figure()
    win2=get_current_fig_manager()
    win2.window.wm_geometry("+0+600")
    axis3=fig2.add_subplot(121)
    axis4=fig2.add_subplot(122)
    axis3.set_aspect('equal')
    axis3.plot([0,0],[0,0])
    axis3.set_xlim([-1,1])
    axis3.set_ylim([-1,1])
    axis4.set_aspect('equal')
    axis4.plot([0,0],[0,0])
    axis4.set_xlim([-1,1])
    axis4.set_ylim([-1,1])
    plt.show(block=False)










#Compass Calibration Data
invw=np.array([[ 0.86683482, -0.02540421, -0.0678184 ],
 [-0.02540421,  0.78165629, -0.07496409],
 [-0.0678184,  -0.07496409,  0.52663494]])
offsets=np.array([ 0.27141446,  0.46536375,  0.66921883])

#Initialize values
ser=serial.Serial(port="COM12",timeout=2)
magarr=np.array([0,0,0])
m=np.array([0.0,0.0,0.0])
a=np.array([0.0,0.0,0.0])
q=np.array([0.0,0.0,0.0,0.0])
lpm=np.array([0,0,0],dtype=float)
lpm2=np.array([0,0,0],dtype=float)
yawoffset=0
iovec=np.array([0,0,1])
iovec2=np.array([0,1,0])
iovec3=np.array([0,1,1])
iovec4=np.array([1,0,0])
d=np.array([0,0,0],dtype=float)
pos=np.array([0,0,0],dtype=float)


i=0
cond=True
while cond:
    i+=1

    try:
        #obtain data over serial from teensy board
        ser.flushInput()
        b=ser.readline()
        print b
        b=b[:-2].split('/')
        print b
            
        
        q[0]=b[0]
        q[1]=b[1]
        q[2]=b[2]
        q[3]=b[3]
        

        

        #notice that mx and my are switched to acount for magnetometer orientation
        #mz is also negative.   

        m[1]=float(b[4])/100.0
        m[0]=float(b[5])/100.0
        m[2]=-float(b[6])/100.0
        
        a[0]=float(b[7])
        a[1]=float(b[8])
        a[2]=float(b[9])
        print b
        #if not magcalibrate:
            #APPLY COMPASS CALIBRATION
            #m=calibratemag(m,offsets,invw) 
        g=gravity(q)
        
        a-=g
        
        
        m=rotate2world(q,m)
        
        lpm=np.vstack([lpm,m])
        if len(lpm)>1000:
            lpm=np.delete(lpm,0,0)
        m=np.mean(lpm,axis=0)

        proj=projectio(np.array([0,0,1]),m)


   


                 
        
    
        
        #head=compassHeadingRadians(proj,np.radians(p),np.radians(r))               
        head=np.arctan2(m[0],m[1])
        if head<0:
            head=2*np.pi+head
        
        qmag=yawaxis2quat(head,np.array([0,0,1]))
        
                
        #quaternion mixing. This prevents gyro errors from causing yaw drift by
        #stabilising the quaternion to a low-pass filtered magnetic heading.
                
        fusion=quatmult(qmag,q)
        
        ovec=rotate2world(fusion,iovec)
        ovec2=rotate2world(fusion,iovec2)
        ovec3=rotate2world(fusion,iovec3)
        ovec4=rotate2world(fusion,iovec4)

        a=rotate2world(fusion,a)        
        g=rotate2world(fusion,g)
        
        
        #rudimentary acceleromter calibration
        if i==1:
            avg=a
        elif i<100:
            avg=np.vstack([avg,a])
        elif i==100:
            avg=np.mean(avg,axis=0)
        else:
            a-=avg
       
        
        
        
        #for visual purposes
        a=a*5
        
        #acceleration low-pass filter
        lpm2=np.vstack([lpm2,a])
        if len(lpm)>5:
            lpm2=np.delete(lpm2,0,0)
        a=np.mean(lpm2,axis=0)
        
        #unsuccessful attempt to calculation displacement from acceleration        
        d+=a
        pos+=d
        
        #Plotting
        mag=axis.scatter(m[0],m[1],m[2],color='g')
        grav=axis.scatter(g[0],g[1],g[2],color='m')
        project=axis.scatter(proj[0],proj[1],proj[2],color='c')       
        accelline=axis.plot([0,a[0]], [0,a[1]],[0,a[2]],color='m')
        magline=axis.plot([0,m[0]], [0,m[1]],[0,m[2]],color='g')
        gravline=axis.plot([0,g[0]],[0,g[1]],[0,g[2]],color='m')
        projline=axis.plot([0,proj[0]],[0,proj[1]],[0,proj[2]],color='c')     
        gpline=axis.plot([g[0],proj[0]],[g[1],proj[1]],[g[2],proj[2]],color='c')     
        mpline=axis.plot([m[0],proj[0]],[m[1],proj[1]],[m[2],proj[2]],color='c')                 
        qline=axis.plot([0,ovec[0],ovec2[0]],[0,ovec[1],ovec2[1]],[0,ovec[2],ovec2[2]],color='b')        
        qline2=axis.plot([0,ovec2[0]],[0,ovec2[1]],[0,ovec2[2]],color='r')        
        qline3=axis.plot([0,ovec3[0]/2],[0,ovec3[1]/2],[0,ovec3[2]/2],color='r')                
        qline4=axis.plot([0,ovec4[0]],[0,ovec4[1]],[0,ovec4[2]],color='r')        
        
        fig.canvas.draw()
       
        accelline[0].remove()
        mag.remove() 
        grav.remove()
        project.remove()
        magline[0].remove()
        gravline[0].remove()
        projline[0].remove()
        gpline[0].remove()
        mpline[0].remove()
        qline[0].remove()
        qline2[0].remove()
        qline3[0].remove()
        qline4[0].remove()
        
        #Optional plotting
        if magplot:
            mspecial=axis3.plot([0,np.cos(head)],[0,np.sin(head)],color='b')
            mspecial3=axis4.plot([0,np.cos(head)],[0,np.sin(head)],color='g')            
            fig2.canvas.draw()
            mspecial[0].remove()
            mspecial3[0].remove()
        
        if magcalibrate or magcalcheck:
            magarr=np.vstack([magarr,m])
        
        
        
        
    except KeyboardInterrupt:
        cond=False
        ser.close()
    except ValueError:
        print b
        ser.close()
        raise
        
    except:
        print "unknown error"
        cond=False
        ser.close()


#Close figures and serial port
plt.close(fig)
if magplot:
    plt.close(fig2)
ser.close()


#Plot out magnetometer data and calculate calibration values.
if magcalibrate or magcalcheck:
    magarr=np.delete(magarr,0,axis=0)
    calmagarr=np.zeros(magarr.shape)
    
    ell=fitellipse(magarr)
    offsets=ell[1]
    invw=la.sqrtm(ell[0])
    
    for i in range(magarr.shape[0]):
        calmagarr[i,0],calmagarr[i,1],calmagarr[i,2]=calibratemag(magarr[i],offsets,invw)
    
    
    
    fig=plt.figure()
    axis2=fig.gca(projection="3d")
    axis2.scatter(0,0,0,color='r')
    for i in range(magarr.shape[0]):
        axis2.scatter(magarr[i,0],magarr[i,1],magarr[i,2])
        axis2.scatter(calmagarr[i,0],calmagarr[i,1],calmagarr[i,2],color='g')
    axis2.set_xlabel('x')
    axis2.set_ylabel('y')
    axis2.set_zlabel('z')
    axis2.set_aspect("equal")
    axis2.set_xlim([-ra,ra])
    axis2.set_ylim([-ra,ra])
    axis2.set_zlim([-ra,ra])
    print invw
    print offsets
    
    
    
    
    plt.show()

