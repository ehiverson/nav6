# -*- coding: utf-8 -*-
"""
Created on Sun Jun 28 12:17:13 2015

@author: Nick

q1=w
q2=x
q3=y
q4=z

"""
magcalibrate=False
magcalcheck=False

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import scipy.linalg as la
from pylab import *
from mpu9150mathfuncs import *
import time

from Drone import Drone



#Initiate plotting objects 
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





magarr=np.array([0,0,0])
m=np.array([0.0,0.0,0.0])
rm=np.array([0.0,0.0,0.0])
z=np.array([0.0,0.0,0.0])
a=np.array([0.0,0.0,0.0])
q=np.array([0.0,0.0,0.0,0.0])
lpm=np.array([0,0,0],dtype=float)
lpm2=np.array([0,0,0],dtype=float)
yawoffset=0

#Initial orientation vectors. These are rotated by the quaternion to provide visualization.
iovec=np.array([0,0,1])
iovec2=np.array([0,1,0])
iovec3=np.array([0,1,1])
iovec4=np.array([1,0,0])


drone=Drone('COM14')
drone.setStream(True)

#Loop variables.
cond=True
v=0
yaw2=0

try:
    while cond:

        #Serial data transfer        
        q,m,a,rm,skippedBytes=drone.dataQueue.get()
       
        print(skippedBytes)

        
        #print('r','%.1f' %np.rad2deg(np.arctan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2))),'y','%.1f'%np.rad2deg(np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))),'p', '%.1f'%np.rad2deg(np.arcsin(2*(q[0]*q[2]-q[3]*q[1]))))   
          
        
        
        

        proj=projectio(np.array([0,0,1]),m)

   


        #Rotate orientation vectors for visualization
        ovec=rotate2world(q,iovec)
        ovec2=rotate2world(q,iovec2)
        ovec3=rotate2world(q,iovec3)
        ovec4=rotate2world(q,iovec4)
 
        #Plotting
        try:
            accelline[0].remove()
            accel.remove()
            mag.remove()       
            project.remove()
            magline[0].remove()
            projline[0].remove()
            gpline[0].remove()
            mpline[0].remove()
            qline[0].remove()
            qline2[0].remove()
            qline3[0].remove()
            qline4[0].remove()
        except:
            pass
        
        mag=axis.scatter(m[0],m[1],m[2],color='g')
        project=axis.scatter(proj[0],proj[1],proj[2],color='c')   
        accelline=axis.plot([0,a[0]], [0,a[1]],[0,a[2]],color='m')
        accel=axis.scatter(a[0],a[1],a[2],color='m')        
        magline=axis.plot([0,m[0]], [0,m[1]],[0,m[2]],color='g')
        projline=axis.plot([0,proj[0]],[0,proj[1]],[0,proj[2]],color='c')     
        gpline=axis.plot([0,proj[0]],[0,proj[1]],[1,proj[2]],color='c')     
        mpline=axis.plot([m[0],proj[0]],[m[1],proj[1]],[m[2],proj[2]],color='c')                  
        qline=axis.plot([0,ovec[0],ovec2[0]],[0,ovec[1],ovec2[1]],[0,ovec[2],ovec2[2]],color='b')        
        qline2=axis.plot([0,ovec2[0]],[0,ovec2[1]],[0,ovec2[2]],color='r')        
        qline3=axis.plot([0,ovec3[0]/2],[0,ovec3[1]/2],[0,ovec3[2]/2],color='r')                
        qline4=axis.plot([0,ovec4[0]],[0,ovec4[1]],[0,ovec4[2]],color='r')        
        
        fig.canvas.draw()

        
        #Magnetic data accumulation for calibration.
        if magcalibrate or magcalcheck:
            magarr=np.vstack([magarr,rm])
        

finally:
    cond=False
    time.sleep(0.1)

    drone.exit=True
    plt.close(fig)
    
    if magcalibrate or magcalcheck:
        #run a magnetometer calibration 
        magarr=np.delete(magarr,0,axis=0)
        calmagarr=np.zeros(magarr.shape)
        print("Fitting Ellipse")
        ell=fitellipse(magarr)
        offsets=ell[1]
        invw=la.sqrtm(ell[0])
        
        for i in range(magarr.shape[0]):
            calmagarr[i,0],calmagarr[i,1],calmagarr[i,2]=calibratemag(magarr[i],offsets,invw)
        
        
        print("Plotting")
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
        print(invw)
        print(offsets)
        
        
        
        
        plt.show()
