# -*- coding: utf-8 -*-
"""
Created on Thu May  5 18:36:38 2016

@author: Nick
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
data=pd.read_csv('log_27_2016-8-6-15-20-46.csv')

#strip nan and byte columns
da={}
for i in data.columns:
    if data[i].dtype==np.dtype('O'):
        del data[i]
        continue
    elif ((data[i].unique().shape==(1,)) and np.isnan(data[i].unique()[0])):
        del data[i]
        continue
    #Subdivide
    cat=i[:i.index('_')]
    if cat in da:
        data[i].name=i[i.index('_')+1:]
        da[cat]=pd.concat([da[cat],data[i]],axis=1)
    else:
        data[i].name=i[i.index('_')+1:]
        da[cat]=pd.DataFrame(data[i])

time=da["TIME"]["StartTime"]

subj=da["ATT"]
rollfig=plt.figure()
rollax=rollfig.add_subplot('111')
rollax.plot(time,subj["Roll"]/(2*np.pi)*360)
plt.title('Roll')

pitchfig=plt.figure()
pitchax=pitchfig.add_subplot('111')
pitchax.plot(time,subj["Pitch"]/(2*np.pi)*360)
plt.title('Pitch')

yawfig=plt.figure()
yawax=yawfig.add_subplot('111')
yawax.plot(time,subj["Yaw"]/(2*np.pi)*360+180)
plt.title('Yaw')

subj=da["ATTC"]
rollfig=plt.figure()
rollax=rollfig.add_subplot('111')
rollax.plot(time,subj["Roll"]/(2*np.pi)*360)
plt.title('Roll')

pitchfig=plt.figure()
pitchax=pitchfig.add_subplot('111')
pitchax.plot(time,subj["Pitch"]/(2*np.pi)*360)
plt.title('Pitch')

yawfig=plt.figure()
yawax=yawfig.add_subplot('111')
yawax.plot(time,subj["Yaw"]/(2*np.pi)*360+180)
plt.title('Yaw')

subj=da["ATSP"]
rollfig=plt.figure()
rollax=rollfig.add_subplot('111')
rollax.plot(time,subj["RollSP"]/(2*np.pi)*360)
plt.title('RollSP')

pitchfig=plt.figure()
pitchax=pitchfig.add_subplot('111')
pitchax.plot(time,subj["PitchSP"]/(2*np.pi)*360)
plt.title('PitchSP')

yawfig=plt.figure()
yawax=yawfig.add_subplot('111')
yawax.plot(time,subj["YawSP"]/(2*np.pi)*360+180)
plt.title('YawSP')

barofig=plt.figure()
baroax=barofig.add_subplot('211')
baroax3=barofig.add_subplot('212')
baroax.plot(time,da["SENS"]['BaroAlt'])
baroax2=baroax.twinx()
baroax2.plot(time,da["SENS"]['BaroPres'],'r--')
baroax3.plot(time,da["SENS"]['BaroTemp'])
plt.title('Baro')

subj=da['LPOS']
posfig=plt.figure()
posax=posfig.add_subplot('111')
#posax3=posax.twinx()
posax.plot(time,subj['Z'])
posax.plot(time,subj['VZ'],'g')
plt.title('pos z')
posfig2=plt.figure()
posax2=posfig2.add_subplot('111')
plt.title("XY")
#posax2.plot(time,subj["X"],time,subj["Y"])
posax2.scatter(subj["X"],subj["Y"])

subj=da['LPSP']
posfig=plt.figure()
posax=posfig.add_subplot('111')
#posax3=posax.twinx()
posax.plot(time,subj['Z'])
posax.plot(time,subj['VZ'],'g')
plt.title('pos z')
posfig2=plt.figure()
posax2=posfig2.add_subplot('111')
plt.title("XY")
#posax2.plot(time,subj["X"],time,subj["Y"])
posax2.scatter(subj["X"],subj["Y"])
'''
magfig=plt.figure()
da["IMU"]["MagMag"]=np.sqrt(da["IMU"]["MagX"]**2+da["IMU"]["MagY"]**2+da["IMU"]["MagZ"]**2)
magax=magfig.add_subplot('111')
magax.plot(time,da["IMU"]["MagMag"])
magax.plot(time,da["ATSP"]["ThrustSP"])
plt.title("Magnet magnitude")
'''
plt.figure()
for i in da["STAT"].columns:
    plt.plot(da["STAT"][i],label=i)
    
plt.legend()
plt.show()
'''
GPSfig
ATTCfig
ATSPfig
GPOSfig
STATfig
#ATT vs ATTC?
'''

'''TIME - Time stamp
ATT - Vehicle attitude
ATSP - Vehicle attitude setpoint
IMU - IMU sensors
SENS - Other sensors
LPOS - Local position estimate
LPSP - Local position setpoint
GPS - GPS position
ATTC - Attitude controls (actuator_0 output)
STAT - Vehicle state
RC - RC input channels
OUT0 - Actuator_0 output
AIRS - Airspeed
ARSP - Attitude rate setpoint
FLOW - Optical flow
GPOS - Global position estimate
GPSP - Global position setpoint
ESC - ESC state
GVSP - Global velocity setpoint
'''