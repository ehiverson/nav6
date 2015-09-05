# -*- coding: utf-8 -*-
"""
Created on Sat Jul 11 20:48:48 2015

@author: Nick Anthony
"""
import numpy as np
import math

import scipy.linalg as la

def quatmult(q1,q2):
    #Multiplies two quaternions, q1 and q2
    Q=np.array([0,0,0,0],dtype=float)
    Q[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]
    Q[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2]
    Q[2]=q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1]
    Q[3]=q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]
    return Q
 
def quatconj(quat):
    #returns the conjugate of the provided quaternion
    return np.array([quat[0],-quat[1],-quat[2],-quat[3]])   

def rotate2world(quat,vec):
    #rotates the vector, vec, by the rotation described by the quaternion, quat
    quat2=np.array([0,vec[0],vec[1],vec[2]])
    qprod=quatmult(quat,quat2)
    qconj=quatconj(quat)
    qfinal=quatmult(qprod,qconj)
    return qfinal[1:]
    
def calibratemag(m,offsets,invw):
    #Given the raw magnetic field vector from an MPU9150, m, a vector describing 
    #the hard iron offset of the magetometer, offsets, and a 3x3 matrix describing
    #the elliptical soft iron error of the magnetometer, invw; this function returns
    #a calibrated magnetic field vector which should always have the same magnitude
    #if the there is no magnetic field source other than the earth's present.
    m[0]-=offsets[0]
    m[1]-=offsets[1]
    m[2]-=offsets[2]
    iSumx=invw[0,0]*m[0]+invw[0,1]*m[1]+invw[0,2]*m[2]
    iSumy=invw[1,0]*m[0]+invw[1,1]*m[1]+invw[1,2]*m[2]
    iSumz=invw[2,0]*m[0]+invw[2,1]*m[1]+invw[2,2]*m[2]
    return np.array([iSumx,iSumy,iSumz])

def fitellipse(points, tol = 0.001):
    """
    Find the minimum volume ellipse that bounds all points.
    Return A, c where the equation for the ellipse given in "center form" is
    (x-c).T * A * (x-c) = 1
    """
    points = np.asmatrix(points)
    N, d = points.shape
    Q = np.column_stack((points, np.ones(N))).T
    err = tol+1.0
    u = np.ones(N)/N
    while err > tol:
        X = Q * np.diag(u) * Q.T
        M = np.diag(Q.T * la.inv(X) * Q)
        jdx = np.argmax(M)
        step_size = (M[jdx]-d-1.0)/((d+1)*(M[jdx]-1.0))
        new_u = (1-step_size)*u
        new_u[jdx] += step_size
        err = la.norm(new_u-u)
        u = new_u
    c = u*points
    A = la.inv(points.T*np.diag(u)*points - c.T*c)/d    
    return np.asarray(A), np.squeeze(np.asarray(c))

def decode(inp):
    outp=int(inp,16)
    if outp>0x7FFF:
        outp-=0x10000
    return outp
    
def gravity(q):
    #Uses the quaternion from the MPU9150 sensor to obtain a vector that points
    #away from the ground. This vector is in the MPU9150's rotational frame of reference.
    gx=2*(q[1]*q[3]-q[0]*q[2])
    gy=2*(q[0]*q[1]+q[2]*q[3])
    gz=(q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])
    return np.array([gx,gy,gz])
    
def YawPitchRoll(q,g):
    #Returns the yaw, pitch, and roll of the mpu9150 from its quaternion.
  yaw= math.degrees(np.arctan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1))
  pitch = math.degrees(np.arctan(g[0] / np.sqrt(g[1]**2 + g[2]**2)))
  roll = math.degrees(np.arctan(g[1] / np.sqrt(g[0]**2+ g[2]**2)))
  return yaw,pitch,roll

    
    
    
def compassHeadingRadians(mag,pitch_radians,roll_radians):
    #Make sure that the mag vector is perpendicular to gravity. i.e. it should have inclination removed.
    cos_roll = np.cos(roll_radians)
    sin_roll = np.sin(roll_radians)
    cos_pitch = np.cos(pitch_radians)
    sin_pitch = np.sin(pitch_radians)
    MAG_X = mag[0] * cos_pitch + mag[2] * sin_pitch
    MAG_Y = mag[0] * sin_roll * sin_pitch + mag[1] * cos_roll - mag[2] * sin_roll * cos_pitch
    tilt_compensated_heading = np.arctan2(MAG_Y,MAG_X)
    return tilt_compensated_heading;
    
def yawaxis2quat(yawradians,vecaxis):
    #Converts an axis of rotation and the rotation in radians to a quaternion.
    a=np.sin(yawradians/2)
    b=np.array([np.cos(yawradians/2),a*vecaxis[0],a*vecaxis[1],a*vecaxis[2]])
    return b/np.sqrt(np.dot(b,b))
def projectio(g,m):
    #returns the projection of vector, m, perpendicular to vector, g.
    return m-np.dot(m,g)*g/np.sqrt(np.dot(g,g))**2