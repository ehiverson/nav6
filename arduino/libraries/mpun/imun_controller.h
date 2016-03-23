

#ifndef _IMUN_CONT_
#define _IMUN_CONT_
#include <Arduino.h>



class PIDMaster{
	PIDMaster(ImunQuadMotor &qm,Quaternion &q);
	void YPR();
	float roll=0,pitch=0,yaw=0,rollset=0,pitchset=0,yawset=0;
	int time=0;
	PID yawPID(float &yaw,float &yawset,int &time);
	PID pitchPID(float &pitch,float &pitchset,int &time);
	PID rollPID(float &roll,float &rollset,int &time);
	
	ImunQuadMotor _qm;
	Quaternion *_q;
};

PIDMaster::PIDMaster(ImunQuadMotor &qm,Quaternion &q):_qm(qm)
{
	_q=&q;
}
void PIDMaster::YPR(){
		roll=atan2(2*(_q->w*_q->x+_q->y*_q->z),1-2*(pow(_q->x,2)+pow(_q->y,2)));
        yaw=atan2(2*(_q->w*_q->z+_q->x*_q->y),1-2*(pow(_q->y,2)+pow(_q->z,2)));
        pitch=asin(2*(_q->w*_q->y-_q->z*_q->x));
}
#endif //_IMUN_CONT_