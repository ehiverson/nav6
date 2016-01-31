

#ifndef _IMUN_CONT_
#define _IMUN_CONT_
#include <Arduino.h>


class PID
{
	float value=0;
	float P=1,I=1,D=1,integral=0,lastValue;
	float *_currentValue,*_setValue;
	int *_timeStamp,lastTimeStamp;
	public:
		PID(float *_currentValue,float *_setValue,int *_timestamp); 
		float calculate();
		void setPID(float p,float i,float d);
		
};

PID::PID(float *currentValue,float *setValue,int *timestamp){	
	_currentValue=currentValue;
	_setValue=setValue;
	_timeStamp=timestamp;
	lastValue=*_currentValue;
	lastTimeStamp=*_timeStamp;
}

float PID::calculate(){
	float difference=*_currentValue-*_setValue;
	integral+=difference/(*_timeStamp-lastTimeStamp);
	float derivative=(*_currentValue-lastValue)/(*_timeStamp-lastTimeStamp);
	return P*difference+I*integral+D*derivative;
}
void PID::setPID(float p, float i, float d){
	P=p;
	I=i;
	D=d;
}

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