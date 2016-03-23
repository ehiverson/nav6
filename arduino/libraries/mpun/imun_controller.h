

#ifndef _IMUN_CONT_
#define _IMUN_CONT_
#include <Arduino.h>


class PID
{
	float output=0,P,I,D,integral=0,lastValue,outMax,outMin;
	int lastTimeStamp;
	public:
		PID(float p, float i, float d, float minOut, float maxOut); 
		float calculate(float currentValue,float setValue,int timeStamp);
		void setPID(float p,float i,float d);
	private:
		void setOutputLimits(float Min, float Max);
		
};

PID::PID(float p, float i, float d, float minOut, float maxOut){
	setPID(p,i,d)
	setOutputLimits(minOut,maxOut);
}
		
//Does time need to be in seconds?
float PID::calculate(float currentValue,float setValue,int timeStamp){
	float error=currentValue-setValue;
	integral+=error*(timeStamp-lastTimeStamp);
	if (integral>outMax) integral=outMax;
	else if (integral<outMin) integral=outMin;
	float derivative=(currentValue-lastValue)/(timeStamp-lastTimeStamp);
	lastValue=currentValue;
	lastTimeStamp=timestamp;
	output=P*error+I*integral+D*derivative;
	if (output>outMax) output=outMax;
	else if (output<outMin) output=outMin;
}

void PID::setOutputLimits(float Min, float Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(output > outMax) output = outMax;
   else if(output < outMin) output = outMin;
 
   if(integral> outMax) integral= outMax;
   else if(integral< outMin) integral= outMin;
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