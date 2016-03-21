
#ifndef _COMMSN
#define _COMMSN

class ImunComms
{
	public:
		ImunComms(Stream& serial);
		void receiveBytes(uint8_t* buffer);
		void receiveCommands(Quaternion q,Quaternion* qorientation);
		void transmitCommandPacket(String command);
		void transmitDataPacket(Quaternion q,VectorFloat magvec,VectorFloat accelvec,VectorFloat rmagvec);
		Stream& _serial;
		bool stream=false;
	private:
		void convertFloatToByte(float * buffer, uint8_t bufferLength,unsigned char  out[]);
		void transmitPacket(uint8_t* in, uint8_t length,uint8_t packetType);
};

ImunComms::ImunComms(Stream& serial):_serial(serial)
{

}

void ImunComms::transmitDataPacket(Quaternion q,VectorFloat magvec,VectorFloat accelvec,VectorFloat rmagvec)
{
	float buff[13]={q.w,q.x,q.y,q.z,magvec.x,magvec.y,magvec.z,accelvec.x,accelvec.y,accelvec.z,rmagvec.x,rmagvec.y,rmagvec.z};
    unsigned char p[sizeof(buff)+1];
    convertFloatToByte(buff,13,p);
    transmitPacket(p,sizeof(p),1);
}
void ImunComms::transmitCommandPacket(String command)
{
	uint8_t buff[256];
	command.getBytes(buff,sizeof(buff));
	transmitPacket(buff,command.length(),2);
}
void ImunComms::transmitPacket(uint8_t* in,uint8_t length,uint8_t packetType)
{
	uint8_t packet[256];
	packet[0]=0x7e;
	packet[1]=packetType;
	int index=3;
	for (uint8_t i=0;i<length;i++){
		if ((in[i]==0x7e) or (in[i]==0x7d)){
			length+=1;
			packet[index]=0x7d;
			packet[index+1]=in[i]^32;
			index+=2;
		}
		else{
			packet[index]=in[i];
			index++;
		}
	}
	
	packet[2]=length;
	_serial.write(packet,index);
}
void ImunComms::receiveBytes(uint8_t* buffer){
	while (true){
		if (_serial.read()==0x7e){break;}
	}
	uint8_t length=_serial.read();
	for (uint8_t i=0;i<length;i+=1){
		uint8_t in=_serial.read();
		if (in==0x7d){
			buffer[i]=_serial.read()^32;
		}
		else{buffer[i]=in;}
	}
}		

void ImunComms::convertFloatToByte(float buffer[],uint8_t bufferLength,unsigned char p[]){
	union {
		float a;
		unsigned char bytes[4];
	} thing;
	for (uint8_t i=0;i<bufferLength;i++){
		thing.a=buffer[i];
		memcpy(&p[i*sizeof(float)],&thing.bytes,4);
	}
	
}

void ImunComms::receiveCommands(Quaternion q,Quaternion* qorientation){
	while (_serial.available()>0)
	{
		String in=_serial.readStringUntil('/');
		if (in=="isitadrone?")
		{
			transmitCommandPacket("yesitis!");
		}
		
		if (in=="streamon")
			stream=true;
		if (in=="streamoff")
			stream=false;
		if (in=="flatten")
		{
			_serial.println(qorientation->w);
			_serial.println(qorientation->x);
			_serial.println(qorientation->y);
			_serial.println(qorientation->z);
			*qorientation=q;	
		}
	}
}




#endif				/* _COMMSN */

