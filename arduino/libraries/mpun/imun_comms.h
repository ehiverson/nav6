
#ifndef _COMMSN
#define _COMMSN

class ImunComms
{
	public:
		ImunComms(Stream& serial);
		void transmitBytes(uint8_t* in, uint8_t length);
		uint8_t receiveBytes(uint8_t* buffer);
		void convertData(float * buffer, uint8_t bufferLength);
		Stream& _serial;

};

ImunComms::ImunComms(Stream& serial):_serial(serial)
{

}


void ImunComms::transmitBytes(uint8_t* in,uint8_t length)
{
	uint8_t packet[256];
	packet[0]=0x7e;
	int index=2;
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
	packet[1]=length;
	_serial.write(packet,index);
}
uint8_t ImunComms::receiveBytes(uint8_t* buffer){
	while (true){
		if (_serial.read()==0x7e){break;}
	}
	uint8_t length=_serial.read();
	uint8_t finalLength=length;
	for (uint8_t i=0;i<length;i+=1){
		uint8_t in=_serial.read();
		if ((in==0x7d) and ((_serial.peek()^32==0x7e) or (_serial.peek()^32==0x7d))){
			finalLength-=1;
			buffer[i]=_serial.read()^32;
		}
		else{buffer[i]=in;}
	}
	return finalLength;
}		

void ImunComms::convertData(float* buffer,uint8_t bufferLength){
	unsigned char p[bufferLength*sizeof(float)];
	for (uint8_t i=0;i<bufferLength;i++){
		p[i*sizeof(float)]=(unsigned char *)&buffer[i];
	}

}


#endif				/* _COMMSN */

