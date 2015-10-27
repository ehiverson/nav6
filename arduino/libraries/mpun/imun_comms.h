
#ifndef _COMMSN
#define _COMMSN

class ImunComms
{
	public:
		ImunComms(Stream& serial);
		void transmitBytes(float in);
		Stream& _serial;

};

ImunComms::ImunComms(Stream& serial):
	_serial(serial)
{

}


void ImunComms::transmitBytes(float in)
{
	unsigned char const *p=(unsigned char const *)&in;
	_serial.write(p,sizeof(float));
}


#endif				/* _COMMSN */

