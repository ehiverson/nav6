
#ifndef _COMMSN
#define _COMMSN

class ImunComms
{
	public:
		ImunComms(HardwareSerial& serial);
		void floatToBytes(float in);
	private:
		HardwareSerial& _serial;

};

ImunComms::ImunComms(HardwareSerial& serial):
	_serial(serial)
{


}


void ImunComms::floatToBytes(float in)
{
	uint32_t b=(uint32_t)in;
	uint32_t* a=(uint32_t*)&b;
	for (int i=0;i<4;i++){	
		_serial.write(*(a+i));
	}
}


#endif				/* _COMMSN */

