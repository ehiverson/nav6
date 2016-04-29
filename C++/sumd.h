/*  SUMD serial (115200,8,n,1) protocol.
 
 http://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
 
 */
#ifndef SUMD_H
#define SUMD_H


class SUMDCommunicator
{
	public:
		uint8_t readPacket();
		void sendPacket(uint8_t numOfChannels,uint16_t* channelData);
		void sendFailsafe();
		uint16_t channelOutput[16];
		bool RXfailsafe=false;
	private:
		Stream& _serial;
		
}




// raw data (min mid max): 8800 12000 15200
SUMDCommunicator::SUMDCommunicator(Stream& serial): _serial(serial)
{
}

void SUMDCommunicator::sendFailsafe()
{
	uint8_t packet[2]={0xA8,0x81);
	_serial.write(packet,2);
void SUMDCommunicator::sendPacket(uint8_t numOfChannels,uint16_t channelData[])
{
	uint8_t packet[3+2*numOfChannels+2];
	packet[0]=0xA8;
	packet[1]=0x01;
	packet[2]=numOfChannels;
	for (uint8_t i==0;i<(2*numOfChannels);i++)
	{
		packet[3+2*i]=(uint8_t)(channelData[i*2]>>8);
		packet[4+2*i]=(uint8_t)(channelData[2*i+1]);
	}
	uint16_t crc = 0;
	for (uint8_t j=0;j<(3+2*numOfChannels);j++) 
	{
		crc^=(uint16_t)packet[j]<< 8;
		for (i=0;i<8;i++)
		{
			crc=(crc&0x8000)?(crc<<1)^0x1021:(crc<<1);
		}
	}
	packet[3+2*numOfChannels]=(uint8_t)(crc>>8);
	packet[4+2*numOfChannels]=(uint8_t)crc;
	_serial.write(packet,5+2*numofChannels);
}
uint8_t SUMDCommunicator::readPacket()
{
	//returns the number of channels received or 0 if there was an error.
	//writes the channel data at uin16_t to channelOutput.
	//value:	equivalent servo pulse time:	percentage:
	//12000		1500							0%
	//8800		1100							-100%
	//15200		1900							100%
	//16800		2100							150%
	//7200		900								-150%
	
	while true 
	{
		if (_serial.read() == 0xA8)//Sumd header indicates the beginning of a packet
		{
			uint8_t data=_serial.read()
			if (data==0x01)
			{
				RXfailsafe=false;
				break; //Status is ok
			}
			else if (data=0x81) //transmitter is indicating that there is an issue
			{
				RXfailsafe=true;
				return 0;
			}
		}
	} 
	channelCnt = _serial.read();
	uint8_t RXBuffer[3+2*channelCnt];
	RXBuffer[0]=0xA8;
	RXBuffer[1]=0x01;
	RXBuffer[2]=channelCnt;
	union
	{
		uint8_t channelBuffer[2];
		uint16_t channelValue;
	} converter;
	for (uint i=0;i<channelCnt;i++)
	{
		converter.channelBuffer[0]=_serial.read();
		converter.channelBuffer[1]=_serial.read();
		RXBuffer[3+i*2]=converter.channelBuffer[0];
		RXBuffer[4+i*2]=converter.channelBuffer[1];
		channelOutput[i]=converter.channelValue; //convert each 2 bytes to a 16 bit integer and store it in an output array.
	}
	/* SUMD has 16 bit CCITT CRC */
	uint16_t crc = 0;
	for (uint8_t j=0;j<(3+2*channelCnt);j++) 
	{
		crc^=(uint16_t)RXBuffer[j]<< 8;
		for (i=0;i<8;i++)
		{
			crc=(crc&0x8000)?(crc<<1)^0x1021:(crc<<1);
		}
	}
	uint8_t a=_serial.read();
	uint8_t b=_serial.read()
	if (crc!=(((uint16_t)a<<8)|b))
	{
		return 0;
	}
	return channelCnt;
}        

#endif