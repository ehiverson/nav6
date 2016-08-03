
#include <XBOXUSB.h>
#include <sumd.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif


USB Usb;
XBOXUSB Xbox(&Usb);

SUMDCommunicator comms(Serial1);
uint8_t numOfChannels=10;
elapsedMicros t=0;
elapsedMillis t2=0;
bool pulseon=false;
uint8_t j=0;
bool debug=false;
uint16_t channelSelectSwitch=8800;
int pulsepin=15;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  delay(1000);
  pinMode(pulsepin,OUTPUT);
}


void loop() {
  
  Usb.Task();
  if (Serial.available()>0){
    if (Serial.readStringUntil('/')=="debug"){
      debug=true;
      }
    }
   
  if (t2>=10000){
    digitalWrite(pulsepin,HIGH);  
    t2=0;
    pulseon=true;
  }
  if (pulseon and (t2>=18)){
    digitalWrite(pulsepin,LOW);
    pulseon=false;
  }
  
  if (t>=10000){
    t=0;
    if (Xbox.Xbox360Connected) {
  
      if (Xbox.getButtonClick(DOWN))
        channelSelectSwitch=convertPercent(0);
      if (Xbox.getButtonClick(RIGHT))
        channelSelectSwitch=convertPercent(20);
      if (Xbox.getButtonClick(LEFT))
        channelSelectSwitch=convertPercent(40);
      if (Xbox.getButtonClick(UP))
        channelSelectSwitch=convertPercent(60);
      if (Xbox.getButtonClick(A))
        channelSelectSwitch=convertPercent(80);
      if (Xbox.getButtonClick(X))
        channelSelectSwitch=convertPercent(100);
      uint16_t packet[numOfChannels];
      packet[0]=convert(Xbox.getAnalogHat(LeftHatX));
      packet[1]=convert(Xbox.getAnalogHat(LeftHatY));
      packet[2]=convert(Xbox.getAnalogHat(RightHatX));
      packet[3]=convert(Xbox.getAnalogHat(RightHatY));
      packet[4]=convertThrottle(Xbox.getButtonPress(R2));
      packet[5]=channelSelectSwitch;
      packet[6]=convertButton(Xbox.getButtonPress(L1));
      packet[7]=convertButton(Xbox.getButtonPress(R1));
      packet[8]=convertButton(Xbox.getButtonPress(B));
      packet[9]=convertButton(Xbox.getButtonPress(Y));
      if (debug==true){
        j++;
        if (j==10){
          j=0;
        
          for (uint8_t i=0;i<numOfChannels;i++){
            Serial.print(packet[i]);
            Serial.print(",");
          }
          Serial.println();
        }
      }
      comms.sendPacket(numOfChannels,packet);
    }
  }
}

uint16_t convert(int in){
  return (uint16_t) ((float)in/32768.0*3200+12000);
}
uint16_t convertThrottle(uint8_t in){
  return (uint16_t) ((float)in/255*6400+8800);
}
uint16_t convertButton(bool in){
  if (in){
    return 15200;
  }
  else
    return 8800;
}
uint16_t convertPercent(uint8_t in){
  return 8800+in*64;
}

