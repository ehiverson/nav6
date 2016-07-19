#include <sumd.h> 
SUMDCommunicator sum(Serial1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(700);
}

elapsedMillis t;

void loop() {
  // put your main code here, to run repeatedly:
  if (t>=10){
    t=0;
    uint16_t p[]={12000,14000,8800,10000,11000,11000,11000,11000,11000,11000,11000,11000};
    uint8_t numOC=12;
    sum.sendPacket(numOC,p);}
}
  

