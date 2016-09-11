#include <Servo.h>
#include <i2c_t3.h>

uint8_t pins[4]={3,4,5,6};
String e;
Servo serv1;
Servo serv2;
Servo serv3;
Servo serv4;
Servo servs[4]={serv1,serv2,serv3,serv4};
int high=2000;
int low=1000;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //serv.attach(pwmpin);
  for (uint8_t i=0;i<4;i++){
    servs[i].attach(pins[i]);
  }
  for (uint8_t i=0;i<4;i++){
    servs[i].writeMicroseconds(low);
  }
}

void loop() {
  if (Serial.available()>0){
    e=Serial.readStringUntil('/');
    if (e=="h"){
      for (uint8_t i=0;i<4;i++){
        servs[i].writeMicroseconds(high);
      }
    }
    else if (e=="l"){
      for (uint8_t i=0;i<4;i++){
        servs[i].writeMicroseconds(low);
      }
    }
    else{
      for (uint8_t i=0;i<4;i++){
        servs[i].writeMicroseconds(e.toInt());
      }
    }
  }
}


