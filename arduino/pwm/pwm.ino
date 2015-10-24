#include <Servo.h>
#include <i2c_t3.h>
#include <log.h>
#include <imun_motor.h>
uint8_t pwmpin=23;

ImunMotor nmotor;
String e;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nmotor.init(pwmpin);
  //nmotor.escCalibration();
  nmotor.setThrottle(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    e=Serial.readStringUntil('/');
    if ((e.toInt()<=1000) and (e.toInt()>=0)){Serial.println(e); nmotor.setThrottle(e.toInt());}
    //if (e=="1") nmotor.setThrottle(1000);
    //else if (e=="0") nmotor.setThrottle(0);
  }
}
