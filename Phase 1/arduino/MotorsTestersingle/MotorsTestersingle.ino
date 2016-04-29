#include <Servo.h>
#include <i2c_t3.h>
#include <log.h>
#include <imun_motor.h>
uint8_t pwmpin1=6;
uint8_t pwmpin2=5;
uint8_t pwmpin3=4;
uint8_t pwmpin4=3;

ImunMotor nmotor1(pwmpin1);


String e;
void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);


  nmotor1.setThrottle(0);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available()>0){
    e=Serial1.readStringUntil('/');
    Serial1.println(e);
    if (e.substring(0,1)=="1")
    {
      nmotor1.setThrottle(e.substring(1).toInt()/100.f);
    }

  }
}


