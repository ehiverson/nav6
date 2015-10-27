#include <Servo.h>
#include <i2c_t3.h>
#include <log.h>
#include <imun_motor.h>
uint8_t pwmpin1=6;
uint8_t pwmpin2=5;
uint8_t pwmpin3=4;
uint8_t pwmpin4=3;

ImunMotor nmotor1;
ImunMotor nmotor2;
ImunMotor nmotor3;
ImunMotor nmotor4;
ImunQuadMotor motors;

String e;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nmotor1.init(pwmpin1);
  nmotor2.init(pwmpin2);
  nmotor3.init(pwmpin3);
  nmotor4.init(pwmpin4);
  motors.init(nmotor1,nmotor2,nmotor3,nmotor4);
  nmotor1.setThrottle(0);
  nmotor2.setThrottle(0);
  nmotor3.setThrottle(0);
  nmotor4.setThrottle(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    e=Serial.readStringUntil('/');
    if (e.substring(0,1)=="1")
    {
      nmotor1.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="2")
    {
      nmotor2.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="3")
    {
      nmotor3.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="4")
    {
      nmotor4.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="a")
    {
      nmotor1.setThrottle(e.substring(1).toInt()/100.f);
      nmotor2.setThrottle(e.substring(1).toInt()/100.f);
      nmotor3.setThrottle(e.substring(1).toInt()/100.f);
      nmotor4.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='y')
    {
      motors.setYaw(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='t')
    {
      motors.setThrust(e.substring(1).toInt()/100.f);
    }
  }
}


