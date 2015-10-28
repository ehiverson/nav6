#include <Servo.h>
#include <i2c_t3.h>
#include <log.h>
#include <imun_motor.h>
uint8_t pwmpin1=6;
uint8_t pwmpin2=5;
uint8_t pwmpin3=4;
uint8_t pwmpin4=3;

ImunMotor motor1(pwmpin1);
ImunMotor motor2(pwmpin2);
ImunMotor motor3(pwmpin3);
ImunMotor motor4(pwmpin4);
ImunQuadMotor motors(motor1,motor2,motor3,motor4);

String e;
void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);


  motor1.setThrottle(0);
  motor2.setThrottle(0);
  motor3.setThrottle(0);
  motor4.setThrottle(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available()>0){
    e=Serial1.readStringUntil('/');
    if (e.substring(0,1)=="1")
    {
      motor1.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="2")
    {
      motor2.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="3")
    {
      motor3.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="4")
    {
      motor4.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=="a")
    {
      motor1.setThrottle(e.substring(1).toInt()/100.f);
      motor2.setThrottle(e.substring(1).toInt()/100.f);
      motor3.setThrottle(e.substring(1).toInt()/100.f);
      motor4.setThrottle(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='y')
    {
      motors.setYaw(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='t')
    {
      motors.setThrust(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='r')
    {
      motors.setRoll(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='p')
    {
      motors.setPitch(e.substring(1).toInt()/100.f);
    }
    else if (e.substring(0,1)=='k')
    {
      motors.killMotors();
    }
  }
}


