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

String e;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nmotor1.init(pwmpin1);
  nmotor2.init(pwmpin2);
  nmotor3.init(pwmpin3);
  nmotor4.init(pwmpin4);
  nmotor1.setThrottle(0);
  nmotor2.setThrottle(0);
  nmotor3.setThrottle(0);
  nmotor4.setThrottle(0);
  delay(5000);
  for (uint8_t i=1;i<8;i++)
  {
    setAll(i*100);
    delay(3000);
  }
  for (uint8_t i=0;i<4;i++)
  {
    setAll(600-i*100);
    delay(3000);
  }
  setAll(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  

}

void setAll(int throttle)
{
      nmotor1.setThrottle(throttle);
      nmotor2.setThrottle(throttle);
      nmotor3.setThrottle(throttle);
      nmotor4.setThrottle(throttle); 
}

