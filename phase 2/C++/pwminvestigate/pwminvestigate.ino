#include <Servo.h>
#include <i2c_t3.h>

uint8_t pwmpin=6;
String e;
//Servo serv;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //serv.attach(pwmpin);
  pinMode(6,INPUT);
}

void loop() {
  delay(10);
  Serial.println(pulseIn(pwmpin,HIGH));
}


