#define EMPL_TARGET_ATMEGA328
#include "i2c_t3.h"
#include "Servo.h"
#include <imun_header.h>
extern "C" {

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}



ImunComms comms(Serial1,9600);
void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  comms.transmitBytes(0.1);
}
