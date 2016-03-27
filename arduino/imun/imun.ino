//Includes
//{
#define EMPL_TARGET_ATMEGA328
#include "Servo.h"
#include "i2c_t3.h"
#include <imun_header.h>
 
 extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

//Define comm port
ImunComms comms(Serial1);


//Timing
elapsedMillis elapsed_since_compass=0;
elapsedMillis outputtimer=0;
unsigned int compass_measurement_period=125; //milliseconds


ImunImu imu;

void setup() {
	teensyInit();
	Serial.println(F("Nick IMU firmware"));
  digitalWrite(STATUS_LED, HIGH);
	Serial.print(F("Initializing MPU..."));
	Serial.flush();
	Serial.println();
  imu.initialize();
	Serial.println(F("Initialization Complete"));
	Serial.flush();
	digitalWrite(STATUS_LED, LOW);
}


void loop() 
{
	// Read compass heading data if it has been updated recently.
	if ( elapsed_since_compass>compass_measurement_period )
	{
    imu.updateCompass();
		//Reset magnetometer reading timer.
		elapsed_since_compass=0;
	}
  imu.updateQuat();
	//Serial Output
	if (outputtimer>50)
  {
		serialout();
		outputtimer=0;
	}
}

void serialout(){
	// Update client with quaternions and some raw sensor data
  if (comms.stream)
  {
    comms.transmitDataPacket(imu.qout,imu.magvec,imu.a,imu.rmagvec);
  }
  comms.receiveCommands(imu.q,&imu.qorientation);
}

