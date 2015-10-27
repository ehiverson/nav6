//Includes
//{
#define EMPL_TARGET_ATMEGA328
#include "i2c_t3.h"
#include "Servo.h"
#include <imun_header.h>


extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}



#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif


// MPU Calibration
ImunComms comms(Serial1);

//Magnetometer State
VectorFloat rmagvec,magvec,magvec2;
//Alpha controls the lowpass filtering of the magnetometer. a higher value will result in faster but shakier magnetometer data.
const float alpha=0.02;
float yawradians=0;
//Magnetometer Calibration
static float magCalMatrix[9] = 
{ 
 0.877478, -0.014416, -0.06554,
 -0.014416,  0.836852, -0.056047,
 -0.06554,  -0.056047,  0.529377,
};
 static float magCalOffsets[3]={0.2956307617,  0.4688498635,  0.8140717455};

//Gyro/Accel/DMP State
float temp_centigrade = 0.0;  // Gyro/Accel die temperature
long curr_mpu_temp;
unsigned long sensor_timestamp,sensor_timestamp2, interv;
short compass_data[3];
elapsedMillis elapsed_since_compass=0;
elapsedMillis outputtimer=0;
unsigned int compass_measurement_period=125; //milliseconds



VectorFloat zaxis0(0,0,1);
VectorFloat zaxis(0,0,1);
VectorFloat a(0,0,0);
VectorFloat g(0,0,0);
VectorFloat v(0,0,0);
VectorFloat acal(0,0,0);
Quaternion q,qmag,fusedq,qout;

//Gyro/Accel/DMP Configuration
unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000
unsigned short compass_fsr;
long accel_biases[3]={0,-600,725};//568};
// The mounting matrix below tells the MPL how to rotate the raw data from the driver(s). The matrix below reflects the axis orientation of the MPU-6050 on the247 nav6 circuit board.
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };

//HAL
short gyro[3], accel[3], sensors;
unsigned char more = 0;
long quat[4];

void setup() {
	teensyInit();
	Serial.println(F("Nick IMU firmware"));


	// MPU-6050 Initialization

	// Initialize the MPU:
	//
	// Gyro sensitivity:      2000 degrees/sec
	// Accel sensitivity:     2 g
	// Gyro Low-pass filter:  42Hz
	// DMP Update rate:       100Hz

	Serial.print(F("Initializing MPU..."));
	Serial.flush();
	boolean mpu_initialized = false;
	while ( !mpu_initialized ) {
		digitalWrite(STATUS_LED, HIGH);
		if ( hal_initialize_mpu(gyro_orientation,dmp_update_rate,gyro_fsr,accel_fsr,compass_fsr) ) {
			mpu_initialized = true;
			Serial.print(F("Success"));
			mpu_set_accel_bias(accel_biases);
			hal_enable_mpu();
		}
		else {
			digitalWrite(STATUS_LED, LOW);
			Serial.print(F("Failed"));
			mpu_force_reset();
			delay(100);
			Serial.println(F("Re-initializing"));
			Serial.flush();
		}
	}
	Serial.println();
	Serial.println(F("Initialization Complete"));
	Serial.flush();
 
	digitalWrite(STATUS_LED, LOW);
}

bool firstmagloop=true;
uint8_t i=0;
void loop() {
	// Read compass heading data if it has been updated recently.
	if ( elapsed_since_compass>compass_measurement_period )
	{
		//Get raw magnetometer data and store it as a vector.
		mpu_get_compass_reg(compass_data,NULL);
		//Note that the x and y values are switched and the z value is negative. This is becuase the magnetometer has a different orientation than the accelerometer and gyro.
		rmagvec.init(compass_data[1],compass_data[0],-compass_data[2]);
		rmagvec=rmagvec.multiply(0.01);
		//Calibrate raw magnetometer data based on previously determined calibration values.
		magvec=ellipseTransform(rmagvec,magCalMatrix,magCalOffsets);
		//Get the temperature and convert to centigrade
		mpu_get_temperature(&curr_mpu_temp, &sensor_timestamp);
		temp_centigrade = (float)curr_mpu_temp/65536.0;
		//Math operations
		
		magvec=magvec.rotate(q);
    
		//Lowpass filter of rotated magnetic vector for stability. Via exponential smoothing.
		if (firstmagloop)
		{
			magvec2=magvec;
			firstmagloop=false;
		}
		magvec=magvec.multiply(alpha).add(magvec2.multiply(1-alpha));
		magvec2=magvec;
		
		//Calculate quaternion form magnetometer data.
		yawradians=atan2(magvec.x,magvec.y);
		qmag=qfromYaw(zaxis0,yawradians);		
    
		//Reset magnetometer reading timer.
		elapsed_since_compass=0;
	}

	// If the MPU Interrupt occurred, read the fifo and process the data
	if (hal.new_gyro && hal.dmp_on)
	{
		/* This function gets new data from the FIFO when the DMP is in
		 * use. The FIFO can contain any combination of gyro, accel,
		 * quaternion, and gesture data. The sensors parameter tells the
		 * caller which data fields were actually populated with new data.
		 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
		 * the FIFO isn't being filled with accel data.
		 * The driver parses the gesture data to determine if a gesture
		 * event has occurred; on an event, the application will be notified
		 * via a callback (assuming that a callback function was properly
		 * registered). The more parameter is non-zero if there are
		 * leftover packets in the FIFO.
		 */
		int success = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);
		if (!more)
		  hal.new_gyro = 0;
		if (( success == 0 ) and ((sensors & INV_XYZ_ACCEL)) and ((sensors & INV_WXYZ_QUAT))) 
		{
			q.init( (float)((quat[0] >> 16)/16384.0f),(float)((quat[1] >> 16)/16384.0f) ,(float)((quat[2] >> 16)/16384.0f) ,(float)((quat[3] >> 16)/16384.0f) );
			a.init((float)(accel[0]/16384.0f),(float)(accel[1]/16384.0f),(float)(accel[2]/16384.0f));
      g.init((float)(gyro[0]/16384.0f),(float)(gyro[1]/16384.0f),(float)(gyro[2]/16384.0f));
			//Math processing
      mathprocess();
      qout=fusedq;
      interv=sensor_timestamp-sensor_timestamp2;
      sensor_timestamp2=sensor_timestamp;
		}	
	}
	//Serial Output
	if (outputtimer>20){
		serialout();
		outputtimer=0;
	}
}
void mathprocess()
{
    //Math processing
    //Generate a quaternion from a fusion of the magnetometer and gyro data.
    fusedq=qmag.multiply(q);  
    a=a.rotate(fusedq);
    //Subtract gravity
    a=a.subtract(VectorFloat(0,0,1));
    v=v.add(a.multiply(interv/1000.0f));
    
}
void serialout(){
	// Update client with quaternions and some raw sensor data
  comms._serial.write('+');

  comms._serial.write((char)40);
  comms.transmitBytes(q.w);
  comms.transmitBytes(q.x);
  comms.transmitBytes(q.y);
  comms.transmitBytes(q.z);
  comms.transmitBytes(magvec.x);
  comms.transmitBytes(magvec.y);
  comms.transmitBytes(magvec.z);
  comms.transmitBytes(a.x);
  comms.transmitBytes(a.y);
  comms.transmitBytes(a.z);



}

