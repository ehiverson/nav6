#define EMPL_TARGET_ATMEGA328
#include <i2c_t3.h>
#include <helper_3dmathn.h>
#include <hal.h>
extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#define SDA_PIN A4
#define SCL_PIN A5
#define STATUS_LED 13

//Gyro/Accel/DMP Configuration
unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000
unsigned short compass_fsr;
// The mounting matrix below tells the MPL how to rotate the raw data from the driver(s). The matrix below reflects the axis orientation of the MPU-6050 on the247 nav6 circuit board.
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };

long biases[3];
elapsedMillis outputtimer=0;
VectorFloat a(0,0,0);
VectorFloat v(0,0,0);
short gyro[3], accel[3], sensors;
unsigned char more = 0;
long quat[4];
unsigned long sensor_timestamp;

void setup() {
  // put your setup code here, to run once:
	teensy_init();
  Serial.println(F("Nick IMU accel calibration"));


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
      //hal_run_accel_self_test(biases);
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
  Serial.println(F("Initialization Complete. Accel biases are:"));
  for (uint8_t i=0;i<3;i++) Serial.println(biases[i]);
  Serial.flush();
 
  digitalWrite(STATUS_LED, LOW);
}

void loop() {
	// put your main code here, to run repeatedly:
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
			a.init((float)(accel[0]),(float)(accel[1]),(float)(accel[2]));
			a=a.subtract(VectorFloat(0,0,16384));
			//v=v.add(a);
			v=a;
		}	
	}
	//Serial Output
	if (outputtimer>20){
		Serial.print(v.x);
		Serial.print(',');
		Serial.print(v.y);
		Serial.print(',');
		Serial.print(v.z);
		Serial.print(',');
		Serial.println(v.magnitude());
		outputtimer=0;
	}

}
