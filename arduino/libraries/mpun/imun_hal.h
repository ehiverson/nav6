
#ifndef _HALN
#define _HALN

#define SDA_PIN A4
#define SCL_PIN A5
#define STATUS_LED 13

extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}
/***************************************
* Invensense Hardware Abstracation Layer
***************************************/

struct hal_s {
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned char motion_int_mode;
	unsigned long next_temp_ms;
    unsigned long next_compass_ms;
	unsigned short report;
	unsigned short dmp_features;
};

static struct hal_s hal = { 0 };

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ    (100)
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0,-1}
};



void gyro_data_ready_cb(void) {

	hal.new_gyro = 1;
}

boolean hal_initialize_mpu(signed char gyro_orientation[9],unsigned short &dmp_update_rate, unsigned short &gyro_fsr, unsigned char &accel_fsr, unsigned short &compass_fsr){
	int result;
	struct int_param_s int_param;

	/* Set up gyro.
	* Every function preceded by mpu_ is a driver function and can be found
	* in inv_mpu.h.
	*/
	int_param.cb = gyro_data_ready_cb;
	int_param.pin = 14;
	result = mpu_init(&int_param);

	if (result != 0) {
		Serial.print("mpu_init failed!");
		return false;
	}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo(INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);
	
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&dmp_update_rate);
	mpu_get_gyro_fsr(&gyro_fsr);	
	mpu_get_accel_fsr(&accel_fsr);
	mpu_get_compass_fsr(&compass_fsr);

	/* Initialize HAL state variables. */
	memset(&hal, 0, sizeof(hal));
	hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
	hal.report = PRINT_QUAT;
	hal.dmp_on = 0;
    hal.report = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;
	/* To initialize the DMP:
	* 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
	*    inv_mpu_dmp_motion_driver.h into the MPU memory.
	* 2. Push the gyro and accel orientation matrix to the DMP.
	* 3. Register gesture callbacks. Don't worry, these callbacks won't be
	*    executed unless the corresponding feature is enabled.
	* 4. Call dmp_enable_feature(mask) to enable different features.
	* 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	* 6. Call any feature-specific control functions.
	*
	* To enable the DMP, just call mpu_set_dmp_state(1). This function can
	* be called repeatedly to enable and disable the DMP at runtime.
	*
	* The following is a short summary of the features supported in the DMP
	* image provided in inv_mpu_dmp_motion_driver.c:
	* DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	* 200Hz. Integrating the gyro data at higher rates reduces numerical
	* errors (compared to integration on the MCU at a lower sampling rate).
	* DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
	* 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
	* DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	* DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
	* an event at the four orientations where the screen should rotate.
	* DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
	* no motion.
	* DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
	* DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
	* DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
	* be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
	*/
	result = dmp_load_motion_driver_firmware();
	if (result != 0) {
		Serial.print("Firmware Load ERROR ");
		Serial.println(result);
		return false;
	}
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));

	unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
		DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	return true;
}
void hal_enable_mpu(){
	mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
	hal.dmp_on = 1;
}
void hal_disable_mpu() {
	mpu_set_dmp_state(0);
	hal.dmp_on = 0;
}


boolean hal_run_gyro_self_test() {
	int result;
	long gyro[3], accel[3];
	boolean gyro_ok = false;
	result = mpu_run_self_test(gyro, accel);
	
	if ((result & 0x1) != 0) {
		// Gyro passed self test
		gyro_ok = true;
		float sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
	}

	return gyro_ok;
}
void hal_run_accel_self_test(long *biases){
	short accel[3];
	long acceltotal[3]={0,0,0};
	for (uint8_t i=0;i<255;i++)
	{
		mpu_get_accel_reg(accel,NULL);
		for (uint8_t j=0;j<3;j++) acceltotal[j]+= accel[j];
	}
	for (uint8_t j=0;j<3;j++) acceltotal[j]/=255;
	unsigned short sens;
	mpu_get_accel_sens(&sens);
	acceltotal[2]-=sens;
	for (uint8_t i=0;i<3;i++) {acceltotal[i]*=-4;biases[i]=acceltotal[i];}//The four here is based on experiment I don't know why it works.
	mpu_set_accel_bias(acceltotal);
	
}
void teensyI2cInit(){
	// reset I2C bus
	// This ensures that if the nav6 was reset, but the devices
	// on the I2C bus were not, that any transfer in progress do
	// not hang the device/bus.  Since the MPU-6050 has a 1024-byte
	// fifo, and there are 8 bits/byte, 10,000 clock edges
	// are generated to ensure the fifo is completely cleared
	// in the worst case.

	pinMode(SDA_PIN, INPUT);
	pinMode(SCL_PIN, OUTPUT);
	pinMode(STATUS_LED, OUTPUT);

	digitalWrite(STATUS_LED, LOW);
	// Clock through up to 1000 bits
	int x = 0;
	for ( int i = 0; i < 10000; i++ ) {

		digitalWrite(SCL_PIN, HIGH);
		digitalWrite(SCL_PIN, LOW);
		digitalWrite(SCL_PIN, HIGH);

		x++;
		if ( x == 8 ) {
		  x = 0;
		  // send a I2C stop signal
		  digitalWrite(SDA_PIN, HIGH);
		  digitalWrite(SDA_PIN, LOW);
		}
	}

  // send a I2C stop signal
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);

  // join I2C bus
  Wire.begin();

}
void teensyInit(){
	Serial.begin(57600);
	Serial1.begin(115200);
	pinMode(21,OUTPUT);
	pinMode(20,OUTPUT);
	pinMode(15,OUTPUT);
	digitalWrite(21,HIGH);
	digitalWrite(20,LOW);
	digitalWrite(15,LOW);
	pinMode(14,INPUT);
	teensyI2cInit();
}

	

#endif	// _HALN 