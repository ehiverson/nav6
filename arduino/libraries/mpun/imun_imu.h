
#ifndef _IMUNIMU_H
#define _IMUNIMU_H




extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

class ImunImu
{
	public:
		ImunImu();
		void initialize();
		void updateCompass();
		void updateQuat();
		
		VectorFloat zaxis0;
		VectorFloat zaxis;
		VectorFloat a;
		VectorFloat g;
		VectorFloat v;
		VectorFloat acal;
		Quaternion q,qmag,fusedq,qout,qorientation;
		float yawradians=0;
		VectorFloat rmagvec,magvec,magvec2;
		
		//Gyro/Accel/DMP Configuration
		unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
		unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
		unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000
		unsigned short compass_fsr;
		
		//CALIBRATION AND USER SETTABLE VALUES
		//Alpha controls the lowpass filtering of the magnetometer. a higher value will result in faster but shakier magnetometer data.
		const float alpha=0.02;

		//Magnetometer Calibration
		//This adjusts for elliptical magnetometer readings
		float magCalMatrix[9] = 
		{ 
		 0.54211957, -0.00700604, -0.00089606,
		-0.00700604,  0.53088298, -0.02684748,
		 -0.00089606, -0.02684748,  0.47934341
		};
		//This adjusts for off-center magnetometer readings
		float magCalOffsets[3]={ 0.01663884,  0.03021143,  1.27465444};

		short compass_data[3];

		long accel_biases[3]={0,-600,725};

		//Temperature
		float temp_centigrade = 22;  // Gyro/Accel die temperature, initial setting shouldn't matter
		long curr_mpu_temp;

		unsigned long sensor_timestamp,sensor_timestamp2, interv;
		
		bool firstmagloop=true;
		
		//HAL
		short gyro[3], accel[3], sensors;
		unsigned char more = 0;
		long quat[4];

	private:
		void mathprocess();
};

ImunImu::ImunImu()
{
}

void ImunImu::initialize()
{
	zaxis0.init(0,0,1);
	zaxis.init(0,0,1);
	a.init(0,0,0);
	g.init(0,0,0);
	v.init(0,0,0);
	acal.init(0,0,0);
	bool mpu_initialized = false;
	while ( !mpu_initialized )
	{
		if ( hal_initialize_mpu(dmp_update_rate,gyro_fsr,accel_fsr,compass_fsr) ) 
		{
			mpu_initialized = true;
			mpu_set_accel_bias(accel_biases);
			hal_enable_mpu();
		}
		else 
		{
			mpu_force_reset();
			delay(50);
		}
	}
}

void ImunImu::updateCompass()
{
	//Get raw magnetometer data and store it as a vector.
	mpu_get_compass_reg(compass_data,NULL);
	rmagvec.init(compass_data[1],compass_data[0],-compass_data[2]);     //Note that the x and y values are switched and the z value is negative. This is becuase the magnetometer has a different orientation than the accelerometer and gyro.
	rmagvec=rmagvec.multiply(0.01);
	
	//Calibrate raw magnetometer data based on previously determined calibration values.
	magvec=ellipseTransform(rmagvec,magCalMatrix,magCalOffsets);
	
	//Get the temperature and convert to centigrade
	mpu_get_temperature(&curr_mpu_temp, &sensor_timestamp);
	temp_centigrade = (float)curr_mpu_temp/65536.0;
	
	//Math operations
	magvec=magvec.rotate(q); //rotate the magnetic vector to world frame of reference

	//Lowpass filter of rotated magnetic vector for stability. Via exponential smoothing.
	if (firstmagloop){magvec2=magvec;firstmagloop=false;}
	magvec=magvec.multiply(alpha).add(magvec2.multiply(1-alpha));
	magvec2=magvec;
	
	//Calculate yaw quaternion from magnetometer data.
	yawradians=atan2(magvec.x,magvec.y);
	qmag=qfromYaw(zaxis0,yawradians);
}

void ImunImu::updateQuat()
{
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
			q.init((float)((quat[0] >> 16)/16384.0f),(float)((quat[1] >> 16)/16384.0f) ,(float)((quat[2] >> 16)/16384.0f) ,(float)((quat[3] >> 16)/16384.0f) );
			a.init((float)(accel[0]/16384.0f),(float)(accel[1]/16384.0f),(float)(accel[2]/16384.0f));
			g.init((float)(gyro[0]/16384.0f),(float)(gyro[1]/16384.0f),(float)(gyro[2]/16384.0f));
			//Math processing
			mathprocess();
			qout=fusedq;
			interv=sensor_timestamp-sensor_timestamp2;
			sensor_timestamp2=sensor_timestamp;
		}	
	}
}
void ImunImu::mathprocess()
{
    //Math processing
    //Generate a quaternion from a fusion of the magnetometer and gyro data.
    fusedq=qmag.multiply(q);
    a=a.rotate(fusedq);
    fusedq=fusedq.multiply(qorientation.conjugate()); //This compensates for uneven mountinig of the IMU on the drone. when the flatten command is sent the drone is assumed to be placed on an even surface and facing magnetic north Later this qorientation should be loaded from and saved to eeprom. It is important that this step in performed after rotation of vectors.
    //Subtract gravity
    a=a.subtract(VectorFloat(0,0,1));
    v=v.add(a.multiply(interv/1000.0f));
    
}

#endif				/* _IMUNIMU_H */

