#include "Arduino.h"
#include "teensy_port.h"
#include "i2c_t3.h"

//#define I2CDEV_SERIAL_DEBUG

volatile bool i2c_t3_started = false;
uint32_t timeout = 1000;

void start_i2c(void)
{
	Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	i2c_t3_started = true;
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	if(!i2c_t3_started)
		start_i2c();

	#ifdef I2CDEV_SERIAL_DEBUG
		Serial.print("I2C (0x");
		Serial.print(slave_addr, HEX);
		Serial.print(") writing ");
		Serial.print(length, DEC);
		Serial.print(" bytes to 0x");
		Serial.print(reg_addr, HEX);
		Serial.print("...");
	#endif

    int ret;

	Wire.beginTransmission(slave_addr);
	Wire.write(reg_addr);

	for (uint8_t i = 0; i < length; i++)
	{
		Wire.write(data[i]);
		#ifdef I2CDEV_SERIAL_DEBUG
			Serial.print(data[i], HEX);
			if (i + 1 < length) Serial.print(" ");
		#endif
    }

	ret = Wire.endTransmission(I2C_STOP);

	#ifdef I2CDEV_SERIAL_DEBUG
		Serial.println(". Done.");
	#endif

	return ret;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
	if(!i2c_t3_started)
		start_i2c();

	#ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(slave_addr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(reg_addr, HEX);
        Serial.print("...");
    #endif

	int8_t count = 0;
	uint32_t t1 = millis();

	for (uint8_t k = 0; k < length; k += min(length, I2C_RX_BUFFER_LENGTH))
	{
		Wire.beginTransmission(slave_addr);
		Wire.write(reg_addr);
		Wire.endTransmission();
		Wire.beginTransmission(slave_addr);
		Wire.requestFrom(slave_addr, (uint8_t)min(length - k, I2C_RX_BUFFER_LENGTH), I2C_NOSTOP, timeout);

		for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++)
		{
			data[count] = Wire.read();
			#ifdef I2CDEV_SERIAL_DEBUG
				Serial.print(data[count], HEX);
				if (count + 1 < length) Serial.print(" ");
			#endif
		}

		Wire.endTransmission();
	}

	#ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count == length ? 0 : 1;
}

void delay_ms(unsigned long ms)
{
	delay(ms);
}

void get_ms(unsigned long *ms)
{
	*ms = millis();
}

