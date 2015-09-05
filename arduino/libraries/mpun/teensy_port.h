#include <stdbool.h>

#include "log.h"
#include "inv_mpu.h"

extern volatile bool i2c_t3_started;

#define log_i       MPL_LOGI
#define log_e       MPL_LOGE

#define __no_operation() __asm("nop\n\t")

#ifndef min
	#define min(a,b) ((a<b)?a:b)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

void delay_ms(unsigned long ms);
void get_ms(unsigned long *ms);


#ifdef __cplusplus
}
#endif
