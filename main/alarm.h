#ifndef ALARM_MPU9150_H
#define ALARM_MPU9150_H

#include <stdint.h>


#define I2C_MASTER_SCL_IO 19 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18 /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_ADDRESS 0x40 /*!< address for MPU9150 */


void alrmInit(uint8_t threshold);
uint8_t alarmStatus(void);
void alarmReset(void);


#endif // ALARM_MPU9150_H
