#ifndef ALARM_MPU9150_H
#define ALARM_MPU9150_H

#include "esp_err.h"
#include <math.h>
#include <stdint.h>


#define I2C_MASTER_SCL_IO 19 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18 /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define MPU9150_SENSOR_ADDRESS 0x68 /*!< address for MPU9150 */


esp_err_t  alarmInit(float_t threshold);
uint8_t alarmStatus(void);
void alarmReset(void);


#endif // ALARM_MPU9150_H
