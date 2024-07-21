// =========================================================================
// Copyright 2024 Damian Pablo Machtey. All rights reserved.
// =========================================================================

#ifndef ALARM_MPU9150_H
#define ALARM_MPU9150_H

#include "esp_err.h"
#include <math.h>
#include <stdint.h>

#define I2C_MASTER_SCL_IO 19        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18        /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define MPU9150_SENSOR_ADDRESS 0x68 /*!< address for MPU9150 */

/**
 * Initialize alarm
 *
 * @param threshold [0.01 - 400.00] sensibility
 *
 * @return esp_err_t error codes
 */
esp_err_t alarmInit(float_t threshold);

/**
 * Check alarm status, need to be polled continuous
 *
 *
 * @return >0 if triggered
 */
uint8_t alarmStatus(void);

/**
 * Reset the alarm
 *
 */
void alarmReset(void);

#endif // ALARM_MPU9150_H
