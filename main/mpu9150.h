#ifndef MPU9150_HRIVER_H
#define MPU9150_HRIVER_H

#include "esp_err.h"
#include <stdint.h>


#define I2C_MASTER_TIMEOUT_MS 1000
#define MPU9150_PWR_MGMT_1_REG_ADDR                                            \
  0x6B /*!< Register addresses of the power managment register */
#define MPU9150_RESET_BIT 7 /* !< Power reset register */
#define MPU9150_WAKEUP 0x00 /* !< Wake up device */
#define MPU9150_ACCEL_OUT 0x3B /* !< Acceleratior registers start address [6 bytes] */

typedef struct {
  uint8_t addr;
  uint8_t i2c_num;
} I2CMASTER_DEV_T;

  void MPU9150setAddrPort(I2CMASTER_DEV_T *dev, uint8_t addr, uint8_t i2c_num);
  esp_err_t MPU9150initialize(I2CMASTER_DEV_T dev);
  esp_err_t MPU9150reset(I2CMASTER_DEV_T dev);
  esp_err_t MPU9150acceleration(I2CMASTER_DEV_T dev, uint16_t* x, uint16_t* y, uint16_t* z);



#endif /* MPU9150_HRIVER_H */
