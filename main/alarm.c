#include "alarm.h"
#include "mpu9150.h"
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <math.h>
#include <stdint.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */

uint8_t master_initialized = 0;
I2CMASTER_DEV_T i2c_master;
float_t threshold;
uint8_t status = 0;

esp_err_t i2c_master_init(I2CMASTER_DEV_T dev);

esp_err_t alarmInit(float_t threshold) {
  esp_err_t err = 0;
  if (!master_initialized) {
    MPU9150setAddrPort(&i2c_master, MPU9150_SENSOR_ADDRESS, I2C_NUM_0);
    err = i2c_master_init(i2c_master);
    if (err)
      return err;
    master_initialized = 1;
    err = MPU9150reset(i2c_master);
    if (err)
      return err;
    err = MPU9150initialize(i2c_master);
    if (err)
      return err;
  }
  threshold = threshold;
  return err;
}

uint8_t alarmStatus(void) {
  static float_t accel_vector_old = 0.0;
  uint16_t x, y, z;
  MPU9150acceleration(i2c_master, &x, &y, &z);
  float_t accel_vector = sqrt(x * x + y * y + z * z);
  float_t accel_dif = fabs(accel_vector_old - accel_vector);
  if ((accel_vector_old != 0.0) && ((accel_dif / accel_vector) > threshold))
    status = 1;

  accel_vector_old = accel_vector;
  return status;
}

void alarmReset(void) { status = 0; }

esp_err_t i2c_master_init(I2CMASTER_DEV_T dev) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  int i2c_master_port = dev.i2c_num;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}
