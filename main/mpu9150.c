// =========================================================================
// Copyright 2024 Damian Pablo Machtey. All rights reserved.
// =========================================================================

#include "mpu9150.h"
#include <driver/i2c.h>
#include <math.h>
#include <stdint.h>

/**
 * Write a byte into a device register
 *
 * @param dev I2CMASTER_DEV_T holding the master port and slave address info
 * @param reg_addr
 * @param data
 *
 * @return esp_err_t error codes
 */
static inline esp_err_t MPU9150_register_write_byte(I2CMASTER_DEV_T dev, uint8_t reg_addr,
                                      uint8_t data);

/**
 *  Read len bytes from a device register
 *
 * @param dev I2CMASTER_DEV_T holding the master port and slave address info
 * @param reg_addr
 * @param data
 * @param len
 *
 * @return esp_err_t error codes
 */
static inline esp_err_t MPU9150_register_read(I2CMASTER_DEV_T dev, uint8_t reg_addr,
                                uint8_t *data, size_t len);

void MPU9150setAddrPort(I2CMASTER_DEV_T *dev, uint8_t addr, uint8_t i2c_num) {
  dev->addr = addr;
  dev->i2c_num = i2c_num;
}

esp_err_t MPU9150initialize(I2CMASTER_DEV_T dev) {
  return MPU9150_register_write_byte(dev, MPU9150_PWR_MGMT_1_REG_ADDR,
                                     MPU9150_WAKEUP);
}

esp_err_t MPU9150reset(I2CMASTER_DEV_T dev) {
  return MPU9150_register_write_byte(dev, MPU9150_PWR_MGMT_1_REG_ADDR,
                                     1 << MPU9150_RESET_BIT);
}

esp_err_t MPU9150acceleration(I2CMASTER_DEV_T dev, uint16_t *x, uint16_t *y,
                              uint16_t *z) {
  esp_err_t err = 0;
  uint8_t data[6];
  err = MPU9150_register_read(dev, 0x3B, data, 6);
  *x = (data[0] << 8) | data[1];
  *y = (data[2] << 8) | data[3];
  *z = (data[4] << 8) | data[5];
  return err;
}

static inline esp_err_t MPU9150_register_read(I2CMASTER_DEV_T dev, uint8_t reg_addr,
                                uint8_t *data, size_t len) {
  return i2c_master_write_read_device(
      dev.i2c_num, dev.addr, &reg_addr, 1, data, len,
      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static inline esp_err_t MPU9150_register_write_byte(I2CMASTER_DEV_T dev, uint8_t reg_addr,
                                      uint8_t data) {
  int ret;
  uint8_t write_buf[2] = {reg_addr, data};

  ret = i2c_master_write_to_device(dev.i2c_num, dev.addr, write_buf,
                                   sizeof(write_buf),
                                   I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

  return ret;
}
