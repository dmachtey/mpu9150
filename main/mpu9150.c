#include "mpu9150.h"
#include <driver/i2c.h>
#include <math.h>
#include <stdint.h>

void MPU9150setAddrPort(uint8_t addr, i2c_port_t i2c_num);
esp_err_t MPU9150initialize();
esp_err_t MPU9150reset();

esp_err_t MPU9150setSampleRate(uint16_t rate);
esp_err_t MPU9150setClockSource(clock_src_t clockSrc);
esp_err_t MPU9150setDigitalLowPassFilter(dlpf_t dlpf);

esp_err_t MPU9150setAccelFullScale(accel_fs_t fsr);

esp_err_t MPU9150acceleration(int16_t* x, int16_t* y, int16_t* z);

/**
 * Read 2 bytes from i2c register
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param regaddr register to read from
 * @param valueA
 * @param valueB
 *
 * @return esp_err_t error codes
 */
esp_err_t generic_read_two_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                        uint8_t *valueA, uint8_t *valueB);

/**
 * Write 1 byte to i2c register
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param regaddr register to write to
 * @param value
 *
 * @return esp_err_t error codes
 */
esp_err_t generic_write_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                     uint8_t value);

/**
 * Write 2 words to i2c register
 *
 * @param dev PCA9685_t struct that hold the device information
 * @param regaddr register address to read
 * @param valueA
 * @param valueB
 *
 * @return esp_err_t error codes
 */
esp_err_t generic_write_i2c_register_two_words(PCA9685_t dev, uint8_t regaddr,
                                               uint16_t valueA,
                                               uint16_t valueB);


void MPU9150setAddrPort(uint8_t addr, i2c_port_t i2c_num) {}
esp_err_t MPU9150initialize() {}
esp_err_t MPU9150reset() {}

esp_err_t MPU9150setSampleRate(uint16_t rate) {}
esp_err_t MPU9150setClockSource(clock_src_t clockSrc) {}
esp_err_t MPU9150setDigitalLowPassFilter(dlpf_t dlpf) {}

esp_err_t MPU9150setAccelFullScale(accel_fs_t fsr) {}

esp_err_t MPU9150acceleration(int16_t* x, int16_t* y, int16_t* z) {}




esp_err_t generic_write_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                     uint8_t value) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t generic_read_two_i2c_register(PCA9685_t dev, uint8_t regaddr,
                                        uint8_t *valueA, uint8_t *valueB) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, dev.addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, valueA, ACK_VAL);
  i2c_master_read_byte(cmd, valueB, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t generic_write_i2c_register_two_words(PCA9685_t dev, uint8_t regaddr,
                                               uint16_t valueA,
                                               uint16_t valueB) {
  esp_err_t ret;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev.addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, valueA & 0xff, ACK_VAL);
  i2c_master_write_byte(cmd, valueA >> 8, NACK_VAL);
  i2c_master_write_byte(cmd, valueB & 0xff, ACK_VAL);
  i2c_master_write_byte(cmd, valueB >> 8, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(dev.i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}
