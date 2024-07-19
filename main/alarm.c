#include "alarm.h"
#include "mpu9150.h"
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <stdint.h>

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */

uint8_t master_initialized = 0;
I2CMASTER_DEV_T i2c_master;

/**
 * Initialize the i2c master
 *
 */
void i2c_master_init(void);



void alrmInit(uint8_t threshold) {
  if (!master_initialized){
    master_initialized = 1;
    i2c_master_init();
    MPU9150initialize();

  }

}
uint8_t alarmStatus(void){}
void alarmReset(void){}


void i2c_master_init(void) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  int i2c_master_port = I2C_MASTER_NUM;
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                                     I2C_MASTER_RX_BUF_DISABLE,
                                     I2C_MASTER_TX_BUF_DISABLE, 0));
}
