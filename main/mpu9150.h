#ifndef MPU9150_HRIVER_H
#define MPU9150_HRIVER_H

#include "esp_err.h"
#include <stdint.h>

typedef struct {
  uint8_t addr;
  uint8_t i2c_num;
} I2CMASTER_T;

/*! Accel full-scale range */
  typedef enum {
    ACCEL_FS_2G  = 0,  //!< +/- 2 g  -> 16.384 LSB/g
    ACCEL_FS_4G  = 1,  //!< +/- 4 g  -> 8.192 LSB/g
    ACCEL_FS_8G  = 2,  //!< +/- 8 g  -> 4.096 LSB/g
    ACCEL_FS_16G = 3   //!< +/- 16 g -> 2.048 LSB/g
  } accel_fs_t;

  /*! Digital low-pass filter (based on gyro bandwidth) */
  typedef enum {
    DLPF_256HZ_NOLPF = 0,
    DLPF_188HZ       = 1,
    DLPF_98HZ        = 2,
    DLPF_42HZ        = 3,
    DLPF_20HZ        = 4,
    DLPF_10HZ        = 5,
    DLPF_5HZ         = 6,
  } dlpf_t;

  /*! Clock Source */
  typedef enum {
    CLOCK_INTERNAL = 0,  //!< Internal oscillator: 20MHz for MPU6500 and 8MHz for MPU6050
    CLOCK_PLL      = 3,  //!< Selects automatically best pll source (recommended)
    CLOCK_EXT32KHZ = 4,  //!< PLL with external 32.768kHz reference
    CLOCK_EXT19MHZ = 5,  //!< PLL with external 19.2MHz reference
    CLOCK_KEEP_RESET = 7,  //!< Stops the clock and keeps timing generator in reset
  } clock_src_t;

  void MPU9150setAddrPort(uint8_t addr, i2c_port_t i2c_num);
  esp_err_t MPU9150initialize();
  esp_err_t MPU9150reset();

  esp_err_t MPU9150setSampleRate(uint16_t rate);
  esp_err_t MPU9150setClockSource(clock_src_t clockSrc);
  esp_err_t MPU9150setDigitalLowPassFilter(dlpf_t dlpf);

  esp_err_t MPU9150setAccelFullScale(accel_fs_t fsr);

  esp_err_t MPU9150acceleration(int16_t* x, int16_t* y, int16_t* z);
#+END_SR



#endif /* MPU9150_HRIVER_H */
