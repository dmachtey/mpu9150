// =========================================================================
// Copyright 2024 Damian Pablo Machtey. All rights reserved.
// =========================================================================

#include "alarm.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void app_main(void) {

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_23);
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_DEF_OUTPUT);

  alarmInit(0.30);

  while (1) {

    if (alarmStatus()) {
      gpio_set_level(GPIO_NUM_23, 1);
      vTaskDelay(1200 / portTICK_PERIOD_MS);
    }
    gpio_set_level(GPIO_NUM_23, 0);
    alarmReset();
  }
}
