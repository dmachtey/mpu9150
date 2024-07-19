#include <stdint.h>
#include <stdio.h>
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "alarm.h"



void app_main(void) {
  esp_rom_gpio_pad_select_gpio(GPIO_NUM_23);
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_DEF_OUTPUT);

  uint8_t output = 0;

  while (1) {
    gpio_set_level(GPIO_NUM_23, output);
    output = !output;
    vTaskDelay(1000/portTICK_PERIOD_MS);

  }
}
