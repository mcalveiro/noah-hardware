/**
 * @file STM32Adapter
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief This file defines all the basic functions required by MPU9250 to communicate with the device.
 *
 * @version 0.1
 * @date 11-07-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

// Standard libraries
#include <stdint.h>

// Project libraries
#include <GY_91/STM32Adapter.h>

int32_t _MLPrintLog(const char *tag, ...) {
  return 0;
}

uint32_t get_clock_ms(uint32_t *count) {
  *count = HAL_GetTick();
  return 0;
}

uint16_t constrain(uint16_t x, uint16_t a, uint16_t b) {
  if (x < a) {
    return a;
  } else if (b < x) {
    return b;
  }
  return x;
}
