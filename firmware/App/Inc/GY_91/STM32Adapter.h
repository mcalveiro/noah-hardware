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

#pragma once

// Standard libraries
#include <stdint.h>

// STM32 libraries
#include "main.h"
#include "i2c.h"

// Project libraries
#include "Config.h"

/* The following functions must be defined for this platform:
 * i2c_write(uint8_t slave_addr, uint8_t reg_addr,
 *      uint8_t length, uint8_t const *data)
 * i2c_read(uint8_t slave_addr, uint8_t reg_addr,
 *      uint8_t length, uint8_t *data)
 * delay_ms(uint32_t num_ms)
 * get_ms(uint32_t *count)
 */
#define i2c_write(a, b, c, d) HAL_I2C_Mem_Write(&IMU_PORT, a, b, I2C_MEMADD_SIZE_8BIT, d, c, IMU_MAX_TIMEOUT)
#define i2c_read(a, b, c, d)  HAL_I2C_Mem_Read(&IMU_PORT, a, b, I2C_MEMADD_SIZE_8BIT, d, c, IMU_MAX_TIMEOUT)
#define delay_ms  osDelay
#define get_ms    get_clock_ms
#define log_i     _MLPrintLog
#define log_e     _MLPrintLog

/**
 * @brief prints log messages.
 * @param tag message to print
 * @return 0 if there was no issue
 */
int32_t _MLPrintLog(const char *tag, ...);

/**
 * @brief calculates how many ms have passed since start.
 * @param count variable to store the tick count.
 * @return 0
 */
uint32_t get_clock_ms(uint32_t *count);

/**
 * @brief ANSI C version of constrain method.
 * @param x value to be constrained
 * @param a min posible value of x
 * @param b max possible value of x
 * @return value constrained
 */
uint16_t constrain(uint16_t x, uint16_t a, uint16_t b);
