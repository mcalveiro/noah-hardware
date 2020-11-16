/**
 * @file MPU9250Constants.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Constants for MPU9250
 *
 * @version 0.1
 * @date 11-09-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Standard libraries
#include <cstdint>

namespace gy91 {
namespace mpu9250 {

// Change to 0x69 if AD0 is in high state
constexpr uint8_t MPU9250_ADDRESS { 0x68 };
constexpr uint8_t MPU9250_ADDRESS_SHIFTED{MPU9250_ADDRESS<<1};
// MPU9250 is 0x71 and MPU9255 is 0x73
constexpr uint8_t WHO_AM_I_RESP { 0x73 };
// MPU9250 must bypass to access the AK8963 in INT_PIN_CFG
constexpr uint8_t AK8963_ADDRESS { 0x0C };
constexpr uint8_t WHO_AM_I_RESP_MAG { 0x48 };

//Accelerometer and Gyroscope registers
constexpr uint8_t SMPLRT_DIV { 0x19 };
constexpr uint8_t CONFIG { 0x1A };
constexpr uint8_t GYRO_CONFIG { 0x1B };
constexpr uint8_t ACCEL_CONFIG { 0x1C };
constexpr uint8_t ACCEL_CONFIG2 { 0x1D };
constexpr uint8_t INT_PIN_CFG { 0x37 };
constexpr uint8_t INT_ENABLE { 0x38 };
constexpr uint8_t INT_STATUS { 0x3A };
constexpr uint8_t ACCEL_XOUT_H { 0x3B };
constexpr uint8_t TEMP_OUT_H { 0x41 };
constexpr uint8_t GYRO_XOUT_H { 0x43 };
constexpr uint8_t PWR_MGMT_1 { 0x6B };
constexpr uint8_t PWR_MGMT_2 { 0x6C };
constexpr uint8_t WHO_AM_I { 0x75 };

//Magnetometer Registers
// should return 0x48
constexpr uint8_t WHO_AM_I_AK8963 { 0x00 };
// data status
constexpr uint8_t AK8963_ST1 { 0x02 };
// data
constexpr uint8_t AK8963_XOUT_L { 0x03 };
// Power down (0000), Continuous measurement mode 1 (0010), CMM2 (0110), Fuse ROM (1111) on bits 3:0
constexpr uint8_t AK8963_CNTL { 0x0A };
// Fuse ROM x-axis sensitivity adjustment value
constexpr uint8_t AK8963_ASAX { 0x10 };
// Fuse ROM y-axis sensitivity adjustment value
constexpr uint8_t AK8963_ASAY { 0x11 };
// Fuse ROM z-axis sensitivity adjustment value
constexpr uint8_t AK8963_ASAZ { 0x12 };
// Amount of trials that it will check for connection with the device.
constexpr uint8_t NUMBER_TRIALS { 0x12 };
// Time it will wait for response from the device [ms].
constexpr uint32_t COMMUNICATION_TIMEOUT { 100 };

}

}
