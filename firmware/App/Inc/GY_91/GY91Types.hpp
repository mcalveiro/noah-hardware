/**
 * @file GY91Types.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Set of objects required for the library.
 *
 * @version 0.1
 * @date 11-07-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

namespace gy91 {

/// @brief Available accelerometer range.
enum class AccelRange {
  RANGE_16G = 0b11,
  RANGE_8G = 0b10,
  RANGE_4G = 0b01,
  RANGE_2G = 0b00
} accel_range;

/// @brief Sensitivity of the gyroscope.
enum class GyroRange {
  RANGE_GYRO_2000 = 0b11,
  RANGE_GYRO_1000 = 0b10,
  RANGE_GYRO_500 = 0b01,
  RANGE_GYRO_250 = 0b00
};

/// @brief Sensitivity of the magnetometer.
enum class MagScale {
  SCALE_14_BITS = 0,
  SCALE_16_BITS = 1
};

/// @brief Update rate of the magnetometer.
enum class MagSpeed {
  MAG_8_Hz = 0,
  MAG_100_Hz = 1
};
}
