/**
 * @file MPU9250Types.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief Types used for MPU9250
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

enum class AccelerometerRange {
  RANGE_16G = 0b11,
  RANGE_8G = 0b10,
  RANGE_4G = 0b01,
  RANGE_2G = 0b00
};

enum class GyroscopeRange {
  RANGE_GYRO_2000 = 0b11,
  RANGE_GYRO_1000 = 0b10,
  RANGE_GYRO_500 = 0b01,
  RANGE_GYRO_250 = 0b00
};

enum class MagnetometerScale{
  SCALE_14_BITS = 0,
  SCALE_16_BITS = 1
};

enum class MagnetometerSpeed {
  MAG_8_Hz = 0,
  MAG_100_Hz = 1
};

}

}
