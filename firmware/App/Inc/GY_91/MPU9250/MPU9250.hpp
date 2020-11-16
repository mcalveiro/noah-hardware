/**
 * @file MPU9250.hpp
 * @author Gonzalo Cervetti (cervetti.g@gmail.com)
 * @brief This library allows to manage a MPU9250 IMU sensor.
 *
 * @version 0.1
 * @date 11-07-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once

// Standard libraries
#include <GY_91/MPU9250/Mpu9250Constants.hpp>
#include <GY_91/MPU9250/Mpu9250Types.hpp>
#include <cstdint>
#include <memory>

// CubeMx libraries
#include "cmsis_os.h"
#include "main.h"
#include "i2c.h"

// GY libraries
#include "NoahUtils.h"

using noah::noah_utils::Pin;

namespace gy91 {
namespace mpu9250 {

class MPU9250 {
 public:
  /**
   * @brief constructor
   * @param i2c_handle reference to the i2c_hadle needed to communicate with the sensor.
   */
  explicit MPU9250(std::shared_ptr<I2C_HandleTypeDef>& i2c_handle);

  /// @brief Default destructor.
  ~MPU9250() = default;

  /**
   * @brief Initializes the library.
   * @return True if everything is OK, False otherwise.
   */
  bool begin();

  void getAcceleration();
  void getAccelerationG();

 private:


  /// @brief Stores the values of each acceleration register.
  int16_t x_reg_;
  int16_t y_reg_;
  int16_t z_reg_;

  /// @brief Stores the acceleration in G's.
  float x_g_;
  float y_g_;
  float z_g_;

  /// @brief I2C handler to communicate with the sensor.
  std::shared_ptr<I2C_HandleTypeDef> i2c_handle_;

};

}

}
