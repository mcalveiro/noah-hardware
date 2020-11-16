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

// MPU Libraries
#include <GY_91/MPU9250/Mpu9250.hpp>
#include <GY_91/MPU9250/Mpu9250Constants.hpp>
#include <GY_91/MPU9250/Mpu9250Types.hpp>

namespace gy91 {
namespace mpu9250 {

MPU9250::MPU9250(std::shared_ptr<I2C_HandleTypeDef> &i2c_handle) :
    i2c_handle_ { i2c_handle } {
}

bool MPU9250::begin() {
  // Check if the device can be found.
  auto result = HAL_I2C_IsDeviceReady(i2c_handle_.get(), MPU9250_ADDRESS_SHIFTED, NUMBER_TRIALS, COMMUNICATION_TIMEOUT);
  if (result != HAL_OK) {
    return false;
  }

  // Check that the device connected is an MPU9250.
  uint8_t response;
  HAL_I2C_Mem_Read(i2c_handle_.get(), MPU9250_ADDRESS_SHIFTED, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &response, 0x01,
                   COMMUNICATION_TIMEOUT);
  if (response != WHO_AM_I_RESP) {
    return false;
  }

  // Reset the internal registers and restores the default settings.
  HAL_I2C_Mem_Write(i2c_handle_.get(), MPU9250_ADDRESS_SHIFTED, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t*) 0b10000000,
                    0x01, COMMUNICATION_TIMEOUT);
  osDelay(100);
  return true;
}

}
}
