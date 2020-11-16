/******************************************************************************
 SparkFunMPU9250-DMP.cpp - MPU-9250 Digital Motion Processor Arduino Library
 Jim Lindblom @ SparkFun Electronics
 original creation date: November 23, 2016
 https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

 This library implements motion processing functions of Invensense's MPU-9250.
 It is based on their Emedded MotionDriver 6.12 library.
 https://www.invensense.com/developers/software-downloads/

 Development environment specifics:
 Arduino IDE 1.6.12
 SparkFun 9DoF Razor IMU M0

 Supported Platforms:
 - ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
 ******************************************************************************/

// Standard libraries
#include <cmath>

// Project libraries
#include <GY_91/MPU9250/Mpu9250Dmp.hpp>
#include <GY_91/MPU9250/Mpu9250RegisterMap.h>
#include <GY_91/STM32Adapter.h>

extern "C" {
#include <GY_91/MPU9250/util/inv_mpu.h>
}

static uint8_t mpu9250_orientation;
static uint8_t tap_count;
static uint8_t tap_direction;
static bool _tap_available;
static void orient_cb(uint8_t orient);
static void tap_cb(uint8_t direction, uint8_t count);

MPU9250_DMP::MPU9250_DMP() {
  _mSense = 6.665f;  // Constant - 4915 / 32760
  _aSense = 0.0f;   // Updated after accel FSR is set
  _gSense = 0.0f;   // Updated after gyro FSR is set
}

inv_error_t MPU9250_DMP::begin(void) {
  inv_error_t result;
  struct int_param_s int_param;

  result = mpu_init(&int_param);

  if (result)
    return result;

  mpu_set_bypass(1);  // Place all slaves (including compass) on primary bus

  setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  _gSense = getGyroSens();
  _aSense = getAccelSens();

  return result;
}

inv_error_t MPU9250_DMP::enableInterrupt(uint8_t enable) {
  return set_int_enable(enable);
}

inv_error_t MPU9250_DMP::setIntLevel(uint8_t active_low) {
  return mpu_set_int_level(active_low);
}

inv_error_t MPU9250_DMP::setIntLatched(uint8_t enable) {
  return mpu_set_int_latched(enable);
}

int16_t MPU9250_DMP::getIntStatus(void) {
  int16_t status;
  if (mpu_get_int_status(&status) == INV_SUCCESS) {
    return status;
  }
  return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40, 
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
inv_error_t MPU9250_DMP::lowPowerAccel(uint16_t rate) {
  return mpu_lp_accel_mode(rate);
}

inv_error_t MPU9250_DMP::setGyroFSR(uint16_t fsr) {
  inv_error_t err;
  err = mpu_set_gyro_fsr(fsr);
  if (err == INV_SUCCESS) {
    _gSense = getGyroSens();
  }
  return err;
}

inv_error_t MPU9250_DMP::setAccelFSR(uint8_t fsr) {
  inv_error_t err;
  err = mpu_set_accel_fsr(fsr);
  if (err == INV_SUCCESS) {
    _aSense = getAccelSens();
  }
  return err;
}

uint16_t MPU9250_DMP::getGyroFSR(void) {
  uint16_t tmp;
  if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS) {
    return tmp;
  }
  return 0;
}

uint8_t MPU9250_DMP::getAccelFSR(void) {
  uint8_t tmp;
  if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS) {
    return tmp;
  }
  return 0;
}

uint16_t MPU9250_DMP::getMagFSR(void) {
  uint16_t tmp;
  if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS) {
    return tmp;
  }
  return 0;
}

inv_error_t MPU9250_DMP::setLPF(uint16_t lpf) {
  return mpu_set_lpf(lpf);
}

uint16_t MPU9250_DMP::getLPF(void) {
  uint16_t tmp;
  if (mpu_get_lpf(&tmp) == INV_SUCCESS) {
    return tmp;
  }
  return 0;
}

inv_error_t MPU9250_DMP::setSampleRate(uint16_t rate) {
  return mpu_set_sample_rate(rate);
}

uint16_t MPU9250_DMP::getSampleRate(void) {
  uint16_t tmp;
  if (mpu_get_sample_rate(&tmp) == INV_SUCCESS) {
    return tmp;
  }
  return 0;
}

inv_error_t MPU9250_DMP::setCompassSampleRate(uint16_t rate) {
  return mpu_set_compass_sample_rate(rate);
}

uint16_t MPU9250_DMP::getCompassSampleRate(void) {
  uint16_t tmp;
  if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS) {
    return tmp;
  }

  return 0;
}

float MPU9250_DMP::getGyroSens(void) {
  float sens;
  if (mpu_get_gyro_sens(&sens) == INV_SUCCESS) {
    return sens;
  }
  return 0;
}

uint16_t MPU9250_DMP::getAccelSens(void) {
  uint16_t sens;
  if (mpu_get_accel_sens(&sens) == INV_SUCCESS) {
    return sens;
  }
  return 0;
}

float MPU9250_DMP::getMagSens(void) {
  return 0.15;  // Static, 4915/32760
}

uint8_t MPU9250_DMP::getFifoConfig(void) {
  uint8_t sensors;
  if (mpu_get_fifo_config(&sensors) == INV_SUCCESS) {
    return sensors;
  }
  return 0;
}

inv_error_t MPU9250_DMP::configureFifo(uint8_t sensors) {
  return mpu_configure_fifo(sensors);
}

inv_error_t MPU9250_DMP::resetFifo(void) {
  return mpu_reset_fifo();
}

uint16_t MPU9250_DMP::fifoAvailable(void) {
  uint8_t fifoH, fifoL;

  if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
    return 0;
  if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
    return 0;

  return (fifoH << 8) | fifoL;
}

inv_error_t MPU9250_DMP::updateFifo(void) {
  int16_t gyro[3], accel[3];
  uint32_t timestamp;
  uint8_t sensors, more;

  if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
    return INV_ERROR;

  if (sensors & INV_XYZ_ACCEL) {
    ax = accel[X_AXIS];
    ay = accel[Y_AXIS];
    az = accel[Z_AXIS];
  }
  if (sensors & INV_X_GYRO)
    gx = gyro[X_AXIS];
  if (sensors & INV_Y_GYRO)
    gy = gyro[Y_AXIS];
  if (sensors & INV_Z_GYRO)
    gz = gyro[Z_AXIS];

  time = timestamp;

  return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::setSensors(uint8_t sensors) {
  return mpu_set_sensors(sensors);
}

bool MPU9250_DMP::dataReady() {
  uint8_t intStatusReg;

  if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS) {
    return (intStatusReg & (1 << INT_STATUS_RAW_DATA_RDY_INT));
  }
  return false;
}

inv_error_t MPU9250_DMP::update(uint8_t sensors) {
  inv_error_t aErr = INV_SUCCESS;
  inv_error_t gErr = INV_SUCCESS;
  inv_error_t mErr = INV_SUCCESS;
  inv_error_t tErr = INV_SUCCESS;

  if (sensors & UPDATE_ACCEL)
    aErr = updateAccel();
  if (sensors & UPDATE_GYRO)
    gErr = updateGyro();
  if (sensors & UPDATE_COMPASS)
    mErr = updateCompass();
  if (sensors & UPDATE_TEMP)
    tErr = updateTemperature();

  return aErr | gErr | mErr | tErr;
}

int16_t MPU9250_DMP::updateAccel(void) {
  int16_t data[3];

  if (mpu_get_accel_reg(data, &time)) {
    return INV_ERROR;
  }
  ax = data[X_AXIS];
  ay = data[Y_AXIS];
  az = data[Z_AXIS];
  return INV_SUCCESS;
}

int16_t MPU9250_DMP::updateGyro(void) {
  int16_t data[3];

  if (mpu_get_gyro_reg(data, &time)) {
    return INV_ERROR;
  }
  gx = data[X_AXIS];
  gy = data[Y_AXIS];
  gz = data[Z_AXIS];
  return INV_SUCCESS;
}

int16_t MPU9250_DMP::updateCompass(void) {
  int16_t data[3];

  if (mpu_get_compass_reg(data, &time)) {
    return INV_ERROR;
  }
  mx = data[X_AXIS];
  my = data[Y_AXIS];
  mz = data[Z_AXIS];
  return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::updateTemperature(void) {
  return mpu_get_temperature(&temperature, &time);
}

int16_t MPU9250_DMP::selfTest(uint8_t debug) {
  int32_t gyro[3], accel[3];
  return mpu_run_self_test(gyro, accel);
}

inv_error_t MPU9250_DMP::dmpBegin(uint16_t features, uint16_t fifoRate) {
  uint16_t feat = features;
  uint16_t rate = fifoRate;

  if (dmpLoad() != INV_SUCCESS)
    return INV_ERROR;

  // 3-axis and 6-axis LP quat are mutually exclusive.
  // If both are selected, default to 3-axis
  if (feat & DMP_FEATURE_LP_QUAT) {
    feat &= ~(DMP_FEATURE_6X_LP_QUAT);
    dmp_enable_lp_quat(1);
  } else if (feat & DMP_FEATURE_6X_LP_QUAT)
    dmp_enable_6x_lp_quat(1);

  if (feat & DMP_FEATURE_GYRO_CAL)
    dmp_enable_gyro_cal(1);

  if (dmpEnableFeatures(feat) != INV_SUCCESS)
    return INV_ERROR;

  rate = constrain(rate, 1, 200);
  if (dmpSetFifoRate(rate) != INV_SUCCESS)
    return INV_ERROR;

  return mpu_set_dmp_state(1);
}

inv_error_t MPU9250_DMP::dmpLoad(void) {
  return dmp_load_motion_driver_firmware();
}

uint16_t MPU9250_DMP::dmpGetFifoRate(void) {
  uint16_t rate;
  if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
    return rate;

  return 0;
}

inv_error_t MPU9250_DMP::dmpSetFifoRate(uint16_t rate) {
  if (rate > MAX_DMP_SAMPLE_RATE)
    rate = MAX_DMP_SAMPLE_RATE;
  return dmp_set_fifo_rate(rate);
}

inv_error_t MPU9250_DMP::dmpUpdateFifo(void) {
  int16_t gyro[3];
  int16_t accel[3];
  int32_t quat[4];
  uint32_t timestamp;
  int16_t sensors;
  uint8_t more;

  if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != INV_SUCCESS) {
    return INV_ERROR;
  }

  if (sensors & INV_XYZ_ACCEL) {
    ax = accel[X_AXIS];
    ay = accel[Y_AXIS];
    az = accel[Z_AXIS];
  }
  if (sensors & INV_X_GYRO)
    gx = gyro[X_AXIS];
  if (sensors & INV_Y_GYRO)
    gy = gyro[Y_AXIS];
  if (sensors & INV_Z_GYRO)
    gz = gyro[Z_AXIS];
  if (sensors & INV_WXYZ_QUAT) {
    qw = quat[0];
    qx = quat[1];
    qy = quat[2];
    qz = quat[3];
  }

  time = timestamp;

  return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::dmpEnableFeatures(uint16_t mask) {
  uint16_t enMask = 0;
  enMask |= mask;
  // Combat known issue where fifo sample rate is incorrect
  // unless tap is enabled in the DMP.
  enMask |= DMP_FEATURE_TAP;
  return dmp_enable_feature(enMask);
}

uint16_t MPU9250_DMP::dmpGetEnabledFeatures(void) {
  uint16_t mask;
  if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
    return mask;
  return 0;
}

inv_error_t MPU9250_DMP::dmpSetTap(uint16_t xThresh, uint16_t yThresh, uint16_t zThresh, uint8_t taps, uint16_t tapTime,
                                   uint16_t tapMulti) {
  uint8_t axes = 0;
  if (xThresh > 0) {
    axes |= TAP_X;
    xThresh = constrain(xThresh, 1, 1600);
    if (dmp_set_tap_thresh(1 << X_AXIS, xThresh) != INV_SUCCESS)
      return INV_ERROR;
  }
  if (yThresh > 0) {
    axes |= TAP_Y;
    yThresh = constrain(yThresh, 1, 1600);
    if (dmp_set_tap_thresh(1 << Y_AXIS, yThresh) != INV_SUCCESS)
      return INV_ERROR;
  }
  if (zThresh > 0) {
    axes |= TAP_Z;
    zThresh = constrain(zThresh, 1, 1600);
    if (dmp_set_tap_thresh(1 << Z_AXIS, zThresh) != INV_SUCCESS)
      return INV_ERROR;
  }
  if (dmp_set_tap_axes(axes) != INV_SUCCESS)
    return INV_ERROR;
  if (dmp_set_tap_count(taps) != INV_SUCCESS)
    return INV_ERROR;
  if (dmp_set_tap_time(tapTime) != INV_SUCCESS)
    return INV_ERROR;
  if (dmp_set_tap_time_multi(tapMulti) != INV_SUCCESS)
    return INV_ERROR;

  dmp_register_tap_cb(tap_cb);

  return INV_SUCCESS;
}

uint8_t MPU9250_DMP::getTapDir(void) {
  _tap_available = false;
  return tap_direction;
}

uint8_t MPU9250_DMP::getTapCount(void) {
  _tap_available = false;
  return tap_count;
}

bool MPU9250_DMP::tapAvailable(void) {
  return _tap_available;
}

inv_error_t MPU9250_DMP::dmpSetOrientation(const int8_t *orientationMatrix) {
  uint16_t scalar;
  scalar = orientation_row_2_scale(orientationMatrix);
  scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
  scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;

  dmp_register_android_orient_cb(orient_cb);

  return dmp_set_orientation(scalar);
}

uint8_t MPU9250_DMP::dmpGetOrientation(void) {
  return mpu9250_orientation;
}

inv_error_t MPU9250_DMP::dmpEnable3Quat(void) {
  uint16_t dmpFeatures;

  // 3-axis and 6-axis quat are mutually exclusive
  dmpFeatures = dmpGetEnabledFeatures();
  dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
  dmpFeatures |= DMP_FEATURE_LP_QUAT;

  if (dmpEnableFeatures(dmpFeatures) != INV_SUCCESS)
    return INV_ERROR;

  return dmp_enable_lp_quat(1);
}

uint32_t MPU9250_DMP::dmpGetPedometerSteps(void) {
  uint32_t steps;
  if (dmp_get_pedometer_step_count(&steps) == INV_SUCCESS) {
    return steps;
  }
  return 0;
}

inv_error_t MPU9250_DMP::dmpSetPedometerSteps(uint32_t steps) {
  return dmp_set_pedometer_step_count(steps);
}

uint32_t MPU9250_DMP::dmpGetPedometerTime(void) {
  uint32_t walkTime;
  if (dmp_get_pedometer_walk_time(&walkTime) == INV_SUCCESS) {
    return walkTime;
  }
  return 0;
}

inv_error_t MPU9250_DMP::dmpSetPedometerTime(uint32_t time) {
  return dmp_set_pedometer_walk_time(time);
}

float MPU9250_DMP::calcAccel(int16_t axis) {
  return (float) axis / (float) _aSense;
}

float MPU9250_DMP::calcGyro(int16_t axis) {
  return (float) axis / (float) _gSense;
}

float MPU9250_DMP::calcMag(int16_t axis) {
  return (float) axis / (float) _mSense;
}

float MPU9250_DMP::calcQuat(int32_t axis) {
  return qToFloat(axis, 30);
}

float MPU9250_DMP::qToFloat(int32_t number, uint8_t q) {
  uint32_t mask = 0;
  for (int16_t i = 0; i < q; i++) {
    mask |= (1 << i);
  }
  return (number >> q) + ((number & mask) / (float) (2 << (q - 1)));
}

void MPU9250_DMP::computeEulerAngles(bool degrees) {
  float dqw = qToFloat(qw, 30);
  float dqx = qToFloat(qx, 30);
  float dqy = qToFloat(qy, 30);
  float dqz = qToFloat(qz, 30);

  float ysqr = dqy * dqy;
  float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
  float t1 = +2.0f * (dqx * dqy - dqw * dqz);
  float t2 = -2.0f * (dqx * dqz + dqw * dqy);
  float t3 = +2.0f * (dqy * dqz - dqw * dqx);
  float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

  // Keep t2 within range of asin (-1, 1)
  t2 = t2 > 1.0f ? 1.0f : t2;
  t2 = t2 < -1.0f ? -1.0f : t2;

  pitch = asin(t2) * 2;
  roll = atan2(t3, t4);
  yaw = atan2(t1, t0);

  if (degrees) {
    pitch *= (180.0 / M_PI);
    roll *= (180.0 / M_PI);
    yaw *= (180.0 / M_PI);
    if (pitch < 0)
      pitch = 360.0 + pitch;
    if (roll < 0)
      roll = 360.0 + roll;
    if (yaw < 0)
      yaw = 360.0 + yaw;
  }
}

float MPU9250_DMP::computeCompassHeading(void) {
  if (my == 0)
    heading = (mx < 0) ? M_PI : 0;
  else
    heading = atan2(mx, my);

  if (heading > M_PI)
    heading -= (2 * M_PI);
  else if (heading < -M_PI)
    heading += (2 * M_PI);
  else if (heading < 0)
    heading += 2 * M_PI;

  heading *= 180.0 / M_PI;

  return heading;
}

uint16_t MPU9250_DMP::orientation_row_2_scale(const int8_t *row) {
  uint16_t b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;		// error
  return b;
}

static void tap_cb(uint8_t direction, uint8_t count) {
  _tap_available = true;
  tap_count = count;
  tap_direction = direction;
}

static void orient_cb(uint8_t orient) {
  mpu9250_orientation = orient;
}
