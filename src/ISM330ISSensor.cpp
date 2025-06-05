/**
 ******************************************************************************
 * @file    ISM330ISSensor.cpp
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    05 June 2025
 * @brief   Implementation of a ISM330IS sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ISM330ISSensor.h"
/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the ism330is's instance
 */
ISM330ISSensor::ISM330ISSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = ISM330IS_io_write;
  reg_ctx.read_reg = ISM330IS_io_read;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  is_initialized = 0;
  acc_is_enabled = 0;
  gyro_is_enabled = 0;
}
/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
ISM330ISSensor::ISM330ISSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = ISM330IS_io_write;
  reg_ctx.read_reg = ISM330IS_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  is_initialized = 0;
  acc_is_enabled = 0;
  gyro_is_enabled = 0;
}
/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
ISM330ISStatusTypeDef ISM330ISSensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  /* Set main memory bank */
  if (Set_Mem_Bank((uint8_t)ISM330IS_MAIN_MEM_BANK) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (ism330is_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Enable BDU */
  if (ism330is_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Select default output data rate. */
  acc_odr = ISM330IS_XL_ODR_AT_104Hz_HP;
  /* Output data rate selection - power down. */
  if (ism330is_xl_data_rate_set(&reg_ctx, ISM330IS_XL_ODR_OFF) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Full scale selection. */
  if (ism330is_xl_full_scale_set(&reg_ctx, ISM330IS_2g) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Select default output data rate. */
  gyro_odr = ISM330IS_GY_ODR_AT_104Hz_HP;
  /* Output data rate selection - power down. */
  if (ism330is_gy_data_rate_set(&reg_ctx, ISM330IS_GY_ODR_OFF) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Full scale selection. */
  if (ism330is_gy_full_scale_set(&reg_ctx, ISM330IS_2000dps) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  if (ret == ISM330IS_OK) {
    is_initialized = 1;
  }
  return ret;
}
/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
ISM330ISStatusTypeDef ISM330ISSensor::end()
{
  if (is_initialized == 1U) {
    return ISM330IS_ERROR;
  }
  /* Reset CS configuration */
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, INPUT);
  }
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  /* Disable the component */
  if (Disable_X() != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  if (Disable_G() != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  if (ret == ISM330IS_OK) {
    /* Reset output data rate. */
    acc_odr = ISM330IS_XL_ODR_OFF;
    gyro_odr = ISM330IS_GY_ODR_OFF;
    is_initialized = 0;
  }
  return ret;
}
/**
  * @brief  Read component ID
  * @param  Id the WHO_AM_I value
  * @retval 0 in case of success, an error code otherwise
  */
ISM330ISStatusTypeDef ISM330ISSensor::ReadID(uint8_t *Id)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  if (ism330is_device_id_get(&reg_ctx, Id) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Enable the ISM330IS accelerometer sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Enable_X()
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  /* Check if the component is already enabled */
  if (acc_is_enabled == 1U) {
    ret = ISM330IS_OK;
  } else {
    /* Output data rate selection. */
    if (ism330is_xl_data_rate_set(&reg_ctx, acc_odr) != ISM330IS_OK) {
      ret = ISM330IS_ERROR;
    }
    acc_is_enabled = 1;
  }
  return ret;
}
/**
* @brief  Disable the ISM330IS accelerometer sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Disable_X()
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  /* Check if the component is already disabled */
  if (acc_is_enabled == 0U) {
    ret = ISM330IS_OK;
  } else {
    /* Get current output data rate. */
    if (ism330is_xl_data_rate_get(&reg_ctx, &acc_odr) != ISM330IS_OK) {
      ret = ISM330IS_ERROR;
    }
    /* Output data rate selection - power down. */
    if (ism330is_xl_data_rate_set(&reg_ctx, ISM330IS_XL_ODR_OFF) != ISM330IS_OK) {
      ret = ISM330IS_ERROR;
    }
    acc_is_enabled = 0;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS accelerometer sensor sensitivity
* @param  Sensitivity pointer
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_Sensitivity(float_t *Sensitivity)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_xl_full_scale_t full_scale;
  /* Read actual full scale selection from sensor. */
  if (ism330is_xl_full_scale_get(&reg_ctx, &full_scale) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case ISM330IS_2g:
      *Sensitivity = ISM330IS_ACC_SENSITIVITY_FS_2G;
      break;
    case ISM330IS_4g:
      *Sensitivity = ISM330IS_ACC_SENSITIVITY_FS_4G;
      break;
    case ISM330IS_8g:
      *Sensitivity = ISM330IS_ACC_SENSITIVITY_FS_8G;
      break;
    case ISM330IS_16g:
      *Sensitivity = ISM330IS_ACC_SENSITIVITY_FS_16G;
      break;
    default:
      ret = ISM330IS_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS accelerometer sensor output data rate
* @param  Odr pointer where the output data rate is written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_OutputDataRate(float_t *Odr)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_xl_data_rate_t odr_low_level;
  /* Get current output data rate. */
  if (ism330is_xl_data_rate_get(&reg_ctx, &odr_low_level) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  switch (odr_low_level) {
    case ISM330IS_XL_ODR_OFF:
      *Odr = 0.0f;
      break;
    case ISM330IS_XL_ODR_AT_1Hz6_LP:
      *Odr = 1.6f;
      break;
    case ISM330IS_XL_ODR_AT_12Hz5_LP:
    case ISM330IS_XL_ODR_AT_12Hz5_HP:
      *Odr = 12.5f;
      break;
    case ISM330IS_XL_ODR_AT_26H_LP:
    case ISM330IS_XL_ODR_AT_26H_HP:
      *Odr = 26.0f;
      break;
    case ISM330IS_XL_ODR_AT_52Hz_LP:
    case ISM330IS_XL_ODR_AT_52Hz_HP:
      *Odr = 52.0f;
      break;
    case ISM330IS_XL_ODR_AT_104Hz_LP:
    case ISM330IS_XL_ODR_AT_104Hz_HP:
      *Odr = 104.0f;
      break;
    case ISM330IS_XL_ODR_AT_208Hz_LP:
    case ISM330IS_XL_ODR_AT_208Hz_HP:
      *Odr = 208.0f;
      break;
    case ISM330IS_XL_ODR_AT_416Hz_LP:
    case ISM330IS_XL_ODR_AT_416Hz_HP:
      *Odr = 416.0f;
      break;
    case ISM330IS_XL_ODR_AT_833Hz_LP:
    case ISM330IS_XL_ODR_AT_833Hz_HP:
      *Odr = 833.0f;
      break;
    case ISM330IS_XL_ODR_AT_1667Hz_LP:
    case ISM330IS_XL_ODR_AT_1667Hz_HP:
      *Odr = 1667.0f;
      break;
    case ISM330IS_XL_ODR_AT_3333Hz_LP:
    case ISM330IS_XL_ODR_AT_3333Hz_HP:
      *Odr = 3333.0f;
      break;
    case ISM330IS_XL_ODR_AT_6667Hz_LP:
    case ISM330IS_XL_ODR_AT_6667Hz_HP:
      *Odr = 6667.0f;
      break;
    default:
      ret = ISM330IS_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS accelerometer sensor output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_X_OutputDataRate(float_t Odr)
{
  ISM330ISStatusTypeDef ret;
  /* Check if the component is enabled */
  if (acc_is_enabled == 1U) {
    ret = Set_X_OutputDataRate_When_Enabled(Odr);
  } else {
    ret = Set_X_OutputDataRate_When_Disabled(Odr);
  }
  return ret;
}
/**
* @brief  Get the ISM330IS accelerometer sensor full scale
* @param  FullScale pointer where the full scale is written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_FullScale(int32_t *FullScale)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_xl_full_scale_t fs_low_level;
  /* Read actual full scale selection from sensor. */
  if (ism330is_xl_full_scale_get(&reg_ctx, &fs_low_level) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  switch (fs_low_level) {
    case ISM330IS_2g:
      *FullScale =  2;
      break;
    case ISM330IS_4g:
      *FullScale =  4;
      break;
    case ISM330IS_8g:
      *FullScale =  8;
      break;
    case ISM330IS_16g:
      *FullScale = 16;
      break;
    default:
      ret = ISM330IS_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS accelerometer sensor full scale
* @param  FullScale the functional full scale to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_X_FullScale(int32_t FullScale)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_xl_full_scale_t new_fs;
  new_fs = (FullScale <= 2) ? ISM330IS_2g
           : (FullScale <= 4) ? ISM330IS_4g
           : (FullScale <= 8) ? ISM330IS_8g
           :                    ISM330IS_16g;
  if (ism330is_xl_full_scale_set(&reg_ctx, new_fs) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS accelerometer sensor raw axes
* @param  Value pointer where the raw values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_AxesRaw(ISM330IS_AxesRaw_t *Value)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  int16_t data_raw[3];
  /* Read raw data values. */
  if (ism330is_acceleration_raw_get(&reg_ctx, data_raw) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Format the data. */
  Value->x = data_raw[0];
  Value->y = data_raw[1];
  Value->z = data_raw[2];
  return ret;
}
/**
* @brief  Get the ISM330IS accelerometer sensor axes
* @param  Acceleration pointer where the values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_Axes(ISM330IS_Axes_t *Acceleration)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  int16_t data_raw[3];
  float_t sensitivity = 0.0f;
  /* Read raw data values. */
  if (ism330is_acceleration_raw_get(&reg_ctx, data_raw) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Get ISM330IS actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Calculate the data. */
  Acceleration->x = (int32_t)((float_t)((float_t)data_raw[0] * sensitivity));
  Acceleration->y = (int32_t)((float_t)((float_t)data_raw[1] * sensitivity));
  Acceleration->z = (int32_t)((float_t)((float_t)data_raw[2] * sensitivity));
  return ret;
}
/**
* @brief  Enable the ISM330IS gyroscope sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Enable_G()
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  /* Check if the component is already enabled */
  if (gyro_is_enabled == 1U) {
    ret = ISM330IS_OK;
  } else {
    /* Output data rate selection. */
    if (ism330is_gy_data_rate_set(&reg_ctx, gyro_odr) != ISM330IS_OK) {
      ret = ISM330IS_ERROR;
    }
    gyro_is_enabled = 1;
  }
  return ret;
}
/**
* @brief  Disable the ISM330IS gyroscope sensor
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Disable_G()
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  /* Check if the component is already disabled */
  if (gyro_is_enabled == 0U) {
    ret = ISM330IS_OK;
  } else {
    /* Get current output data rate. */
    if (ism330is_gy_data_rate_get(&reg_ctx, &gyro_odr) != ISM330IS_OK) {
      ret = ISM330IS_ERROR;
    }
    /* Output data rate selection - power down. */
    if (ism330is_gy_data_rate_set(&reg_ctx, ISM330IS_GY_ODR_OFF) != ISM330IS_OK) {
      ret = ISM330IS_ERROR;
    }
    gyro_is_enabled = 0;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS gyroscope sensor sensitivity
* @param  Sensitivity pointer
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_Sensitivity(float_t *Sensitivity)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_gy_full_scale_t full_scale;
  /* Read actual full scale selection from sensor. */
  if (ism330is_gy_full_scale_get(&reg_ctx, &full_scale) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case ISM330IS_125dps:
      *Sensitivity = ISM330IS_GYRO_SENSITIVITY_FS_125DPS;
      break;
    case ISM330IS_250dps:
      *Sensitivity = ISM330IS_GYRO_SENSITIVITY_FS_250DPS;
      break;
    case ISM330IS_500dps:
      *Sensitivity = ISM330IS_GYRO_SENSITIVITY_FS_500DPS;
      break;
    case ISM330IS_1000dps:
      *Sensitivity = ISM330IS_GYRO_SENSITIVITY_FS_1000DPS;
      break;
    case ISM330IS_2000dps:
      *Sensitivity = ISM330IS_GYRO_SENSITIVITY_FS_2000DPS;
      break;
    default:
      ret = ISM330IS_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS gyroscope sensor output data rate
* @param  Odr pointer where the output data rate is written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_OutputDataRate(float_t *Odr)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_gy_data_rate_t odr_low_level;
  /* Get current output data rate. */
  if (ism330is_gy_data_rate_get(&reg_ctx, &odr_low_level) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  switch (odr_low_level) {
    case ISM330IS_GY_ODR_OFF:
      *Odr = 0.0f;
      break;
    case ISM330IS_GY_ODR_AT_12Hz5_LP:
    case ISM330IS_GY_ODR_AT_12Hz5_HP:
      *Odr = 12.5f;
      break;
    case ISM330IS_GY_ODR_AT_26H_LP:
    case ISM330IS_GY_ODR_AT_26H_HP:
      *Odr = 26.0f;
      break;
    case ISM330IS_GY_ODR_AT_52Hz_LP:
    case ISM330IS_GY_ODR_AT_52Hz_HP:
      *Odr = 52.0f;
      break;
    case ISM330IS_GY_ODR_AT_104Hz_LP:
    case ISM330IS_GY_ODR_AT_104Hz_HP:
      *Odr = 104.0f;
      break;
    case ISM330IS_GY_ODR_AT_208Hz_LP:
    case ISM330IS_GY_ODR_AT_208Hz_HP:
      *Odr = 208.0f;
      break;
    case ISM330IS_GY_ODR_AT_416Hz_LP:
    case ISM330IS_GY_ODR_AT_416Hz_HP:
      *Odr = 416.0f;
      break;
    case ISM330IS_GY_ODR_AT_833Hz_LP:
    case ISM330IS_GY_ODR_AT_833Hz_HP:
      *Odr = 833.0f;
      break;
    case ISM330IS_GY_ODR_AT_1667Hz_LP:
    case ISM330IS_GY_ODR_AT_1667Hz_HP:
      *Odr =  1667.0f;
      break;
    case ISM330IS_GY_ODR_AT_3333Hz_LP:
    case ISM330IS_GY_ODR_AT_3333Hz_HP:
      *Odr =  3333.0f;
      break;
    case ISM330IS_GY_ODR_AT_6667Hz_LP:
    case ISM330IS_GY_ODR_AT_6667Hz_HP:
      *Odr =  6667.0f;
      break;
    default:
      ret = ISM330IS_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS gyroscope sensor output data rate
* @param  Odr the output data rate value to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_G_OutputDataRate(float_t Odr)
{
  ISM330ISStatusTypeDef ret;
  /* Check if the component is enabled */
  if (gyro_is_enabled == 1U) {
    ret = Set_G_OutputDataRate_When_Enabled(Odr);
  } else {
    ret = Set_G_OutputDataRate_When_Disabled(Odr);
  }
  return ret;
}
/**
* @brief  Get the ISM330IS gyroscope sensor full scale
* @param  FullScale pointer where the full scale is written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_FullScale(int32_t  *FullScale)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_gy_full_scale_t fs_low_level;
  /* Read actual full scale selection from sensor. */
  if (ism330is_gy_full_scale_get(&reg_ctx, &fs_low_level) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  switch (fs_low_level) {
    case ISM330IS_125dps:
      *FullScale =  125;
      break;
    case ISM330IS_250dps:
      *FullScale =  250;
      break;
    case ISM330IS_500dps:
      *FullScale =  500;
      break;
    case ISM330IS_1000dps:
      *FullScale = 1000;
      break;
    case ISM330IS_2000dps:
      *FullScale = 2000;
      break;
    default:
      ret = ISM330IS_ERROR;
      break;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS gyroscope sensor full scale
* @param  FullScale the functional full scale to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_G_FullScale(int32_t FullScale)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_gy_full_scale_t new_fs;
  new_fs = (FullScale <= 125)  ? ISM330IS_125dps
           : (FullScale <= 250)  ? ISM330IS_250dps
           : (FullScale <= 500)  ? ISM330IS_500dps
           : (FullScale <= 1000) ? ISM330IS_1000dps
           :                       ISM330IS_2000dps;
  if (ism330is_gy_full_scale_set(&reg_ctx, new_fs) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS gyroscope sensor raw axes
* @param  Value pointer where the raw values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_AxesRaw(ISM330IS_AxesRaw_t *Value)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  int16_t data_raw[3];
  /* Read raw data values. */
  if (ism330is_angular_rate_raw_get(&reg_ctx, data_raw) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Format the data. */
  Value->x = data_raw[0];
  Value->y = data_raw[1];
  Value->z = data_raw[2];
  return ret;
}
/**
* @brief  Get the ISM330IS gyroscope sensor axes
* @param  AngularRate pointer where the values of the axes are written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_Axes(ISM330IS_Axes_t *AngularRate)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  int16_t data_raw[3];
  float_t sensitivity = 0.0f;
  /* Read raw data values. */
  if (ism330is_angular_rate_raw_get(&reg_ctx, data_raw) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Get ISM330IS actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  /* Calculate the data. */
  AngularRate->x = (int32_t)((float_t)((float_t)data_raw[0] * sensitivity));
  AngularRate->y = (int32_t)((float_t)((float_t)data_raw[1] * sensitivity));
  AngularRate->z = (int32_t)((float_t)((float_t)data_raw[2] * sensitivity));
  return ret;
}
/**
* @brief  Get the ISM330IS register value
* @param  Reg address to be read
* @param  Data pointer where the value is written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  if (ism330is_read_reg(&reg_ctx, Reg, Data, 1) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS register value
* @param  Reg address to be written
* @param  Data value to be written
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  if (ism330is_write_reg(&reg_ctx, Reg, &Data, 1) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set self test
* @param  Val the value of st_xl in reg CTRL5_C
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_X_SelfTest(uint8_t Val)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_xl_self_test_t reg;
  reg = (Val == 1U)  ? ISM330IS_XL_ST_POSITIVE
        : (Val == 2U)  ? ISM330IS_XL_ST_NEGATIVE
        :                ISM330IS_XL_ST_DISABLE;
  if (ism330is_xl_self_test_set(&reg_ctx, reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS ACC data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  if (ism330is_xl_flag_data_ready_get(&reg_ctx, Status) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS ACC initialization status
* @param  Status 1 if initialized, 0 otherwise
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_X_Init_Status(uint8_t *Status)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;

  *Status = is_initialized;
  return ret;
}
/**
* @brief  Set DRDY on INT1
* @param  Val the value of int1_drdy_xl in reg INT1_CTRL
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_X_INT1_DRDY(uint8_t Val)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_pin_int1_route_t reg;
  if (ism330is_pin_int1_route_get(&reg_ctx, &reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  if (Val <= 1U) {
    reg.drdy_xl = Val;
  } else {
    ret = ISM330IS_ERROR;
  }
  if (ism330is_pin_int1_route_set(&reg_ctx, reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set self test
* @param  Val the value of st_xl in reg CTRL5_C
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_G_SelfTest(uint8_t Val)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_gy_self_test_t reg;
  reg = (Val == 1U)  ? ISM330IS_GY_ST_POSITIVE
        : (Val == 2U)  ? ISM330IS_GY_ST_NEGATIVE
        :                ISM330IS_GY_ST_DISABLE;
  if (ism330is_gy_self_test_set(&reg_ctx, reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS GYRO data ready bit value
* @param  Status the status of data ready bit
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  if (ism330is_gy_flag_data_ready_get(&reg_ctx, Status) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Get the ISM330IS GYRO initialization status
* @param  Status 1 if initialized, 0 otherwise
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Get_G_Init_Status(uint8_t *Status)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;

  *Status = is_initialized;
  return ret;
}
/**
* @brief  Set DRDY on INT1
* @param  Val the value of int1_drdy_g in reg INT1_CTRL
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_G_INT1_DRDY(uint8_t Val)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_pin_int1_route_t reg;
  if (ism330is_pin_int1_route_get(&reg_ctx, &reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  if (Val <= 1U) {
    reg.drdy_gy = Val;
  } else {
    ret = ISM330IS_ERROR;
  }
  if (ism330is_pin_int1_route_set(&reg_ctx, reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set DRDY mode
* @param  Val the value of drdy_pulsed in reg ISM330IS_DRDY_PULSE_CFG_G
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_DRDY_Mode(uint8_t Val)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_data_ready_mode_t reg;
  reg = (Val == 0U)  ? ISM330IS_DRDY_LATCHED
        :                ISM330IS_DRDY_PULSED;
  if (ism330is_data_ready_mode_set(&reg_ctx, reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set memory bank
* @param  Val the value of memory bank in reg FUNC_CFG_ACCESS
*         0 - ISM330IS_MAIN_MEM_BANK, 2 - ISM330IS_SENSOR_HUB_MEM_BANK, 3 - ISM330IS_ISPU_MEM_BANK
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_Mem_Bank(uint8_t Val)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_mem_bank_t reg;
  reg = (Val == 2U) ? ISM330IS_SENSOR_HUB_MEM_BANK
        : (Val == 3U) ? ISM330IS_ISPU_MEM_BANK
        :               ISM330IS_MAIN_MEM_BANK;
  if (ism330is_mem_bank_set(&reg_ctx, reg) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
  * @}
  */
/** @defgroup ISM330IS_Private_Functions ISM330IS Private Functions
  * @{
  */
/**
* @brief  Set the ISM330IS accelerometer sensor output data rate when enabled
* @param  Odr the functional output data rate to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_X_OutputDataRate_When_Enabled(float_t Odr)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_xl_data_rate_t new_odr;
  new_odr = (Odr <=   12.5f) ? ISM330IS_XL_ODR_AT_12Hz5_HP
            : (Odr <=   26.0f) ? ISM330IS_XL_ODR_AT_26H_HP
            : (Odr <=   52.0f) ? ISM330IS_XL_ODR_AT_52Hz_HP
            : (Odr <=  104.0f) ? ISM330IS_XL_ODR_AT_104Hz_HP
            : (Odr <=  208.0f) ? ISM330IS_XL_ODR_AT_208Hz_HP
            : (Odr <=  416.0f) ? ISM330IS_XL_ODR_AT_416Hz_HP
            : (Odr <=  833.0f) ? ISM330IS_XL_ODR_AT_833Hz_HP
            : (Odr <= 1667.0f) ? ISM330IS_XL_ODR_AT_1667Hz_HP
            : (Odr <= 3333.0f) ? ISM330IS_XL_ODR_AT_3333Hz_HP
            :                    ISM330IS_XL_ODR_AT_6667Hz_HP;
  /* Output data rate selection. */
  if (ism330is_xl_data_rate_set(&reg_ctx, new_odr) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS accelerometer sensor output data rate when disabled
* @param  Odr the functional output data rate to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_X_OutputDataRate_When_Disabled(float_t Odr)
{
  acc_odr = (Odr <=   12.5f) ? ISM330IS_XL_ODR_AT_12Hz5_HP
            : (Odr <=   26.0f) ? ISM330IS_XL_ODR_AT_26H_HP
            : (Odr <=   52.0f) ? ISM330IS_XL_ODR_AT_52Hz_HP
            : (Odr <=  104.0f) ? ISM330IS_XL_ODR_AT_104Hz_HP
            : (Odr <=  208.0f) ? ISM330IS_XL_ODR_AT_208Hz_HP
            : (Odr <=  416.0f) ? ISM330IS_XL_ODR_AT_416Hz_HP
            : (Odr <=  833.0f) ? ISM330IS_XL_ODR_AT_833Hz_HP
            : (Odr <= 1667.0f) ? ISM330IS_XL_ODR_AT_1667Hz_HP
            : (Odr <= 3333.0f) ? ISM330IS_XL_ODR_AT_3333Hz_HP
            :                    ISM330IS_XL_ODR_AT_6667Hz_HP;
  return ISM330IS_OK;
}
/**
* @brief  Set the ISM330IS gyroscope sensor output data rate when enabled
* @param  Odr the functional output data rate to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_G_OutputDataRate_When_Enabled(float_t Odr)
{
  ISM330ISStatusTypeDef ret = ISM330IS_OK;
  ism330is_gy_data_rate_t new_odr;
  new_odr = (Odr <=   12.5f) ? ISM330IS_GY_ODR_AT_12Hz5_HP
            : (Odr <=   26.0f) ? ISM330IS_GY_ODR_AT_26H_HP
            : (Odr <=   52.0f) ? ISM330IS_GY_ODR_AT_52Hz_HP
            : (Odr <=  104.0f) ? ISM330IS_GY_ODR_AT_104Hz_HP
            : (Odr <=  208.0f) ? ISM330IS_GY_ODR_AT_208Hz_HP
            : (Odr <=  416.0f) ? ISM330IS_GY_ODR_AT_416Hz_HP
            : (Odr <=  833.0f) ? ISM330IS_GY_ODR_AT_833Hz_HP
            : (Odr <= 1667.0f) ? ISM330IS_GY_ODR_AT_1667Hz_HP
            : (Odr <= 3333.0f) ? ISM330IS_GY_ODR_AT_3333Hz_HP
            :                    ISM330IS_GY_ODR_AT_6667Hz_HP;
  /* Output data rate selection. */
  if (ism330is_gy_data_rate_set(&reg_ctx, new_odr) != ISM330IS_OK) {
    ret = ISM330IS_ERROR;
  }
  return ret;
}
/**
* @brief  Set the ISM330IS gyroscope sensor output data rate when disabled
* @param  Odr the functional output data rate to be set
* @retval 0 in case of success, an error code otherwise
*/
ISM330ISStatusTypeDef ISM330ISSensor::Set_G_OutputDataRate_When_Disabled(float_t Odr)
{
  gyro_odr = (Odr <=   12.5f) ? ISM330IS_GY_ODR_AT_12Hz5_HP
             : (Odr <=   26.0f) ? ISM330IS_GY_ODR_AT_26H_HP
             : (Odr <=   52.0f) ? ISM330IS_GY_ODR_AT_52Hz_HP
             : (Odr <=  104.0f) ? ISM330IS_GY_ODR_AT_104Hz_HP
             : (Odr <=  208.0f) ? ISM330IS_GY_ODR_AT_208Hz_HP
             : (Odr <=  416.0f) ? ISM330IS_GY_ODR_AT_416Hz_HP
             : (Odr <=  833.0f) ? ISM330IS_GY_ODR_AT_833Hz_HP
             : (Odr <= 1667.0f) ? ISM330IS_GY_ODR_AT_1667Hz_HP
             : (Odr <= 3333.0f) ? ISM330IS_GY_ODR_AT_3333Hz_HP
             :                    ISM330IS_GY_ODR_AT_6667Hz_HP;
  return ISM330IS_OK;
}
int32_t ISM330IS_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((ISM330ISSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}
int32_t ISM330IS_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((ISM330ISSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
