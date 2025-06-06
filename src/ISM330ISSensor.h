/**
 ******************************************************************************
 * @file    ISM330ISSensor.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    05 June 2025
 * @brief   Abstract Class of a ISM330IS sensor.
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
/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __ISM330ISSensor_H__
#define __ISM330ISSensor_H__
/* Includes ------------------------------------------------------------------*/
/* For compatibility with ESP32 platforms */
#ifdef ESP32
  #ifndef MSBFIRST
    #define MSBFIRST SPI_MSBFIRST
  #endif
#endif
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "ism330is_reg.h"
/* Defines -------------------------------------------------------------------*/
#define ISM330IS_I2C_BUS                 0U
#define ISM330IS_SPI_4WIRES_BUS          1U
#define ISM330IS_SPI_3WIRES_BUS          2U
#define ISM330IS_ACC_SENSITIVITY_FS_2G   0.061f
#define ISM330IS_ACC_SENSITIVITY_FS_4G   0.122f
#define ISM330IS_ACC_SENSITIVITY_FS_8G   0.244f
#define ISM330IS_ACC_SENSITIVITY_FS_16G  0.488f
#define ISM330IS_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define ISM330IS_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define ISM330IS_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define ISM330IS_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define ISM330IS_GYRO_SENSITIVITY_FS_2000DPS  70.000f
/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  ISM330IS_OK = 0,
  ISM330IS_ERROR = -1
} ISM330ISStatusTypeDef;
typedef enum {
  ISM330IS_INT1_PIN,
  ISM330IS_INT2_PIN,
} ISM330IS_SensorIntPin_t;
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} ISM330IS_AxesRaw_t;
typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} ISM330IS_Axes_t;
typedef struct {
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} ISM330IS_Event_Status_t;
/* Class Declaration ---------------------------------------------------------*/
/**
 * Abstract class of a ISM330IS pressure sensor.
 */
class ISM330ISSensor {
  public:
    ISM330ISSensor(TwoWire *i2c, uint8_t address = ISM330IS_I2C_ADD_H);
    ISM330ISSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);
    ISM330ISStatusTypeDef begin();
    ISM330ISStatusTypeDef end();
    ISM330ISStatusTypeDef ReadID(uint8_t *Id);
    ISM330ISStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    ISM330ISStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    ISM330ISStatusTypeDef Set_Interrupt_Latch(uint8_t Status);
    ISM330ISStatusTypeDef Enable_X();
    ISM330ISStatusTypeDef Disable_X();
    ISM330ISStatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    ISM330ISStatusTypeDef Get_X_OutputDataRate(float *Odr);
    ISM330ISStatusTypeDef Set_X_OutputDataRate(float Odr);
    ISM330ISStatusTypeDef Get_X_FullScale(int32_t *FullScale);
    ISM330ISStatusTypeDef Set_X_FullScale(int32_t FullScale);
    ISM330ISStatusTypeDef Get_X_AxesRaw(ISM330IS_AxesRaw_t *Value);
    ISM330ISStatusTypeDef Get_X_Axes(ISM330IS_Axes_t *Acceleration);
    ISM330ISStatusTypeDef Enable_G();
    ISM330ISStatusTypeDef Disable_G();
    ISM330ISStatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    ISM330ISStatusTypeDef Get_G_OutputDataRate(float *Odr);
    ISM330ISStatusTypeDef Set_G_OutputDataRate(float Odr);
    ISM330ISStatusTypeDef Get_G_FullScale(int32_t *FullScale);
    ISM330ISStatusTypeDef Set_G_FullScale(int32_t FullScale);
    ISM330ISStatusTypeDef Get_G_AxesRaw(ISM330IS_AxesRaw_t *Value);
    ISM330ISStatusTypeDef Get_G_Axes(ISM330IS_Axes_t *AngularRate);
    ISM330ISStatusTypeDef Set_X_SelfTest(uint8_t Val);
    ISM330ISStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    ISM330ISStatusTypeDef Get_X_Init_Status(uint8_t *Status);
    ISM330ISStatusTypeDef Set_X_INT1_DRDY(uint8_t Val);
    ISM330ISStatusTypeDef Set_G_SelfTest(uint8_t Val);
    ISM330ISStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    ISM330ISStatusTypeDef Get_G_Init_Status(uint8_t *Status);
    ISM330ISStatusTypeDef Set_G_INT1_DRDY(uint8_t Val);
    ISM330ISStatusTypeDef Set_DRDY_Mode(uint8_t Val);
    ISM330ISStatusTypeDef Set_Mem_Bank(uint8_t Val);
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(cs_pin, LOW);
        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }
        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
        return 0;
      }
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);
        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);
        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }
        return 0;
      }
      return 1;
    }
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(cs_pin, LOW);
        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }
        digitalWrite(cs_pin, HIGH);
        dev_spi->endTransaction();
        return 0;
      }
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }
        dev_i2c->endTransmission(true);
        return 0;
      }
      return 1;
    }
  private:
    ISM330ISStatusTypeDef Set_X_OutputDataRate_When_Enabled(float_t Odr);
    ISM330ISStatusTypeDef Set_X_OutputDataRate_When_Disabled(float_t Odr);
    ISM330ISStatusTypeDef Set_G_OutputDataRate_When_Enabled(float_t Odr);
    ISM330ISStatusTypeDef Set_G_OutputDataRate_When_Disabled(float_t Odr);
    /* Helper classes. */
    TwoWire  *dev_i2c;
    SPIClass *dev_spi;
    /* Configuration */
    uint8_t  address;
    int      cs_pin;
    uint32_t spi_speed;
    uint8_t                 is_initialized;
    uint8_t                 acc_is_enabled;
    uint8_t                 gyro_is_enabled;
    ism330is_xl_data_rate_t acc_odr;
    ism330is_gy_data_rate_t gyro_odr;
    ism330is_ctx_t reg_ctx;
};
#ifdef __cplusplus
extern "C" {
#endif
int32_t ISM330IS_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t ISM330IS_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif
#endif /* __ISM330ISSensor_H__ */
