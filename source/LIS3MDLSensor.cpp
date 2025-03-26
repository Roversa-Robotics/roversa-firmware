/**
 ******************************************************************************
 * @file    LIS3MDLSensor.cpp (Microbit-compatible rewrite)
 * @brief   Implementation of LIS3MDL magnetometer sensor using MicroBitI2C
 ******************************************************************************
 */

 #include "LIS3MDLSensor.h"
 #include "MicroBit.h"
 
 extern MicroBit uBit; // External reference to MicroBit instance
 
 LIS3MDLSensor::LIS3MDLSensor(MicroBitI2C* i2c, uint8_t address)
     : dev_i2c(i2c), address(address << 1) // Shifted for MicroBit I2C write/read format
 { }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::begin() {
     if (LIS3MDL_MAG_W_SystemOperatingMode(this, LIS3MDL_MAG_MD_POWER_DOWN) == MEMS_ERROR)
         return LIS3MDL_STATUS_ERROR;
 
     if (LIS3MDL_MAG_W_BlockDataUpdate(this, LIS3MDL_MAG_BDU_ENABLE) == MEMS_ERROR)
         return LIS3MDL_STATUS_ERROR;
 
     if (SetODR(80.0f) == LIS3MDL_STATUS_ERROR)
         return LIS3MDL_STATUS_ERROR;
 
     if (SetFS(4.0f) == LIS3MDL_STATUS_ERROR)
         return LIS3MDL_STATUS_ERROR;
 
     if (LIS3MDL_MAG_W_OperatingModeXY(this, LIS3MDL_MAG_OM_HIGH) == MEMS_ERROR)
         return LIS3MDL_STATUS_ERROR;
 
     if (LIS3MDL_MAG_W_TemperatureSensor(this, LIS3MDL_MAG_TEMP_EN_DISABLE) == MEMS_ERROR)
         return LIS3MDL_STATUS_ERROR;
 
     return LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::end() {
     return (Disable() == LIS3MDL_STATUS_OK) ? LIS3MDL_STATUS_OK : LIS3MDL_STATUS_ERROR;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::Enable() {
     return (LIS3MDL_MAG_W_SystemOperatingMode(this, LIS3MDL_MAG_MD_CONTINUOUS) == MEMS_ERROR)
         ? LIS3MDL_STATUS_ERROR : LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::Disable() {
     return (LIS3MDL_MAG_W_SystemOperatingMode(this, LIS3MDL_MAG_MD_POWER_DOWN) == MEMS_ERROR)
         ? LIS3MDL_STATUS_ERROR : LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::ReadID(uint8_t* p_id) {
     if (!p_id) return LIS3MDL_STATUS_ERROR;
     return (LIS3MDL_MAG_R_WHO_AM_I_(this, p_id) == MEMS_ERROR) ? LIS3MDL_STATUS_ERROR : LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::GetAxes(int32_t* pData) {
     int16_t raw[3];
     float sensitivity = 0;
     if (GetAxesRaw(raw) == LIS3MDL_STATUS_ERROR || GetSensitivity(&sensitivity) == LIS3MDL_STATUS_ERROR)
         return LIS3MDL_STATUS_ERROR;
     for (int i = 0; i < 3; i++) pData[i] = static_cast<int32_t>(raw[i] * sensitivity);
     return LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::GetAxesRaw(int16_t* pData) {
     uint8_t buffer[6];
     if (LIS3MDL_MAG_Get_Magnetic(this, buffer) == MEMS_ERROR) return LIS3MDL_STATUS_ERROR;
     for (int i = 0; i < 3; i++) pData[i] = (int16_t)(buffer[2*i+1] << 8 | buffer[2*i]);
     return LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::GetSensitivity(float* pfData) {
     LIS3MDL_MAG_FS_t fs;
     if (LIS3MDL_MAG_R_FullScale(this, &fs) == MEMS_ERROR) return LIS3MDL_STATUS_ERROR;
     switch (fs) {
         case LIS3MDL_MAG_FS_4Ga:  *pfData = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4G; break;
         case LIS3MDL_MAG_FS_8Ga:  *pfData = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8G; break;
         case LIS3MDL_MAG_FS_12Ga: *pfData = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12G; break;
         case LIS3MDL_MAG_FS_16Ga: *pfData = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16G; break;
         default: *pfData = -1.0f; return LIS3MDL_STATUS_ERROR;
     }
     return LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::GetODR(float* odr) {
     LIS3MDL_MAG_DO_t odr_val;
     if (LIS3MDL_MAG_R_OutputDataRate(this, &odr_val) == MEMS_ERROR) return LIS3MDL_STATUS_ERROR;
     switch (odr_val) {
         case LIS3MDL_MAG_DO_0_625Hz: *odr = 0.625f; break;
         case LIS3MDL_MAG_DO_1_25Hz:  *odr = 1.25f; break;
         case LIS3MDL_MAG_DO_2_5Hz:   *odr = 2.5f; break;
         case LIS3MDL_MAG_DO_5Hz:     *odr = 5.0f; break;
         case LIS3MDL_MAG_DO_10Hz:    *odr = 10.0f; break;
         case LIS3MDL_MAG_DO_20Hz:    *odr = 20.0f; break;
         case LIS3MDL_MAG_DO_40Hz:    *odr = 40.0f; break;
         case LIS3MDL_MAG_DO_80Hz:    *odr = 80.0f; break;
         default: return LIS3MDL_STATUS_ERROR;
     }
     return LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::SetODR(float odr) {
     LIS3MDL_MAG_DO_t odr_val;
     if (odr <= 0.625f) odr_val = LIS3MDL_MAG_DO_0_625Hz;
     else if (odr <= 1.25f) odr_val = LIS3MDL_MAG_DO_1_25Hz;
     else if (odr <= 2.5f) odr_val = LIS3MDL_MAG_DO_2_5Hz;
     else if (odr <= 5.0f) odr_val = LIS3MDL_MAG_DO_5Hz;
     else if (odr <= 10.0f) odr_val = LIS3MDL_MAG_DO_10Hz;
     else if (odr <= 20.0f) odr_val = LIS3MDL_MAG_DO_20Hz;
     else if (odr <= 40.0f) odr_val = LIS3MDL_MAG_DO_40Hz;
     else odr_val = LIS3MDL_MAG_DO_80Hz;
     return (LIS3MDL_MAG_W_OutputDataRate(this, odr_val) == MEMS_ERROR)
         ? LIS3MDL_STATUS_ERROR : LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::GetFS(float* fs) {
     LIS3MDL_MAG_FS_t scale;
     if (LIS3MDL_MAG_R_FullScale(this, &scale) == MEMS_ERROR) return LIS3MDL_STATUS_ERROR;
     switch (scale) {
         case LIS3MDL_MAG_FS_4Ga:  *fs = 4.0f; break;
         case LIS3MDL_MAG_FS_8Ga:  *fs = 8.0f; break;
         case LIS3MDL_MAG_FS_12Ga: *fs = 12.0f; break;
         case LIS3MDL_MAG_FS_16Ga: *fs = 16.0f; break;
         default: return LIS3MDL_STATUS_ERROR;
     }
     return LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::SetFS(float fs) {
     LIS3MDL_MAG_FS_t val;
     if (fs <= 4.0f) val = LIS3MDL_MAG_FS_4Ga;
     else if (fs <= 8.0f) val = LIS3MDL_MAG_FS_8Ga;
     else if (fs <= 12.0f) val = LIS3MDL_MAG_FS_12Ga;
     else val = LIS3MDL_MAG_FS_16Ga;
     return (LIS3MDL_MAG_W_FullScale(this, val) == MEMS_ERROR)
         ? LIS3MDL_STATUS_ERROR : LIS3MDL_STATUS_OK;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::ReadReg(uint8_t reg, uint8_t* data) {
     return (LIS3MDL_IO_Read(this, reg, data, 1) == 0) ? LIS3MDL_STATUS_OK : LIS3MDL_STATUS_ERROR;
 }
 
 LIS3MDLStatusTypeDef LIS3MDLSensor::WriteReg(uint8_t reg, uint8_t data) {
     return (LIS3MDL_IO_Write(this, reg, &data, 1) == 0) ? LIS3MDL_STATUS_OK : LIS3MDL_STATUS_ERROR;
 }
 
 uint8_t LIS3MDL_IO_Read(void* handle, uint8_t reg, uint8_t* pBuffer, uint16_t len) {
     if (!handle || !pBuffer) return 1;
     LIS3MDLSensor* sensor = static_cast<LIS3MDLSensor*>(handle);
     uBit.i2c.write(sensor->address, &reg, 1, true);
     return uBit.i2c.read(sensor->address, pBuffer, len);
 }
 
 uint8_t LIS3MDL_IO_Write(void* handle, uint8_t reg, uint8_t* pBuffer, uint16_t len) {
     if (!handle || !pBuffer) return 1;
     LIS3MDLSensor* sensor = static_cast<LIS3MDLSensor*>(handle);
     uint8_t buffer[len + 1];
     buffer[0] = reg;
     for (uint16_t i = 0; i < len; i++) buffer[i + 1] = pBuffer[i];
     return uBit.i2c.write(sensor->address, buffer, len + 1);
 }
 