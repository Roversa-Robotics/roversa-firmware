#include "LSM6DS3Sensor.h"
#include "MicroBit.h"

extern MicroBit uBit;

LSM6DS3Sensor::LSM6DS3Sensor(MicroBitI2C* i2c, uint8_t address)
    : dev_i2c(i2c), address(address << 1), X_isEnabled(0), G_isEnabled(0), X_Last_ODR(104.0f), G_Last_ODR(104.0f)
{ }

LSM6DS3StatusTypeDef LSM6DS3Sensor::begin() {
    if (LSM6DS3_ACC_GYRO_W_IF_Addr_Incr(this, LSM6DS3_ACC_GYRO_IF_INC_ENABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_BDU(this, LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_FIFO_MODE(this, LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (Set_X_FS(2.0f) != LSM6DS3_STATUS_OK || Set_G_FS(2000.0f) != LSM6DS3_STATUS_OK)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::end() {
    return (Disable_X() == LSM6DS3_STATUS_OK && Disable_G() == LSM6DS3_STATUS_OK)
        ? LSM6DS3_STATUS_OK : LSM6DS3_STATUS_ERROR;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_X() {
    if (X_isEnabled) return LSM6DS3_STATUS_OK;
    if (Set_X_ODR_When_Enabled(X_Last_ODR) != LSM6DS3_STATUS_OK) return LSM6DS3_STATUS_ERROR;
    X_isEnabled = 1;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_X() {
    if (!X_isEnabled) return LSM6DS3_STATUS_OK;
    if (LSM6DS3_ACC_GYRO_W_ODR_XL(this, LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;
    X_isEnabled = 0;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_X_Axes(int32_t *pData) {
    int16_t raw[3]; float sensitivity;
    if (Get_X_AxesRaw(raw) != LSM6DS3_STATUS_OK || Get_X_Sensitivity(&sensitivity) != LSM6DS3_STATUS_OK)
        return LSM6DS3_STATUS_ERROR;
    for (int i = 0; i < 3; i++) pData[i] = (int32_t)(raw[i] * sensitivity);
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_X_AxesRaw(int16_t *pData) {
    uint8_t buffer[6];
    if (LSM6DS3_ACC_GYRO_GetRawAccData(this, buffer) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;
    for (int i = 0; i < 3; i++) pData[i] = (int16_t)(buffer[2*i+1] << 8 | buffer[2*i]);
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_X_Sensitivity(float *pfData) {
    LSM6DS3_ACC_GYRO_FS_XL_t fs;
    if (LSM6DS3_ACC_GYRO_R_FS_XL(this, &fs) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;

    switch (fs) {
        case LSM6DS3_ACC_GYRO_FS_XL_2g: *pfData = LSM6DS3_ACC_SENSITIVITY_FOR_FS_2G; break;
        case LSM6DS3_ACC_GYRO_FS_XL_4g: *pfData = LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G; break;
        case LSM6DS3_ACC_GYRO_FS_XL_8g: *pfData = LSM6DS3_ACC_SENSITIVITY_FOR_FS_8G; break;
        case LSM6DS3_ACC_GYRO_FS_XL_16g: *pfData = LSM6DS3_ACC_SENSITIVITY_FOR_FS_16G; break;
        default: return LSM6DS3_STATUS_ERROR;
    }
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_X_ODR(float *odr) {
    LSM6DS3_ACC_GYRO_ODR_XL_t odr_raw;
    u16_t odr_val;

    if (LSM6DS3_ACC_GYRO_R_ODR_XL(this, &odr_raw) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;
    if (LSM6DS3_ACC_GYRO_translate_ODR_XL(odr_raw, &odr_val) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;

    *odr = (float)odr_val;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_X_ODR(float odr) {
    X_Last_ODR = odr;
    return (X_isEnabled ? Set_X_ODR_When_Enabled(odr) : Set_X_ODR_When_Disabled(odr));
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_X_FS(float *fullScale) {
    LSM6DS3_ACC_GYRO_FS_XL_t fs;
    if (LSM6DS3_ACC_GYRO_R_FS_XL(this, &fs) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;

    switch (fs) {
        case LSM6DS3_ACC_GYRO_FS_XL_2g:  *fullScale = 2.0f; break;
        case LSM6DS3_ACC_GYRO_FS_XL_4g:  *fullScale = 4.0f; break;
        case LSM6DS3_ACC_GYRO_FS_XL_8g:  *fullScale = 8.0f; break;
        case LSM6DS3_ACC_GYRO_FS_XL_16g: *fullScale = 16.0f; break;
        default: return LSM6DS3_STATUS_ERROR;
    }
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_X_FS(float fullScale) {
    LSM6DS3_ACC_GYRO_FS_XL_t fs;

    if      (fullScale <= 2.0f)  fs = LSM6DS3_ACC_GYRO_FS_XL_2g;
    else if (fullScale <= 4.0f)  fs = LSM6DS3_ACC_GYRO_FS_XL_4g;
    else if (fullScale <= 8.0f)  fs = LSM6DS3_ACC_GYRO_FS_XL_8g;
    else                         fs = LSM6DS3_ACC_GYRO_FS_XL_16g;

    return (LSM6DS3_ACC_GYRO_W_FS_XL(this, fs) == MEMS_ERROR) ? LSM6DS3_STATUS_ERROR : LSM6DS3_STATUS_OK;
}



LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_G() {
    if (G_isEnabled) return LSM6DS3_STATUS_OK;
    if (Set_G_ODR_When_Enabled(G_Last_ODR) != LSM6DS3_STATUS_OK) return LSM6DS3_STATUS_ERROR;
    G_isEnabled = 1;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_G() {
    if (!G_isEnabled) return LSM6DS3_STATUS_OK;
    if (LSM6DS3_ACC_GYRO_W_ODR_G(this, LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;
    G_isEnabled = 0;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_G_Axes(int32_t *pData) {
    int16_t raw[3]; float sensitivity;
    if (Get_G_AxesRaw(raw) != LSM6DS3_STATUS_OK || Get_G_Sensitivity(&sensitivity) != LSM6DS3_STATUS_OK)
        return LSM6DS3_STATUS_ERROR;
    for (int i = 0; i < 3; i++) pData[i] = (int32_t)(raw[i] * sensitivity);
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_G_AxesRaw(int16_t *pData) {
    uint8_t buffer[6];
    if (LSM6DS3_ACC_GYRO_GetRawGyroData(this, buffer) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;
    for (int i = 0; i < 3; i++) pData[i] = (int16_t)(buffer[2*i+1] << 8 | buffer[2*i]);
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_G_Sensitivity(float *pfData) {
    LSM6DS3_ACC_GYRO_FS_G_t fs;
    if (LSM6DS3_ACC_GYRO_R_FS_G(this, &fs) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;

    switch (fs) {
        case LSM6DS3_ACC_GYRO_FS_G_245dps:  *pfData = LSM6DS3_GYRO_SENSITIVITY_FOR_FS_245DPS; break;
        case LSM6DS3_ACC_GYRO_FS_G_500dps:  *pfData = LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS; break;
        case LSM6DS3_ACC_GYRO_FS_G_1000dps: *pfData = LSM6DS3_GYRO_SENSITIVITY_FOR_FS_1000DPS; break;
        case LSM6DS3_ACC_GYRO_FS_G_2000dps: *pfData = LSM6DS3_GYRO_SENSITIVITY_FOR_FS_2000DPS; break;
        default: return LSM6DS3_STATUS_ERROR;
    }
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_G_ODR(float *odr) {
    LSM6DS3_ACC_GYRO_ODR_G_t odr_raw;
    u16_t odr_val;

    if (LSM6DS3_ACC_GYRO_R_ODR_G(this, &odr_raw) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;
    if (LSM6DS3_ACC_GYRO_translate_ODR_G(odr_raw, &odr_val) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;

    *odr = (float)odr_val;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_G_ODR(float odr) {
    G_Last_ODR = odr;
    return (G_isEnabled ? Set_G_ODR_When_Enabled(odr) : Set_G_ODR_When_Disabled(odr));
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_G_FS(float *fullScale) {
    LSM6DS3_ACC_GYRO_FS_G_t fs;
    if (LSM6DS3_ACC_GYRO_R_FS_G(this, &fs) == MEMS_ERROR) return LSM6DS3_STATUS_ERROR;

    switch (fs) {
        case LSM6DS3_ACC_GYRO_FS_G_245dps:  *fullScale = 245.0f; break;
        case LSM6DS3_ACC_GYRO_FS_G_500dps:  *fullScale = 500.0f; break;
        case LSM6DS3_ACC_GYRO_FS_G_1000dps: *fullScale = 1000.0f; break;
        case LSM6DS3_ACC_GYRO_FS_G_2000dps: *fullScale = 2000.0f; break;
        default: return LSM6DS3_STATUS_ERROR;
    }
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_G_FS(float fullScale) {
    LSM6DS3_ACC_GYRO_FS_G_t fs;

    if      (fullScale <= 245.0f)  fs = LSM6DS3_ACC_GYRO_FS_G_245dps;
    else if (fullScale <= 500.0f)  fs = LSM6DS3_ACC_GYRO_FS_G_500dps;
    else if (fullScale <= 1000.0f) fs = LSM6DS3_ACC_GYRO_FS_G_1000dps;
    else                           fs = LSM6DS3_ACC_GYRO_FS_G_2000dps;

    return (LSM6DS3_ACC_GYRO_W_FS_G(this, fs) == MEMS_ERROR) ? LSM6DS3_STATUS_ERROR : LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_X_ODR_When_Enabled(float odr) {
    LSM6DS3_ACC_GYRO_ODR_XL_t new_odr;

    if      (odr <= 13.0f)   new_odr = LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
    else if (odr <= 26.0f)   new_odr = LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
    else if (odr <= 52.0f)   new_odr = LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
    else if (odr <= 104.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
    else if (odr <= 208.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
    else if (odr <= 416.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
    else if (odr <= 833.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
    else                     new_odr = LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;

    return (LSM6DS3_ACC_GYRO_W_ODR_XL(this, new_odr) == MEMS_ERROR)
        ? LSM6DS3_STATUS_ERROR : LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_X_ODR_When_Disabled(float odr) {
    X_Last_ODR = odr;
    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_G_ODR_When_Enabled(float odr) {
    LSM6DS3_ACC_GYRO_ODR_G_t new_odr;

    if      (odr <= 13.0f)   new_odr = LSM6DS3_ACC_GYRO_ODR_G_13Hz;
    else if (odr <= 26.0f)   new_odr = LSM6DS3_ACC_GYRO_ODR_G_26Hz;
    else if (odr <= 52.0f)   new_odr = LSM6DS3_ACC_GYRO_ODR_G_52Hz;
    else if (odr <= 104.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_G_104Hz;
    else if (odr <= 208.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_G_208Hz;
    else if (odr <= 416.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_G_416Hz;
    else if (odr <= 833.0f)  new_odr = LSM6DS3_ACC_GYRO_ODR_G_833Hz;
    else                     new_odr = LSM6DS3_ACC_GYRO_ODR_G_1660Hz;

    return (LSM6DS3_ACC_GYRO_W_ODR_G(this, new_odr) == MEMS_ERROR)
        ? LSM6DS3_STATUS_ERROR : LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_G_ODR_When_Disabled(float odr) {
    G_Last_ODR = odr;
    return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::ReadID(uint8_t *p_id) {
    if (!p_id) return LSM6DS3_STATUS_ERROR;
    return (LSM6DS3_ACC_GYRO_R_WHO_AM_I(this, p_id) == MEMS_ERROR) ? LSM6DS3_STATUS_ERROR : LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::ReadReg(uint8_t reg, uint8_t *data) {
    return (dev_i2c->readRegister((uint16_t)address, reg, data, 1, false) == 0)
        ? LSM6DS3_STATUS_OK : LSM6DS3_STATUS_ERROR;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return (dev_i2c->write((uint16_t)address, buf, 2, false) == 0)
        ? LSM6DS3_STATUS_OK : LSM6DS3_STATUS_ERROR;
}

uint8_t LSM6DS3_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  return ((LSM6DS3Sensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}

uint8_t LSM6DS3_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
 return ((LSM6DS3Sensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}