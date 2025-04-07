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


LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Free_Fall_Detection(void)
{
    return Enable_Free_Fall_Detection(LSM6DS3_INT1_PIN);
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Free_Fall_Detection(LSM6DS3_Interrupt_Pin_t int_pin)
{
    // Set Output Data Rate
    if (Set_X_ODR(416.0f) == LSM6DS3_STATUS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    // Set full-scale to 2g (low sensitivity, good for free-fall)
    if (LSM6DS3_ACC_GYRO_W_FS_XL(this, LSM6DS3_ACC_GYRO_FS_XL_2g) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    // Duration (FF_DUR) and related control timers
    if (LSM6DS3_ACC_GYRO_W_FF_Duration(this, 0x06) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_WAKE_DUR(this, 0x00) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_TIMER_HR(this, LSM6DS3_ACC_GYRO_TIMER_HR_6_4ms) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_SLEEP_DUR(this, 0x00) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    // Set threshold for free fall detection
    if (LSM6DS3_ACC_GYRO_W_FF_THS(this, LSM6DS3_ACC_GYRO_FF_THS_10) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    // Route event to INT1 or INT2 pin
    switch (int_pin)
    {
    case LSM6DS3_INT1_PIN:
        if (LSM6DS3_ACC_GYRO_W_FFEvOnInt1(this, LSM6DS3_ACC_GYRO_INT1_FF_ENABLED) == MEMS_ERROR)
            return LSM6DS3_STATUS_ERROR;
        break;

    case LSM6DS3_INT2_PIN:
        if (LSM6DS3_ACC_GYRO_W_FFEvOnInt2(this, LSM6DS3_ACC_GYRO_INT2_FF_ENABLED) == MEMS_ERROR)
            return LSM6DS3_STATUS_ERROR;
        break;

    default:
        return LSM6DS3_STATUS_ERROR;
    }

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_Free_Fall_Detection(void)
{
    if (LSM6DS3_ACC_GYRO_W_FFEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_FF_DISABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_FFEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_FF_DISABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_FF_Duration((void *)this, 0x00) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_FF_THS((void *)this, LSM6DS3_ACC_GYRO_FF_THS_5) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Free_Fall_Threshold(uint8_t thr)
{
    if (LSM6DS3_ACC_GYRO_W_FF_THS((void *)this, (LSM6DS3_ACC_GYRO_FF_THS_t)thr) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Pedometer(void)
{
    if (Set_X_ODR(26.0f) == LSM6DS3_STATUS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (Set_X_FS(2.0f) == LSM6DS3_STATUS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (Set_Pedometer_Threshold(LSM6DS3_PEDOMETER_THRESHOLD_MID_HIGH) == LSM6DS3_STATUS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_FUNC_EN((void *)this, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_PEDO_EN((void *)this, LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1((void *)this, LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_Pedometer(void)
{
    if (LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT1((void *)this, LSM6DS3_ACC_GYRO_INT1_PEDO_DISABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_PEDO_EN((void *)this, LSM6DS3_ACC_GYRO_PEDO_EN_DISABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (LSM6DS3_ACC_GYRO_W_FUNC_EN((void *)this, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    if (Set_Pedometer_Threshold(0x0) == LSM6DS3_STATUS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_Step_Counter(uint16_t *step_count)
{
    if (LSM6DS3_ACC_GYRO_Get_GetStepCounter((void *)this, (uint8_t *)step_count) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Reset_Step_Counter(void)
{
    if (LSM6DS3_ACC_GYRO_W_PedoStepReset((void *)this, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_ENABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    uBit.sleep(10); // required delay

    if (LSM6DS3_ACC_GYRO_W_PedoStepReset((void *)this, LSM6DS3_ACC_GYRO_PEDO_RST_STEP_DISABLED) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Pedometer_Threshold(uint8_t thr)
{
    if (LSM6DS3_ACC_GYRO_W_PedoThreshold((void *)this, thr) == MEMS_ERROR)
        return LSM6DS3_STATUS_ERROR;

    return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Tilt_Detection(void)
{
  return Enable_Tilt_Detection(LSM6DS3_INT1_PIN);
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Tilt_Detection(LSM6DS3_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if (Set_X_ODR(26.0f) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Full scale selection */
  if (Set_X_FS(2.0f) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Enable embedded functionalities */
  if (LSM6DS3_ACC_GYRO_W_FUNC_EN((void *)this, LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Enable tilt calculation */
  if (LSM6DS3_ACC_GYRO_W_TILT_EN((void *)this, LSM6DS3_ACC_GYRO_TILT_EN_ENABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Enable tilt detection on either INT1 or INT2 pin */
  switch (int_pin)
  {
    case LSM6DS3_INT1_PIN:
      if (LSM6DS3_ACC_GYRO_W_TiltEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_TILT_ENABLED) == MEMS_ERROR)
      {
        return LSM6DS3_STATUS_ERROR;
      }
      break;

    case LSM6DS3_INT2_PIN:
      if (LSM6DS3_ACC_GYRO_W_TiltEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_TILT_ENABLED) == MEMS_ERROR)
      {
        return LSM6DS3_STATUS_ERROR;
      }
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_Tilt_Detection(void)
{
  /* Disable tilt event on INT1. */
  if (LSM6DS3_ACC_GYRO_W_TiltEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_TILT_DISABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Disable tilt event on INT2. */
  if (LSM6DS3_ACC_GYRO_W_TiltEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_TILT_DISABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Disable tilt calculation. */
  if (LSM6DS3_ACC_GYRO_W_TILT_EN((void *)this, LSM6DS3_ACC_GYRO_TILT_EN_DISABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Disable embedded functionalities. */
  if (LSM6DS3_ACC_GYRO_W_FUNC_EN((void *)this, LSM6DS3_ACC_GYRO_FUNC_EN_DISABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Wake_Up_Detection(void)
{
  return Enable_Wake_Up_Detection(LSM6DS3_INT2_PIN);
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Wake_Up_Detection(LSM6DS3_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Full scale selection. */
  if (Set_X_FS(2.0f) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* WAKE_DUR setting */
  if (LSM6DS3_ACC_GYRO_W_WAKE_DUR((void *)this, 0x00) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Set wake up threshold. */
  if (LSM6DS3_ACC_GYRO_W_WK_THS((void *)this, 0x02) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Enable wake up detection on either INT1 or INT2 pin */
  switch (int_pin)
  {
    case LSM6DS3_INT1_PIN:
      if (LSM6DS3_ACC_GYRO_W_WUEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_WU_ENABLED) == MEMS_ERROR)
      {
        return LSM6DS3_STATUS_ERROR;
      }
      break;

    case LSM6DS3_INT2_PIN:
      if (LSM6DS3_ACC_GYRO_W_WUEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_WU_ENABLED) == MEMS_ERROR)
      {
        return LSM6DS3_STATUS_ERROR;
      }
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_Wake_Up_Detection(void)
{
  /* Disable wake up event on INT1 */
  if (LSM6DS3_ACC_GYRO_W_WUEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_WU_DISABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* Disable wake up event on INT2 */
  if (LSM6DS3_ACC_GYRO_W_WUEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_WU_DISABLED) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* WU_DUR setting */
  if (LSM6DS3_ACC_GYRO_W_WAKE_DUR((void *)this, 0x00) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  /* WU_THS setting */
  if (LSM6DS3_ACC_GYRO_W_WK_THS((void *)this, 0x00) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Wake_Up_Threshold(uint8_t thr)
{
  if (LSM6DS3_ACC_GYRO_W_WK_THS((void *)this, thr) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Single_Tap_Detection(void)
{
  return Enable_Single_Tap_Detection(LSM6DS3_INT1_PIN);
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Single_Tap_Detection(LSM6DS3_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Full scale selection. */
  if (Set_X_FS(2.0f) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Enable tap directions */
  if (LSM6DS3_ACC_GYRO_W_TAP_X_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (LSM6DS3_ACC_GYRO_W_TAP_Y_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (LSM6DS3_ACC_GYRO_W_TAP_Z_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Set configuration values */
  if (Set_Tap_Threshold(LSM6DS3_TAP_THRESHOLD_MID_LOW) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (Set_Tap_Shock_Time(LSM6DS3_TAP_SHOCK_TIME_MID_HIGH) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (Set_Tap_Quiet_Time(LSM6DS3_TAP_QUIET_TIME_MID_LOW) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Enable tap event on correct pin */
  switch (int_pin)
  {
  case LSM6DS3_INT1_PIN:
    if (LSM6DS3_ACC_GYRO_W_SingleTapOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_ENABLED) == MEMS_ERROR)
      return LSM6DS3_STATUS_ERROR;
    break;

  case LSM6DS3_INT2_PIN:
    if (LSM6DS3_ACC_GYRO_W_SingleTapOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_ENABLED) == MEMS_ERROR)
      return LSM6DS3_STATUS_ERROR;
    break;

  default:
    return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_Single_Tap_Detection(void)
{
  /* Disable single tap interrupt on INT1 pin. */
  if (LSM6DS3_ACC_GYRO_W_SingleTapOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable single tap interrupt on INT2 pin. */
  if (LSM6DS3_ACC_GYRO_W_SingleTapOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap threshold. */
  if (Set_Tap_Threshold(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap shock time window. */
  if (Set_Tap_Shock_Time(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap quiet time window. */
  if (Set_Tap_Quiet_Time(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable Z direction in tap recognition. */
  if (LSM6DS3_ACC_GYRO_W_TAP_Z_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Z_EN_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable Y direction in tap recognition. */
  if (LSM6DS3_ACC_GYRO_W_TAP_Y_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Y_EN_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable X direction in tap recognition. */
  if (LSM6DS3_ACC_GYRO_W_TAP_X_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_X_EN_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Double_Tap_Detection(void)
{
  return Enable_Double_Tap_Detection(LSM6DS3_INT1_PIN);
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_Double_Tap_Detection(LSM6DS3_Interrupt_Pin_t int_pin)
{
  if (Set_X_ODR(416.0f) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  if (Set_X_FS(2.0f) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  if (LSM6DS3_ACC_GYRO_W_TAP_X_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_X_EN_ENABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (LSM6DS3_ACC_GYRO_W_TAP_Y_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Y_EN_ENABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (LSM6DS3_ACC_GYRO_W_TAP_Z_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Z_EN_ENABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  if (Set_Tap_Threshold(LSM6DS3_TAP_THRESHOLD_MID_LOW) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (Set_Tap_Shock_Time(LSM6DS3_TAP_SHOCK_TIME_HIGH) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (Set_Tap_Quiet_Time(LSM6DS3_TAP_QUIET_TIME_HIGH) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;
  if (Set_Tap_Duration_Time(LSM6DS3_TAP_DURATION_TIME_MID) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  if (LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV((void *)this, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  switch (int_pin)
  {
  case LSM6DS3_INT1_PIN:
    if (LSM6DS3_ACC_GYRO_W_TapEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_TAP_ENABLED) == MEMS_ERROR)
      return LSM6DS3_STATUS_ERROR;
    break;

  case LSM6DS3_INT2_PIN:
    if (LSM6DS3_ACC_GYRO_W_TapEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_TAP_ENABLED) == MEMS_ERROR)
      return LSM6DS3_STATUS_ERROR;
    break;

  default:
    return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_Double_Tap_Detection(void)
{
  /* Disable double tap interrupt on INT1 pin. */
  if (LSM6DS3_ACC_GYRO_W_TapEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_TAP_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable double tap interrupt on INT2 pin. */
  if (LSM6DS3_ACC_GYRO_W_TapEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_TAP_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap threshold. */
  if (Set_Tap_Threshold(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap shock time window. */
  if (Set_Tap_Shock_Time(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap quiet time window. */
  if (Set_Tap_Quiet_Time(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset tap duration time window. */
  if (Set_Tap_Duration_Time(0x0) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Only single tap enabled (clears double tap enable bit). */
  if (LSM6DS3_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV((void *)this, LSM6DS3_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable Z direction in tap recognition. */
  if (LSM6DS3_ACC_GYRO_W_TAP_Z_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Z_EN_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable Y direction in tap recognition. */
  if (LSM6DS3_ACC_GYRO_W_TAP_Y_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_Y_EN_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable X direction in tap recognition. */
  if (LSM6DS3_ACC_GYRO_W_TAP_X_EN((void *)this, LSM6DS3_ACC_GYRO_TAP_X_EN_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Tap_Threshold(uint8_t thr)
{
  if (LSM6DS3_ACC_GYRO_W_TAP_THS((void *)this, thr) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }
  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Tap_Shock_Time(uint8_t time)
{
  if (LSM6DS3_ACC_GYRO_W_SHOCK_Duration((void *)this, time) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }
  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Tap_Quiet_Time(uint8_t time)
{
  if (LSM6DS3_ACC_GYRO_W_QUIET_Duration((void *)this, time) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }
  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Set_Tap_Duration_Time(uint8_t time)
{
  if (LSM6DS3_ACC_GYRO_W_DUR((void *)this, time) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }
  return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_6D_Orientation(void)
{
  return Enable_6D_Orientation(LSM6DS3_INT1_PIN);
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Enable_6D_Orientation(LSM6DS3_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if (Set_X_ODR(416.0f) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Full scale selection. */
  if (Set_X_FS(2.0f) == LSM6DS3_STATUS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Set 6D threshold. */
  if (LSM6DS3_ACC_GYRO_W_SIXD_THS((void *)this, LSM6DS3_ACC_GYRO_SIXD_THS_60_degree) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Enable 6D orientation on the desired interrupt pin */
  switch (int_pin)
  {
  case LSM6DS3_INT1_PIN:
    if (LSM6DS3_ACC_GYRO_W_6DEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_6D_ENABLED) == MEMS_ERROR)
      return LSM6DS3_STATUS_ERROR;
    break;

  case LSM6DS3_INT2_PIN:
    if (LSM6DS3_ACC_GYRO_W_6DEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_6D_ENABLED) == MEMS_ERROR)
      return LSM6DS3_STATUS_ERROR;
    break;

  default:
    return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Disable_6D_Orientation(void)
{
  /* Disable 6D orientation interrupt on INT1 pin. */
  if (LSM6DS3_ACC_GYRO_W_6DEvOnInt1((void *)this, LSM6DS3_ACC_GYRO_INT1_6D_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Disable 6D orientation interrupt on INT2 pin. */
  if (LSM6DS3_ACC_GYRO_W_6DEvOnInt2((void *)this, LSM6DS3_ACC_GYRO_INT2_6D_DISABLED) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  /* Reset 6D threshold to default (typically 80 degrees) */
  if (LSM6DS3_ACC_GYRO_W_SIXD_THS((void *)this, LSM6DS3_ACC_GYRO_SIXD_THS_80_degree) == MEMS_ERROR)
    return LSM6DS3_STATUS_ERROR;

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_6D_Orientation_XL(uint8_t *xl)
{
  LSM6DS3_ACC_GYRO_DSD_XL_t xl_raw;

  if (LSM6DS3_ACC_GYRO_R_DSD_XL((void *)this, &xl_raw) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  switch (xl_raw)
  {
    case LSM6DS3_ACC_GYRO_DSD_XL_DETECTED:
      *xl = 1;
      break;

    case LSM6DS3_ACC_GYRO_DSD_XL_NOT_DETECTED:
      *xl = 0;
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_6D_Orientation_XH(uint8_t *xh)
{
  LSM6DS3_ACC_GYRO_DSD_XH_t xh_raw;

  if (LSM6DS3_ACC_GYRO_R_DSD_XH((void *)this, &xh_raw) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  switch (xh_raw)
  {
    case LSM6DS3_ACC_GYRO_DSD_XH_DETECTED:
      *xh = 1;
      break;

    case LSM6DS3_ACC_GYRO_DSD_XH_NOT_DETECTED:
      *xh = 0;
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_6D_Orientation_YL(uint8_t *yl)
{
  LSM6DS3_ACC_GYRO_DSD_YL_t yl_raw;

  if (LSM6DS3_ACC_GYRO_R_DSD_YL((void *)this, &yl_raw) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  switch (yl_raw)
  {
    case LSM6DS3_ACC_GYRO_DSD_YL_DETECTED:
      *yl = 1;
      break;

    case LSM6DS3_ACC_GYRO_DSD_YL_NOT_DETECTED:
      *yl = 0;
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_6D_Orientation_YH(uint8_t *yh)
{
  LSM6DS3_ACC_GYRO_DSD_YH_t yh_raw;

  if (LSM6DS3_ACC_GYRO_R_DSD_YH((void *)this, &yh_raw) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  switch (yh_raw)
  {
    case LSM6DS3_ACC_GYRO_DSD_YH_DETECTED:
      *yh = 1;
      break;

    case LSM6DS3_ACC_GYRO_DSD_YH_NOT_DETECTED:
      *yh = 0;
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_6D_Orientation_ZL(uint8_t *zl)
{
  LSM6DS3_ACC_GYRO_DSD_ZL_t zl_raw;

  if (LSM6DS3_ACC_GYRO_R_DSD_ZL((void *)this, &zl_raw) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  switch (zl_raw)
  {
    case LSM6DS3_ACC_GYRO_DSD_ZL_DETECTED:
      *zl = 1;
      break;

    case LSM6DS3_ACC_GYRO_DSD_ZL_NOT_DETECTED:
      *zl = 0;
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}

LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_6D_Orientation_ZH(uint8_t *zh)
{
  LSM6DS3_ACC_GYRO_DSD_ZH_t zh_raw;

  if (LSM6DS3_ACC_GYRO_R_DSD_ZH((void *)this, &zh_raw) == MEMS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  switch (zh_raw)
  {
    case LSM6DS3_ACC_GYRO_DSD_ZH_DETECTED:
      *zh = 1;
      break;

    case LSM6DS3_ACC_GYRO_DSD_ZH_NOT_DETECTED:
      *zh = 0;
      break;

    default:
      return LSM6DS3_STATUS_ERROR;
  }

  return LSM6DS3_STATUS_OK;
}


LSM6DS3StatusTypeDef LSM6DS3Sensor::Get_Event_Status(LSM6DS3_Event_Status_t *status)
{
  uint8_t Wake_Up_Src = 0, Tap_Src = 0, D6D_Src = 0, Func_Src = 0, Md1_Cfg = 0, Md2_Cfg = 0, Int1_Ctrl = 0;

  memset((void *)status, 0x0, sizeof(LSM6DS3_Event_Status_t));

  if (ReadReg(LSM6DS3_ACC_GYRO_WAKE_UP_SRC, &Wake_Up_Src) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if (ReadReg(LSM6DS3_ACC_GYRO_TAP_SRC, &Tap_Src) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if (ReadReg(LSM6DS3_ACC_GYRO_D6D_SRC, &D6D_Src) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if (ReadReg(LSM6DS3_ACC_GYRO_FUNC_SRC, &Func_Src) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if (ReadReg(LSM6DS3_ACC_GYRO_MD1_CFG, &Md1_Cfg) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if (ReadReg(LSM6DS3_ACC_GYRO_MD2_CFG, &Md2_Cfg) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if (ReadReg(LSM6DS3_ACC_GYRO_INT1_CTRL, &Int1_Ctrl) == LSM6DS3_STATUS_ERROR)
  {
    return LSM6DS3_STATUS_ERROR;
  }

  if ((Md1_Cfg & LSM6DS3_ACC_GYRO_INT1_FF_MASK) || (Md2_Cfg & LSM6DS3_ACC_GYRO_INT2_FF_MASK))
  {
    if ((Wake_Up_Src & LSM6DS3_ACC_GYRO_FF_EV_STATUS_MASK))
    {
      status->FreeFallStatus = 1;
    }
  }

  if ((Md1_Cfg & LSM6DS3_ACC_GYRO_INT1_WU_MASK) || (Md2_Cfg & LSM6DS3_ACC_GYRO_INT2_WU_MASK))
  {
    if ((Wake_Up_Src & LSM6DS3_ACC_GYRO_WU_EV_STATUS_MASK))
    {
      status->WakeUpStatus = 1;
    }
  }

  if ((Md1_Cfg & LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_MASK) || (Md2_Cfg & LSM6DS3_ACC_GYRO_INT2_SINGLE_TAP_MASK))
  {
    if ((Tap_Src & LSM6DS3_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK))
    {
      status->TapStatus = 1;
    }
  }

  if ((Md1_Cfg & LSM6DS3_ACC_GYRO_INT1_TAP_MASK) || (Md2_Cfg & LSM6DS3_ACC_GYRO_INT2_TAP_MASK))
  {
    if ((Tap_Src & LSM6DS3_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK))
    {
      status->DoubleTapStatus = 1;
    }
  }

  if ((Md1_Cfg & LSM6DS3_ACC_GYRO_INT1_6D_MASK) || (Md2_Cfg & LSM6DS3_ACC_GYRO_INT2_6D_MASK))
  {
    if ((D6D_Src & LSM6DS3_ACC_GYRO_D6D_EV_STATUS_MASK))
    {
      status->D6DOrientationStatus = 1;
    }
  }

  if ((Int1_Ctrl & LSM6DS3_ACC_GYRO_INT1_PEDO_MASK))
  {
    if ((Func_Src & LSM6DS3_ACC_GYRO_PEDO_EV_STATUS_MASK))
    {
      status->StepStatus = 1;
    }
  }

  if ((Md1_Cfg & LSM6DS3_ACC_GYRO_INT1_TILT_MASK) || (Md2_Cfg & LSM6DS3_ACC_GYRO_INT2_TILT_MASK))
  {
    if ((Func_Src & LSM6DS3_ACC_GYRO_TILT_EV_STATUS_MASK))
    {
      status->TiltStatus = 1;
    }
  }

  return LSM6DS3_STATUS_OK;
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











