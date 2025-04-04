#ifndef __LSM6DS3Sensor_H__
#define __LSM6DS3Sensor_H__

#include "MicroBit.h"
#include "LSM6DS3_ACC_GYRO_Driver.h"

#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_2G   0.061f
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G   0.122f
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_8G   0.244f
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_16G  0.488f

#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_125DPS   4.375f
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_245DPS   8.750f
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500f
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000f
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000f

#define LSM6DS3_PEDOMETER_THRESHOLD_LOW       0x00
#define LSM6DS3_PEDOMETER_THRESHOLD_MID_LOW   0x07
#define LSM6DS3_PEDOMETER_THRESHOLD_MID       0x0F
#define LSM6DS3_PEDOMETER_THRESHOLD_MID_HIGH  0x17
#define LSM6DS3_PEDOMETER_THRESHOLD_HIGH      0x1F

#define LSM6DS3_WAKE_UP_THRESHOLD_LOW       0x01
#define LSM6DS3_WAKE_UP_THRESHOLD_MID_LOW   0x0F
#define LSM6DS3_WAKE_UP_THRESHOLD_MID       0x1F
#define LSM6DS3_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
#define LSM6DS3_WAKE_UP_THRESHOLD_HIGH      0x3F

#define LSM6DS3_TAP_THRESHOLD_LOW       0x01
#define LSM6DS3_TAP_THRESHOLD_MID_LOW   0x08
#define LSM6DS3_TAP_THRESHOLD_MID       0x10
#define LSM6DS3_TAP_THRESHOLD_MID_HIGH  0x18
#define LSM6DS3_TAP_THRESHOLD_HIGH      0x1F

#define LSM6DS3_TAP_SHOCK_TIME_LOW       0x00
#define LSM6DS3_TAP_SHOCK_TIME_MID_LOW   0x01
#define LSM6DS3_TAP_SHOCK_TIME_MID_HIGH  0x02
#define LSM6DS3_TAP_SHOCK_TIME_HIGH      0x03

#define LSM6DS3_TAP_QUIET_TIME_LOW       0x00
#define LSM6DS3_TAP_QUIET_TIME_MID_LOW   0x01
#define LSM6DS3_TAP_QUIET_TIME_MID_HIGH  0x02
#define LSM6DS3_TAP_QUIET_TIME_HIGH      0x03

#define LSM6DS3_TAP_DURATION_TIME_LOW       0x00
#define LSM6DS3_TAP_DURATION_TIME_MID_LOW   0x04
#define LSM6DS3_TAP_DURATION_TIME_MID       0x08
#define LSM6DS3_TAP_DURATION_TIME_MID_HIGH  0x0C
#define LSM6DS3_TAP_DURATION_TIME_HIGH      0x0F

typedef enum {
  LSM6DS3_STATUS_OK = 0,
  LSM6DS3_STATUS_ERROR,
  LSM6DS3_STATUS_TIMEOUT,
  LSM6DS3_STATUS_NOT_IMPLEMENTED
} LSM6DS3StatusTypeDef;

typedef struct {
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} LSM6DS3_Event_Status_t;

class LSM6DS3Sensor {
public:
    LSM6DS3Sensor(MicroBitI2C* i2c, uint8_t address = 0x6A);

    LSM6DS3StatusTypeDef begin(void);
    LSM6DS3StatusTypeDef end(void);

    LSM6DS3StatusTypeDef Enable_X(void);
    LSM6DS3StatusTypeDef Disable_X(void);
    LSM6DS3StatusTypeDef Get_X_Axes(int32_t *pData);
    LSM6DS3StatusTypeDef Get_X_AxesRaw(int16_t *pData);
    LSM6DS3StatusTypeDef Get_X_Sensitivity(float *pfData);
    LSM6DS3StatusTypeDef Get_X_ODR(float *odr);
    LSM6DS3StatusTypeDef Set_X_ODR(float odr);
    LSM6DS3StatusTypeDef Get_X_FS(float *fullScale);
    LSM6DS3StatusTypeDef Set_X_FS(float fullScale);

    LSM6DS3StatusTypeDef Enable_G(void);
    LSM6DS3StatusTypeDef Disable_G(void);
    LSM6DS3StatusTypeDef Get_G_Axes(int32_t *pData);
    LSM6DS3StatusTypeDef Get_G_AxesRaw(int16_t *pData);
    LSM6DS3StatusTypeDef Get_G_Sensitivity(float *pfData);
    LSM6DS3StatusTypeDef Get_G_ODR(float *odr);
    LSM6DS3StatusTypeDef Set_G_ODR(float odr);
    LSM6DS3StatusTypeDef Get_G_FS(float *fullScale);
    LSM6DS3StatusTypeDef Set_G_FS(float fullScale);

    LSM6DS3StatusTypeDef ReadID(uint8_t *p_id);
    LSM6DS3StatusTypeDef ReadReg(uint8_t reg, uint8_t *data);
    LSM6DS3StatusTypeDef WriteReg(uint8_t reg, uint8_t data);

    /*
    LSM6DS3StatusTypeDef Enable_Free_Fall_Detection(void);
    LSM6DS3StatusTypeDef Disable_Free_Fall_Detection(void);
    LSM6DS3StatusTypeDef Set_Free_Fall_Threshold(uint8_t thr);
    LSM6DS3StatusTypeDef Enable_Pedometer(void);
    LSM6DS3StatusTypeDef Disable_Pedometer(void);
    LSM6DS3StatusTypeDef Get_Step_Counter(uint16_t *step_count);
    LSM6DS3StatusTypeDef Reset_Step_Counter(void);
    LSM6DS3StatusTypeDef Set_Pedometer_Threshold(uint8_t thr);
    LSM6DS3StatusTypeDef Enable_Tilt_Detection(void);
    LSM6DS3StatusTypeDef Disable_Tilt_Detection(void);
    LSM6DS3StatusTypeDef Enable_Wake_Up_Detection(void);
    LSM6DS3StatusTypeDef Disable_Wake_Up_Detection(void);
    LSM6DS3StatusTypeDef Set_Wake_Up_Threshold(uint8_t thr);
    LSM6DS3StatusTypeDef Enable_Single_Tap_Detection(void);
    LSM6DS3StatusTypeDef Disable_Single_Tap_Detection(void);
    LSM6DS3StatusTypeDef Enable_Double_Tap_Detection(void);
    LSM6DS3StatusTypeDef Disable_Double_Tap_Detection(void);
    LSM6DS3StatusTypeDef Set_Tap_Threshold(uint8_t thr);
    LSM6DS3StatusTypeDef Set_Tap_Shock_Time(uint8_t time);
    LSM6DS3StatusTypeDef Set_Tap_Quiet_Time(uint8_t time);
    LSM6DS3StatusTypeDef Set_Tap_Duration_Time(uint8_t time);
    LSM6DS3StatusTypeDef Enable_6D_Orientation(void);
    LSM6DS3StatusTypeDef Disable_6D_Orientation(void);
    LSM6DS3StatusTypeDef Get_6D_Orientation_XL(uint8_t *xl);
    LSM6DS3StatusTypeDef Get_6D_Orientation_XH(uint8_t *xh);
    LSM6DS3StatusTypeDef Get_6D_Orientation_YL(uint8_t *yl);
    LSM6DS3StatusTypeDef Get_6D_Orientation_YH(uint8_t *yh);
    LSM6DS3StatusTypeDef Get_6D_Orientation_ZL(uint8_t *zl);
    LSM6DS3StatusTypeDef Get_6D_Orientation_ZH(uint8_t *zh);
    LSM6DS3StatusTypeDef Get_Event_Status(LSM6DS3_Event_Status_t *status);
    */

    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead) {
        return dev_i2c->readRegister((uint16_t)address, RegisterAddr, pBuffer, NumByteToRead, true);
    }
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite) {
        uint8_t temp[NumByteToWrite + 1];
        temp[0] = RegisterAddr;
        for (uint16_t i = 0; i < NumByteToWrite; ++i) {
            temp[i + 1] = pBuffer[i];
        }

        return dev_i2c->write((uint16_t)address, temp, NumByteToWrite + 1, false);
    }

private:
    LSM6DS3StatusTypeDef Set_X_ODR_When_Enabled(float odr);
    LSM6DS3StatusTypeDef Set_G_ODR_When_Enabled(float odr);
    LSM6DS3StatusTypeDef Set_X_ODR_When_Disabled(float odr);
    LSM6DS3StatusTypeDef Set_G_ODR_When_Disabled(float odr);

    MicroBitI2C* dev_i2c;
    uint8_t address;
    float X_Last_ODR;
    float G_Last_ODR;
    uint8_t X_isEnabled;
    uint8_t G_isEnabled;
};

#ifdef __cplusplus
extern "C" {
#endif
uint8_t LSM6DS3_IO_Write(void* handle, uint8_t reg, uint8_t* pBuffer, uint16_t len);
uint8_t LSM6DS3_IO_Read(void* handle, uint8_t reg, uint8_t* pBuffer, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif // __LSM6DS3Sensor_H__
