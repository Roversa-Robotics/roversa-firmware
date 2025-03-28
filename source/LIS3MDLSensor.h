 #ifndef __LIS3MDLSensor_H__
 #define __LIS3MDLSensor_H__
 
 #include "MicroBit.h"
 #include "LIS3MDL_MAG_Driver.h"

 #define LIS3MDL_MAG_SENSITIVITY_FOR_FS_4G   0.14f
 #define LIS3MDL_MAG_SENSITIVITY_FOR_FS_8G   0.29f
 #define LIS3MDL_MAG_SENSITIVITY_FOR_FS_12G  0.43f
 #define LIS3MDL_MAG_SENSITIVITY_FOR_FS_16G  0.58f
 
 typedef enum {
   LIS3MDL_STATUS_OK = 0,
   LIS3MDL_STATUS_ERROR,
   LIS3MDL_STATUS_TIMEOUT,
   LIS3MDL_STATUS_NOT_IMPLEMENTED
 } LIS3MDLStatusTypeDef;
 
 class LIS3MDLSensor {
 public:
     LIS3MDLSensor(MicroBitI2C* i2c, uint8_t address = 0x1E);
 
     LIS3MDLStatusTypeDef begin(void);
     LIS3MDLStatusTypeDef end(void);
     LIS3MDLStatusTypeDef Enable(void);
     LIS3MDLStatusTypeDef Disable(void);
     LIS3MDLStatusTypeDef ReadID(uint8_t *p_id);
     LIS3MDLStatusTypeDef GetAxes(int32_t *pData);
     LIS3MDLStatusTypeDef GetSensitivity(float *pfData);
     LIS3MDLStatusTypeDef GetAxesRaw(int16_t *pData);
     LIS3MDLStatusTypeDef GetODR(float *odr);
     LIS3MDLStatusTypeDef SetODR(float odr);
     LIS3MDLStatusTypeDef GetFS(float *fullScale);
     LIS3MDLStatusTypeDef SetFS(float fullScale);
     LIS3MDLStatusTypeDef ReadReg(uint8_t reg, uint8_t *data);
     LIS3MDLStatusTypeDef WriteReg(uint8_t reg, uint8_t data);

     uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead) {
         return dev_i2c->readRegister((uint16_t)address, RegisterAddr, pBuffer, NumByteToRead, true);
     }
     uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite) {
        // NOTE: writeRegister could not be used for this function, as it was lacking in implementation (NumByteToWrite was not implemented in the function, nor was the ability to repeat i2c commands)

        // Allocate temp buffer to hold [RegisterAddr, data...]
        uint8_t temp[NumByteToWrite + 1];
        temp[0] = RegisterAddr;
        for (uint16_t i = 0; i < NumByteToWrite; ++i) {
            temp[i + 1] = pBuffer[i];
        }

        // Write to the I2C and get the result
        return dev_i2c->write((uint16_t)address, temp, NumByteToWrite + 1, false);
     }

  private:
     MicroBitI2C* dev_i2c;
     uint8_t address;
 };

 #ifdef __cplusplus
extern "C" {
#endif
uint8_t LIS3MDL_IO_Write(void* handle, uint8_t reg, uint8_t* pBuffer, uint16_t len);
uint8_t LIS3MDL_IO_Read(void* handle, uint8_t reg, uint8_t* pBuffer, uint16_t len);
#ifdef __cplusplus
}
#endif

 
 #endif // __LIS3MDLSensor_H__

