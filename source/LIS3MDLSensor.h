 #ifndef __LIS3MDLSensor_H__
 #define __LIS3MDLSensor_H__
 
 #include "MicroBit.h"
 #include "LIS3MDL_MAG_Driver.h"
 
 extern MicroBit uBit;

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

     // TODO: Complete
     uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead) {
      
     }
     uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite) {

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