#include "DEV_Config.h"
void DEV_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t Value) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(Value); Wire.endTransmission();
}
void DEV_I2C_Read_nByte(uint8_t addr, uint8_t reg, uint8_t *pData, uint32_t Len) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission();
  Wire.requestFrom((uint16_t)addr, (uint32_t)Len, true);
  for (uint32_t i = 0; i < Len && Wire.available(); i++) pData[i] = Wire.read();
}
void DEV_Delay_ms(uint32_t xms) { delay(xms); }
