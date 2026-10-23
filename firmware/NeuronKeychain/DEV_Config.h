// Minimal shim so Waveshare's QMI8658.cpp compiles inside this sketch (replaces their DEV_Config).
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_
#include <Arduino.h>
#include <Wire.h>
#define UBYTE uint8_t
#define UWORD uint16_t
#define UDOUBLE uint32_t
void DEV_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t Value);
void DEV_I2C_Read_nByte(uint8_t addr, uint8_t reg, uint8_t *pData, uint32_t Len);
void DEV_Delay_ms(uint32_t xms);
#endif
