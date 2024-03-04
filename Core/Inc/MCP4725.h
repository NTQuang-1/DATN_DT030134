/*******************************************************************************
 * @file   : SPI
 * @author : NTQuang
 *******************************************************************************
 */
 
#ifndef __MCP4725_H
#define __MCP4725_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define mcp4725_address 0xC0 //0xC4- other address
#define mcp4725_dac_write  0x40  // Writes data to the DAC

#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void Set_Voltage( int vout);

#ifdef __cplusplus
}
#endif
#endif
