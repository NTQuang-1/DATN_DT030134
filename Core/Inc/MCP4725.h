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

#define MCP4725_Address   0xC0 // consist off Device Code 1100 and Address Devide A2,A1,A0(pdf MCP4725 - p.27
#define MCP4725_Dac_Write 0x40 // Pdf MCP4725.pdf - p.24.

void Set_Voltage( int vout);

#ifdef __cplusplus
}
#endif
#endif
