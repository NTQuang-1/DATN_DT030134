#include "MCP4725.h"

void Set_Voltage( int vout)
{
  uint8_t data[3];
  data[0] = mcp4725_dac_write; 
  data[1] = (vout / 16);       // Upper data bits     (D11.D10.D9.D8.D7.D6.D5.D4)
  data[2] = (vout % 16) << 4;  // Lower data bits     (D3.D2.D1.D0.x.x.x.x)
	HAL_I2C_Master_Transmit(&hi2c1, mcp4725_address, data, 3, 100); 
}

//mcp4725_address=0xC0
//mcp4725_dac_write=0x40
