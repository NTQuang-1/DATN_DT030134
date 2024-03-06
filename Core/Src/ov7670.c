/**
 * @file ov7670.c
 * @author NTQuang
 */
#include <stdlib.h>
#include "ov7670.h"
#include "main.h"

/* <!Color mode> */
#define RGB_MODE
// #define YUV_MODE

/* <!Scale image mode> */
// #define VGA_MODE
// #define QVGA_MODE
#define QQVGA_MODE
// #define CIF_MODE
// #define QCIF_MODE
// #define QQCIF_MODE

struct Reg_Data OV7670_Setting[] = {  // All - 124*2 = 248 byte
	{ DCR_Com7, DCR_Com7_reset },
	{ DCR_Com7, (DCR_Com7_reset & (~DCR_Com7_reset))},
	{ DCR_Clkrc, 0x01 },    //PCLK settings, 15fps

	#if defined(YUV_COLOR)
		{DCR_Com7, 0x00},
	#elif defined(BAY_MODE)
		{DCR_Com7, 0x01},
	#elif defined(RGB_MODE)
		{DCR_Com7, 0x04},
	#elif defined(PBAY_MODE)
		{DCR_Com7, 0x05},
	#endif

	#if defined(VGA_MODE)
		{DCR_Com3, 0x00},
		{DCR_Com14, 0x00},
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x11},
		{DCR_Scaling_pclk_div, 0xF0},
		{DCR_Scaling_pckl_delay, 0x02},
	#elif defined(QVGA_MODE)
		{DCR_Com3, 0x04},
		{DCR_Com14, 0x19},
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x11},
		{DCR_Scaling_pclk_div, 0xF1},
		{DCR_Scaling_pckl_delay, 0x02},
	#elif defined(QQVGA_MODE) // 7
		{DCR_Com3, 0x04},
		{DCR_Com14, 0x1A},
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x22},
		{DCR_Scaling_pclk_div, 0xF2},
		{DCR_Scaling_pckl_delay, 0x02},

	#elif defined(CIF_MODE)
		{DCR_Com3, 0x09},
		{DCR_Com14, 0x11},
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x11},
		{DCR_Scaling_pclk_div, 0xF1},
		{DCR_Scaling_pckl_delay, 0x02},
	#elif defined(QCIF_MODE)
		{DCR_Com3, 0x0C},
		{DCR_Com14, 0x11},
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x11},
		{DCR_Scaling_pclk_div, 0xF1},
		{DCR_Scaling_pckl_delay, 0x52},
	#elif defined(QQCIF_MODE)
		{DCR_Com3, 0x0C},
		{DCR_Com14, 0x11},
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x22},
		{DCR_Scaling_pclk_div, 0xF2},
		{DCR_Scaling_pckl_delay, 0x2A},
	#endif

	{ 0xb0, 0x84 },         //Color mode (Not documented??)

// Hardware window - 6
	{ DCR_Href, 0x80 },	    //HREF
	{ DCR_Hstart, 0x17 },   //HSTART
	{ DCR_Hstop, 0x05 },    //HSTOP
	{ DCR_Vref, 0x0a },     //VREF
	{ DCR_Vstart, 0x02 },   //VSTART
	{ DCR_Vstop, 0x7a },    //VSTOP

// Matrix coefficients - 7
	{ DCR_Mtx1, 0x80 }, /* Matrix coefficient 1 */
	{ DCR_Mtx2, 0x80 }, /* Matrix coefficient 2 */
	{ DCR_Mtx3, 0x00 }, /* Matrix coefficient 3 */
	{ DCR_Mtx4, 0x22 }, /* Matrix coefficient 4 */
	{ DCR_Mtx5, 0x5e }, /* Matrix coefficient 5 */
	{ DCR_Mtx6, 0x80 }, /* Matrix coefficient 6 */
	{ DCR_Mtxs, 0x9e }, /* Matrix coefficient sign */

// Gamma curve values - 16
	{ DCR_Slop, 0x20 },
	{ DCR_Gam1, 0x10 },
	{ DCR_Gam2, 0x1e },
	{ DCR_Gam3, 0x35 },
	{ DCR_Gam4, 0x5a },
	{ DCR_Gam5, 0x69 },
	{ DCR_Gam6, 0x76 },
	{ DCR_Gam7, 0x80 },
	{ DCR_Gam8, 0x88 },
	{ DCR_Gam9, 0x8f },
	{ DCR_Gam10, 0x96 },
	{ DCR_Gam11, 0xa3 },
	{ DCR_Gam12, 0xaf },
	{ DCR_Gam13, 0xc4 },
	{ DCR_Gam14, 0xd7 },
	{ DCR_Gam15, 0xe8 },

// AGC and AEC parameters - 14
	{ DCR_Bd50max, 0x05 },
	{ DCR_Bd60max, 0x07 },
	{ DCR_Aew, 0x95 },
	{ DCR_Aeb, 0x33 },
	{ DCR_Vpt, 0xe3 },
	{ DCR_Haecc1, 0x78 },
	{ DCR_Haecc2, 0x68 },
	{ 0xa1, 0x03 }, // RSV
	{ DCR_Haecc3, 0xd8 },
	{ DCR_Haecc4, 0xd8 },
	{ DCR_Haecc5, 0xf0 },
	{ DCR_Haecc6, 0x90 },
	{ DCR_Haecc7, 0x94 },
	{ DCR_Aech, 0x00 },

// AWB parameters - 20
	{ DCR_AWBC1, 0x0a },
	{ DCR_AWBC2, 0xf0 },
	{ DCR_AWBC3, 0x34 },
	{ DCR_AWBC4, 0x58 },
	{ DCR_AWBC5, 0x28 },
	{ DCR_AWBC6, 0x3a },
	{ 0x59, 0x88 },        // 0x59 -> 0x61 : AWB Control
	{ 0x5a, 0x88 },       
	{ 0x5b, 0x44 },       
	{ 0x5c, 0x67 },       
	{ 0x5d, 0x49 },       
	{ 0x5e, 0x0e },       
	{ DCR_Awbctr3, 0x0a },
	{ DCR_Awbctr2, 0x55 },
	{ DCR_Awbctr1, 0x11 },
	{ DCR_Awbctr0, 0x9f },
	{ DCR_Ggain, 0x40 },
	{ DCR_Blue, 0x40 },
	{ DCR_Red, 0x60 },
	{ DCR_Com8, 0xe7 },

// Additional parameters - 49
	{ DCR_Arblm, 0x11 },
	{ DCR_Edge, 0x00 },
	{ DCR_Reg75, 0x05 },
	{ DCR_Reg76, 0xe1 },
	{ DCR_Dnsth, 0x00 },
	{ DCR_Reg77, 0x01 },
	{ 0xb8, 0x0a },
	{ DCR_Com16, 0x18 },
	{ DCR_Com11, 0x12 },
	{ DCR_Nt_ctrl, 0x88 },
	{ 0x96, 0x00 },
	{ 0x97, 0x30 },
	{ 0x98, 0x20 },
	{ 0x99, 0x30 },
	{ 0x9a, 0x84 },
	{ 0x9b, 0x29 },
	{ 0x9c, 0x03 },
	{ DCR_Bd50st, 0x4c },
	{ DCR_Bd60st, 0x3f },
	{ 0x78, 0x04 },
	{ DCR_Com5, 0x61 },
	{ DCR_Com6, 0x4b },
	{ 0x16, 0x02 },
	{ DCR_Mvfp, 0x00 },
	{ DCR_Adcctr1, 0x02 },
	{ DCR_Adcctr2, 0x91 },
	{ 0x29, 0x07 },
	{ DCR_Chlf, 0x0b },
	{ 0x35, 0x0b },
	{ DCR_Adc, 0x1d },
	{ DCR_Acom, 0x71 },
	{ DCR_Ofon, 0x2a },
	{ DCR_Com12, 0x78 },
	{ 0x4d, 0x40 },
	{ 0x4e, 0x20 },
	{ DCR_Gfix, 0x00 },
	{ DCR_Dblv, 0x3a },
	{ DCR_Reg74, 0x10 },
	{ 0x8d, 0x4f },
	{ 0x8e, 0x00 },
	{ 0x8f, 0x00 },
	{ 0x90, 0x00 },
	{ 0x91, 0x00 },
	{ 0x96, 0x00 },
	{ 0x9a, 0x00 },
	{ DCR_Ablc1, 0x0c },
	{ 0xb2, 0x0e },
	{ DCR_Thl_st, 0x82 },
	{ DCR_Reg4b, 0x01 }, 
};

extern I2C_HandleTypeDef hi2c1;

void OV7670_write(uint8_t *data, uint16_t size, uint32_t timeout){
	HAL_I2C_Master_Transmit(&hi2c1,Slave_WR,data,size,timeout);
}	

void OV7670_Init(I2C_HandleTypeDef *__i2c){
	hi2c1 = *__i2c;
	hi2c1.Instance->CR1 |= I2C_CR1_PE;
	
	uint8_t *temp = (uint8_t*)malloc(248*sizeof(uint8_t));
	for(uint8_t i=0;i<124;i++){
		temp[2*i] = OV7670_Setting[i].reg;
		temp[2*i+1] = OV7670_Setting[i].val;
	}
	OV7670_write(temp,248,1000);
	free(temp);
}

void get_Data(uint8_t *buffer){
	uint16_t count=0;
	while(!PCLK_Pin); // D0-D7 sampling while PCLK rising edge(0 to 1)
	while(!HS_Pin);   // Check HS_Pin == 1
	do {
		buffer[count] |=  D7_STATUS << 7;
		buffer[count] |=  D6_STATUS << 6;
		buffer[count] |=  D5_STATUS << 5;
		buffer[count] |=  D4_STATUS << 5;
		buffer[count] |=  D3_STATUS << 3;
		buffer[count] |=  D2_STATUS << 2;
		buffer[count] |=  D1_STATUS << 1;
		buffer[count] |=  D0_STATUS << 0;
		count++;
	}while((count < (320))); // QQVGA_WIDTH*QQVGA_HEIGHT*2
	
}
