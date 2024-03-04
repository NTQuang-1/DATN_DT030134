/******************************************************************************************
 * @file 		ST7735S																																			***
 * @author	NTQuang																																			***
 * @pinout	
 *																|MCU		---		LCD		|																	***
 *																|PA10		---		CS		|																	***
 *																|PA9		---		RESET	|																	***
 *																|PA8		---		DC		|																	***
 *										(SPI2_MOSI)	|PB15		---		SDA		|																	***
 *										(SPI2_SCK)	|PB13		---		SCL		|																	***
 *																|PB12		---		LED		|																	***
 *																|-------------------|																	***
 *																|VDD		---		5V		|																	***
 *																|GND		---		GND		|																	***
 ******************************************************************************************
 */

#ifndef __ST7735S_H
#define __ST7735S_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "stm32f1xx_hal.h"

/*----------------------------------Begin marco define-----------------------------------*/

#define RESX              GPIO_PIN_RESET	// pdf v1.4 p.23
#define NO_RESX           GPIO_PIN_SET

#define ON_LED            GPIO_PIN_SET
#define OFF_LED           GPIO_PIN_RESET

#define CS_HIGH           GPIOA->BSRR = (1<<10)				// pdf v1.4 p.24
#define CS_LOW            GPIOA->BSRR = (1<<(10+16))

#define DC_HIGH           GPIOA->BSRR = (1<<8)				// Port bit set/reset register CD00171190.pdf -p.172
#define DC_LOW            GPIOA->BSRR = (1<<(8+16))		

/*----------------------------------------------------------------------------------------
 *------------------------------------pdf v1.4 -p.125-------------------------------------
 *----------------------------------------------------------------------------------------
 */
//#define LCD_GAMMA_PREDEFINED_1	(1<<0)	/* Gamma Cuver 1 */
//#define LCD_GAMMA_PREDEFINED_2	(1<<1)	/* Gamma Cuver 2 */
//#define LCD_GAMMA_PREDEFINED_3	(1<<2)	/* Gamma Cuver 3 */
//#define LCD_GAMMA_PREDEFINED_4	(1<<3)	/* Gamma Cuver 4 */

/*----------------------------------------------------------------------------------------
 *------------------------------------pdf v1.4 -p.140-------------------------------------
 *----------------------------------------------------------------------------------------
 */
//#define LCD_TEARING_MODE_V        0		/* Turn on  tearing with V-Blanking */
//#define LCD_TEARING_MODE_VH       1		/* Turn on  tearing with V-Blanking H-Blanking */
//#define LCD_TEARING_MODE_OFF      255	/* Turn off tearing effect line */

/*----------------------------------------------------------------------------------------
 *--------------------------------pdf v1.4 -p.150-----------------------------------------
 *----------------------------------------------------------------------------------------
 */
//#define LCD_PIXEL_FORMAT_444	3	/* 12 bit/pixel */
#define LCD_PIXEL_FORMAT_565	5	/* 16 bit/pixel */
//#define LCD_PIXEL_FORMAT_666	6	/* 18 bit/pixel */
//#define LCD_PIXEL_NO_USE			7	/* no use */

/* System Function commands (pdf v1.4 p5) */
/*----------------------------------------------------------------------------------------
 *-------------------------------System Function commands---------------------------------
 *----------------------------------------------------------------------------------------
 */

//#define LCD_CMD_NOP				0x00
#define LCD_CMD_SWRESET   0x01 /* Software Reset (pdf v1.4 p108) */
//#define LCD_CMD_RDDID			0x04 /* Read Display ID (pdf v1.4 p109) */
//#define LCD_CMD_RDDST			0x09 /* Read Display Status (pdf v1.4 p110) */
//#define LCD_CMD_RDDPM			0x0A /* Read Display Power Mode (pdf v1.4 p112) */
//#define LCD_CMD_RDDMADCTL	0x0B /* Read Display MADCTL (pdf v1.4 p113) */
//#define LCD_CMD_RDDCOLMOD	0x0C /* Read Display Pixel Format (pdf v1.4 p114) */
//#define LCD_CMD_RDDIM			0x0D /* Read Display Image Mode (pdf v1.4 p115) */
//#define LCD_CMD_RDDSM			0x0E /* Read Display Signal Mode (pdf v1.4 p116) */
//#define LCD_CMD_RDDSDR		0x0F /* Read Display Self-Diagnostic Result (pdf v1.4 p118) */
//#define LCD_CMD_SLPIN   	0x10 /* Sleep In (pdf v1.4 p119) */
#define LCD_CMD_SLPOUT    0x11 /* Sleep Out (pdf v1.4 p120) */
//#define LCD_CMD_PTLON			0x12 /* Partial Display Mode On (pdf v1.4 p121) */
#define LCD_CMD_NORON     0x13 /* Normal Display Mode On (pdf v1.4 p.122) */
#define LCD_CMD_INVOFF    0x20 /* Display Inversion Off (pdf v1.4 p123) */
//#define LCD_CMD_INVON   	0x21 /* Display Inversion On (pdf v1.4 p124) */
//#define LCD_CMD_GAMSET  	0x26 /* Gamma Set (pdf v1.4 p125) */
//#define LCD_CMD_DISPOFF 	0x28 /* Display Off (pdf v1.4 p126) */
#define LCD_CMD_DISPON    0x29 /* Display On (pdf v1.4 p127) */
#define LCD_CMD_CASET     0x2A /* Column Address Set (pdf v1.4 p128-129) */
#define LCD_CMD_RASET     0x2B /* Row Address Set (pdf v1.4 p130-131) */
#define LCD_CMD_RAMWR     0x2C /* Memory Write (pdf v1.4 p132) */
//#define LCD_CMD_RGBSET		0x2D /* Color Setting for 4K,65K and 262K (pdf v1.4 p.133) */
//#define LCD_CMD_RAMRD			0x2E /* Memory Read (pdf v1.4 p.134) */
//#define LCD_CMD_PTLAR			0x30 /* Partial Area (pdf v1.4 p.135) */
//#define LCD_CMD_SCRLAR		0x33 /* Scroll Area Set (pdf v1.4 p.137 */
//#define LCD_CMD_TEOFF   	0x34 /* Tearing Effect Line OFF (pdf v1.4 p139) */
//#define LCD_CMD_TEON    	0x35 /* Tearing Effect Line ON (pdf v1.4 p140) */
#define LCD_CMD_MADCTL    0x36 /* Memory Data Access Control (pdf v1.4 p142) */
//#define LCD_CMD_VSCSAD		0x37 /* Vertical Scroll Start Address of RAM (pdf v1.4 p.145 */
//#define LCD_CMD_IDMOFF  	0x38 /* Idle Mode Off (pdf v1.4 p147) */
//#define LCD_CMD_IDMON   	0x39 /* Idle Mode On (pdf v1.4 p148) */
#define LCD_CMD_COLMOD    0x3A /* Interface Pixel Format (pdf v1.4 p150) */
//#define LCD_CMD_RDID1			0xDA /* Read ID1 Value (pdf v1.4 p.151) */
//#define LCD_CMD_RDID2			0xDB /* Read ID2 Value (pdf v1.4 p.152) */
//#define LCD_CMD_RDID3			0xDC /* Read ID3 Value (pdf v1.4 p.154) */

/*  Panel Function Command */
#define LCD_CMD_FRMCTR1   0xB1 /* Frame Rate Control (In normal mode/ Full colors) (pdf v1.4 p.6) */
#define LCD_CMD_FRMCTR2   0xB2 /* Frame Rate Control (In Idle mode/ 8-colors) */
#define LCD_CMD_FRMCTR3   0xB3 /* Frame Rate Control (In Partial mode/ full colors) */
#define LCD_CMD_INVCTR    0xB4 /* Display Inversion Control */
#define LCD_CMD_PWCTR1    0xC0 /* Power control 1 */
#define LCD_CMD_PWCTR2    0xC1 /* Power control 2 */
#define LCD_CMD_PWCTR3    0xC2 /* Power control 3 */
#define LCD_CMD_PWCTR4    0xC3 /* Power control 4 */
#define LCD_CMD_PWCTR5    0xC4 /* Power control 5 */
#define LCD_CMD_VMCTR1    0xC5 /* VCOM Control 1 */
//#define LCD_CMD_VMOFCTR		0xC7 /* VCOM Offset Control */
//#define LCD_CMD_WRID2			0xD1 /* Write ID2 Value */
//#define LCD_CMD_WRID3			0xD2 /* Write ID3 Value */
//#define LCD_CMD_NVFCTR1		0xD9 /* NVM Control Status */
//#define LCD_CMD_NVFCTR2		0xDE /* NVM Read Command */
//#define LCD_CMD_NVFCTR3		0xDF /* NVM Write Command */
#define LCD_CMD_GMCTRP1   0xE0 /* Gamma  (�+�polarity) Correction Characteristics Setting */
#define LCD_CMD_GMCTRN1   0xE1 /* Gamma �-�polarity Correction Characteristics Setting */
//#define LCD_CMD_GCV				0xFC /* Gate Pump Clock Frequency Variable */

/*-------------------------------------End marco define---------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
/*-----------------------------------------Function-------------------------------------------*/
/*--------------------------------------------------------------------------------------------*/

void ST7735S_Init(SPI_HandleTypeDef *__spi);
void ST7735S_sendCommand(uint8_t cmd);
void SPI_Write(uint8_t volatile data);
void ST7735S_setWindow(uint8_t x,uint8_t y,uint8_t w,uint8_t h);
void ST7735S_Clear(void);

/***************************************Display Function***************************************/
//void Nomal_Display(void);
//void Y_mirror_Display(void);
//void X_mirror_Display(void);
//void XY_mirror_Display(void);
//void XY_Change_Display(void);
void XY_Change_Y_mirror_Display(void);
//void XY_Change_X_mirror_Display(void);
//void XY_Change_XY_mirror_Display(void);
/**********************************************************************************************/

/**************************************Control Power Function**********************************/
//void power_level_1(void);
//void power_level_2(void);
//void power_level_3(void);
//void power_level_4(void);
//void power_level_5(void);
//void power_level_6(void);
/**********************************************************************************************/
		
#ifdef __cplusplus	
}
#endif
#endif
/*-------------------------------------------End file-----------------------------------------*/
