/*************************************************************************
 * NOTE: 
 * 				+ RTNA	: Readout Time per pixel per line Normal mode A
 *								Thoi gian doc mot diem anh tren mot dong o che do binh thuong A
 *				+ FPA		: Front Porch A (Khoang cach truoc A)
 *				+ BPA		: Back Porch  A (Khoang cach sau   A)
 */

#include "st7735s.h"

void SPI_Write(uint8_t volatile data){
	while(!(SPI2->SR&SPI_SR_TXE));
	SPI2->DR = data;
}

void ST7735S_sendCommand(uint8_t volatile cmd){
	while(!(SPI2->SR & SPI_SR_TXE));
	while(SPI2->SR & SPI_SR_BSY);
	
	DC_LOW;
	SPI_Write(cmd);
	
	while(!(SPI2->SR & SPI_SR_TXE));
	while(SPI2->SR & SPI_SR_BSY);
	DC_HIGH;
}

/**
 * @brief Setting st7735s window
 * @note  
 *              In normal mode                       In Exchange mode                     
 *               weight = 128												   height = 128
 *            ----------------->                    ----------------->
 *            ------------------| |                 ------------------| |
 *            | * * * * * * * * | |                 | * * * * * * * * | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | | height = 160    |                 | | weight = 160
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            |                 | |                 |                 | |
 *            ------------------- V                 ------------------- V
 * @reference ST7735S ver 1.4.pdf p.77
 */
	
void ST7735S_setWindow(uint8_t x,uint8_t y,uint8_t w,uint8_t h){
	// start coordinate(top-left corner)
	ST7735S_sendCommand(LCD_CMD_CASET);
	SPI_Write(0x00);
	SPI_Write(x + 0);	// cong them 0 de x khong bi thay doi boi c�c thao tac truoc do
	SPI_Write(0x00);
	SPI_Write(x + w);
	
	// end coordinate (bottom-right corner)
	ST7735S_sendCommand(LCD_CMD_RASET);
	SPI_Write(0x00);
  SPI_Write(y + 0);
  SPI_Write(0x00);
  SPI_Write((y + h) + 0);
	
	ST7735S_sendCommand(LCD_CMD_RAMWR);
}

void ST7735S_Clear(void){
	#ifdef exchange_display
		ST7735S_setWindow(0,0,160,128);
	#else
		ST7735S_setWindow(0,0,128,160);
	#endif
	
	for(uint16_t i=0;i<40960;i++)
		SPI_Write(0);
}

void ST7735S_Init(void){
	SPI2->CR1 |= SPI_CR1_SPE;
	
	CS_LOW;

  ST7735S_sendCommand(LCD_CMD_SWRESET);
  HAL_Delay(5);
  ST7735S_sendCommand(LCD_CMD_SLPOUT);
  HAL_Delay(5);
  
  
  ST7735S_sendCommand(LCD_CMD_FRMCTR1);
  SPI_Write(0x01);	// RTNA = 1 (Thoi gian doc 1 diem anh / dong la 1 chu ky dao dong)
  SPI_Write(0x2C);	// FPA  = 43(Khoang cach truoc la 44 chu ky dao dong)
  SPI_Write(0x2D);	// BPA  = 44(Khoang cach sau la 45 chu ky dao dong)
  
  
  ST7735S_sendCommand(LCD_CMD_FRMCTR2);
  SPI_Write(0x01);
  SPI_Write(0x2C);
  SPI_Write(0x2D);
  
  
  ST7735S_sendCommand(LCD_CMD_FRMCTR3);
  SPI_Write(0x01);
  SPI_Write(0x2C);
  SPI_Write(0x2D);
  SPI_Write(0x01);
  SPI_Write(0x2C);
  SPI_Write(0x2D);
  
  
  ST7735S_sendCommand(LCD_CMD_INVCTR);
  SPI_Write(0x07);
  
  
  ST7735S_sendCommand(LCD_CMD_PWCTR1);
  SPI_Write(0xA2);	// AVDD = 5v; GVDD = 4.6v
  SPI_Write(0x02);	// VRHN = -4.6v
  SPI_Write(0x84);	// Auto
  
   
  ST7735S_sendCommand(LCD_CMD_PWCTR2);
  SPI_Write(0xC5);	// V25 = 2.4v; VGH = 3*AVDD-0.5; VGL = -10v
  
  
  ST7735S_sendCommand(LCD_CMD_PWCTR3);
  SPI_Write(0x0A);	// BCLK/1;small;Medium Low
  SPI_Write(0x00);	// BCLK/3;BCLK/1;BCLK/1;BCLK/1
  
  
  ST7735S_sendCommand(LCD_CMD_PWCTR4);
  SPI_Write(0x8A);	// BCLK/2;Small;Medium Low
  SPI_Write(0x2A);	// BCLK/3;BCLK/2;BCLK/2;BCLK/2
  
  
  ST7735S_sendCommand(LCD_CMD_PWCTR5);
  SPI_Write(0x8A);	// BCLK/2;Small;Medium Low
  SPI_Write(0xEE);	// BCLK/2;BCLK/2;BCLK/4;BCLK/2
  
  
  ST7735S_sendCommand(LCD_CMD_VMCTR1);
  SPI_Write(0x0E);	// VCOM = -0.775v

  
  ST7735S_sendCommand(LCD_CMD_INVOFF);
  
//	Nomal_Display();
//	Y_mirror_Display();
//	X_mirror_Display();
//	XY_mirror_Display();
//	XY_Change_Display();
	XY_Change_Y_mirror_Display(); //RGB mode + che do man hinh ngang
//	XY_Change_X_mirror_Display();
//	XY_Change_XY_mirror_Display();
  
  ST7735S_sendCommand(LCD_CMD_COLMOD); //Interface Pixel Format
  SPI_Write(LCD_PIXEL_FORMAT_565); //16-bit/pixel
   

  ST7735S_sendCommand(LCD_CMD_CASET);
  SPI_Write(0x00);
  SPI_Write(0x00);
  SPI_Write(0x00);
  SPI_Write(0xA0);
  
  
  ST7735S_sendCommand(LCD_CMD_RASET); //Row address set
  SPI_Write(0x00);
  SPI_Write(0x00);
  SPI_Write(0x00);
  SPI_Write(0x80);
  

  ST7735S_sendCommand(LCD_CMD_GMCTRP1); //Gamm adjustment (+ polarity)
  SPI_Write(0x02);
  SPI_Write(0x1C);
  SPI_Write(0x07);
  SPI_Write(0x12);
  SPI_Write(0x37);
  SPI_Write(0x32);
  SPI_Write(0x29);
  SPI_Write(0x2D);
  SPI_Write(0x29);
  SPI_Write(0x25);
  SPI_Write(0x2B);
  SPI_Write(0x39);
  SPI_Write(0x00);
  SPI_Write(0x01);
  SPI_Write(0x03);
  SPI_Write(0x10);
  
  
  ST7735S_sendCommand(LCD_CMD_GMCTRN1);  //Gamma adjustment(- polarity)
  SPI_Write(0x03);
  SPI_Write(0x1D);
  SPI_Write(0x07);
  SPI_Write(0x06);
  SPI_Write(0x2E);
  SPI_Write(0x2C);
  SPI_Write(0x29);
  SPI_Write(0x2D);
  SPI_Write(0x2E);
  SPI_Write(0x2E);
  SPI_Write(0x37);
  SPI_Write(0x3F);
  SPI_Write(0x00);
  SPI_Write(0x00);
  SPI_Write(0x02);
  SPI_Write(0x10);
  
  ST7735S_sendCommand(LCD_CMD_NORON);
  
  ST7735S_sendCommand(LCD_CMD_DISPON);
  ST7735S_Clear();
}

/*****************************************************************************************************
 ***********************************Display Function (RGB)********************************************
 ***************************************pdf v1.4 p77**************************************************
 *****************************************************************************************************
 */

void Nomal_Display(void){
	// MV = MX = MY = 0
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0x00);
}
void Y_mirror_Display(void){
	// MV = MX = 0; MY = 1
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0x80);
}
void X_mirror_Display(void){
	// MV = 0; MX = 1; MY = 0
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0x40);
}
void XY_mirror_Display(void){
	// MV = 0; MX = 1; MY = 1
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0xC0);
}
void XY_Change_Display(void){
	// MV = 1; MX = 0; MY = 0
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0x20);
}
void XY_Change_Y_mirror_Display(void){
	// MV = 1; MX = 0; MY = 1
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0xA0);
}
void XY_Change_X_mirror_Display(void){
	// MV = 1; MX = 1; MY = 0
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0x60);
}
void XY_Change_XY_mirror_Display(void){
	// MV = 1; MX = 1; MY = 1
	ST7735S_sendCommand(LCD_CMD_MADCTL);
	SPI_Write(0xE0);
}

/*****************************************************************************************************
 *************************************Control Power Function******************************************
 ******************************************pdf v1.4 p.87**********************************************
 *****************************************************************************************************
 */
void power_level_1(void){
	// Normal Mode On (full display), Idle Mode Off, Sleep Out
	// Max = 262,144 colors
}
void power_level_2(void){
	// Partial Mode On, Idle Mode Off, Sleep Out
	// Max = 262,144 colors
}
void power_level_3(void){
	// Normal Mode On (full display), Idle Mode On, Sleep Out.
	// Max = 8 color
}
void power_level_4(void){
	// Partial Mode On, Idle Mode On, Sleep Out
	// Max = 8 color
}
void power_level_5(void){
	// Sleep In Mode
	// Only the MCU interface and memory works with VDDI power supply. Contents of the memory are safe
}
void power_level_6(void){
	// Power Off Mode
	// VDD and VDDI are removed
}
