/**
 * @file ov7670.c
 * @author NTQuang
 */
#include <stdlib.h>
#include "main.h"
#include "ov7670.h"
#include "st7735s.h"
#include "ff.h"

extern SPI_HandleTypeDef hspi1;
extern FATFS   fs;      // file system
extern FIL     fil;     // File
extern FRESULT fresult; // result

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

const Reg_Data OV7670_Setting[] = {
	{ DCR_Com7, 0x80 }, // reset all register

	#if defined(YUV_COLOR)
		{DCR_Com7, 0x00},
	#elif defined(BAY_MODE)
		{DCR_Com7, 0x01},
	#elif defined(RGB_MODE)
		{DCR_Com7, 0x04},
		{DCR_Com15, 0xD0}, // Output format RGB565
		{DCR_Rgb444, 0x00}, // Disable RGB444
		{DCR_Com1, 0x00}, // Disable CCIR656
	#elif defined(PBAY_MODE)
		{DCR_Com7, 0x05},
	#endif
		
	// Internal clock 9MHz x 8 / 12 = 6 MHz
	{DCR_Dblv, 0xC0}, // Input clk Mul x8
	{DCR_Clkrc, 0x0B},// Input ckl div 12

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
		
	#elif defined(QQVGA_MODE)
		{DCR_Com3, 0x04}, // DCW enable
		{DCR_Com14, 0x1A},// PCLK div 4
		{DCR_Scaling_xsc, 0x3A},
		{DCR_Scaling_ysc, 0x35},
		{DCR_Scaling_dcwctr, 0x22},// downsample VGA by 4 -> QQVGA
		{DCR_Scaling_pclk_div, 0xF2},// Enable clock divider and Divided by 4
		//{DCR_Scaling_pckl_delay, 0x01},

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
	
	{ DCR_Mvfp, 0x10 }, // Don't mirror, VFlip image

// Hardware window
	
	{ DCR_Tslb, 0x04 },     // avoid sensor auto set window
	{ DCR_Hstart, 0x1E },   //HSTART = 00011110 000 = 240
	{ DCR_Hstop, 0x1E },    //HSTOP  = 00011110 000 = 640-((640-160)/2 + 160)
	{ DCR_Href, 0x00 },	    //HREF   = 00000000
	{ DCR_Vstart, 0x2D },   //VSTART = 00101101 00  = 180
	{ DCR_Vstop, 0x2D },    //VSTOP  = 00101101 00  = 480-((480-120)/2 + 120)
	{ DCR_Vref, 0x00 },     //VREF   = 00000000

// Matrix coefficients, saturation = 0
	{ DCR_Mtx1, 0x80 }, /* Matrix coefficient 1 */
	{ DCR_Mtx2, 0x80 }, /* Matrix coefficient 2 */
	{ DCR_Mtx3, 0x00 }, /* Matrix coefficient 3 */
	{ DCR_Mtx4, 0x22 }, /* Matrix coefficient 4 */
	{ DCR_Mtx5, 0x5E }, /* Matrix coefficient 5 */
	{ DCR_Mtx6, 0x80 }, /* Matrix coefficient 6 */
	{ DCR_Mtxs, 0x9e }, /* Matrix coefficient sign */

// Gamma curve values with gamma 0.666
	{ DCR_Com13, 0x80 }, // Gamma enable
	{ DCR_Slop, 0x2C },  // (256- gam15)*(4/3) = 44
	{ DCR_Gam1, 0x10 },  // 16
	{ DCR_Gam2, 0x19 },  // 25
	{ DCR_Gam3, 0x28 },  // 40
	{ DCR_Gam4, 0x40 },  // 64
	{ DCR_Gam5, 0x4A },  // 74
	{ DCR_Gam6, 0x54 },  // 84
	{ DCR_Gam7, 0x5D },  // 93
	{ DCR_Gam8, 0x66 },  // 102
	{ DCR_Gam9, 0x6E },  // 110
	{ DCR_Gam10, 0x76 }, // 118
	{ DCR_Gam11, 0x85 }, // 133
	{ DCR_Gam12, 0x93 }, // 147
	{ DCR_Gam13, 0xAE }, // 174
	{ DCR_Gam14, 0xC7 }, // 199
	{ DCR_Gam15, 0xDF }, // 223

// AWB parameters- white balance
	{ DCR_Com8, 0xA7 }, // FsstAEC/AGC | AEC step | Band filter 50Hz | AGC enable | AWB enable | AEC enable 
	{ DCR_AWBC1, 0x0a },
	{ DCR_AWBC2, 0xf0 },
	{ DCR_AWBC3, 0x34 },
	{ DCR_AWBC4, 0x58 },
	{ DCR_AWBC5, 0x28 },
	{ DCR_AWBC6, 0x3a },
	{ DCR_AWB7, 0x88 },
	{ DCR_AWB8, 0x88 },       
	{ DCR_AWB9, 0x44 },       
	{ DCR_AWB10, 0x67 },       
	{ DCR_AWB11, 0x49 },       
	{ DCR_AWB12, 0x0e },       
	{ DCR_Awbctr3, 0x0a },
	{ DCR_Awbctr2, 0x55 },
	{ DCR_Awbctr1, 0x11 },
	{ DCR_Awbctr0, 0x9f }, // Advance AWB 
	{ DCR_Ggain, 0x40 },
	{ DCR_Blue, 0x40 },
	{ DCR_Red, 0x40 },
	{ DCR_Com9, 0x68 },  //-----------------------------------
	
	// Bright image
	{ DCR_Bright, 0x10},

	// Contrast image
	{ DCR_Contras, 0x40},
	{ DCR_Contras_Center, 0xff },
	
	// Effect : normal
	{ DCR_Manu, 0xC0 },
	{ DCR_Manv, 0x80 },
	
	// Banding filter 50 Hz
	{ DCR_Com11, 0x1A }, // Select banding filter 50 Hz | enviroment-dependent exposure
	{ DCR_Bd50st, 0x4C }, // 50Hz banding filter value (COM8[5]=1 and COM11[3]=1)
	{ DCR_Bd50max, 0x05 }, // max banding filter step
	{ DCR_Com5, 0x61 },
	{ DCR_Com6, 0x4B },
	{ 0x16, 0x02 },
	{ DCR_Adcctr0, 0x0C }, // ADC range = 1.5, ADC reference = 1
	{ DCR_Adcctr1, 0x02 },
	{ DCR_Adcctr2, 0x91 },
	{ 0x29, 0x07 },
	{ DCR_Chlf, 0x0B },
	{ 0x35, 0x0B },
	{ DCR_Adc, 0x1D },
	{ DCR_Acom, 0x71 }, // ADC and Analog Common Mode Control
	{ DCR_Ofon, 0x2A },
	{ 0x4D, 0x40 }, // DM Pos, dummy row position
	{ 0x4E, 0x20 },
	{ 0x8D, 0x4F },
	{ 0x8E, 0x00 },
	{ 0x8F, 0x00 },
	{ 0x90, 0x00 },
	{ 0x91, 0x00 },
	{ 0x96, 0x00 },
	{ 0x9A, 0x00 },
	{ 0xB0, 0x84 },
	{ DCR_Ablc1, 0x0C }, // ABLC enable
	{ 0xB2, 0x0E },
	{ DCR_Thl_st, 0x82 }, // ABLC target
	{ 0xB8, 0x0A },
	{ DCR_Edge, 0x00 }, // Edge Enhancement adjustment
	{ DCR_Reg74, 0x10 }, // Digital gain control by REG74[1:0], digital gain manual control bypasss
	{ DCR_Reg75, 0x05 }, // edge enhancement lower limit
	{ DCR_Reg76, 0xE1 }, // black pixel correction enable | white pixel corection enable | bit[4:0]: edge enhancement higher limit
	{ DCR_Reg77, 0xcc }, // de-noise offset
	{ DCR_Dnsth, 0x00 }, // De-noise strength
	{ DCR_Reg4b, 0x09 }, // UV average enable
	{ DCR_Satctr, 0x60 }, // Saturation control
	{ DCR_Arblm, 0x11 }, // Array Reference Control

// AGC and AEC parameters 
	{ DCR_Bd50max, 0x05 },
	{ DCR_Bd60max, 0x07 },
	{ DCR_Aew, 0xFF },
	{ DCR_Aeb, 0x00 },
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

// Additional parameters 
	{ DCR_Com16, 0x18 }, 
	{ DCR_Nt_ctrl, 0x88 },
	{ 0x78, 0x04 },
	{ 0x97, 0x30 },
	{ 0x98, 0x20 },
	{ 0x99, 0x30 },
	{ 0x9b, 0x29 },
	{ 0x9c, 0x03 },
	{ DCR_Gfix, 0x00 },
};

/* Private function */
static void I2C_Start(void);
static void I2C_Addr(uint8_t addr);
static void I2C_WriteByte(uint8_t data);
static uint8_t I2C_Read(uint8_t nack);

/**
 * @brief Change interface Mode
 * @note  By default, it operates in slave mode. The interface automatically switches from slave to
 *        master, after it generates a START condition and from master to slave, if an arbitration loss
 *        or a Stop generation occurs, allowing multimaster capability.
 * @reference CD00171190.pdf - p.758
 */
void OV7670_write_reg(uint8_t reg, uint8_t data)
{
	__disable_irq();
	I2C_Start();
	I2C_Addr(Slave_WR);
	I2C_WriteByte(reg);
	I2C_WriteByte(data);
	I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
	__enable_irq();
	delay_us(100);
}

/**
 * @brief OV7670_read_reg function reads data from register of Camera OV7670.
 * @note  Follow the SCCB interface, if user want to read data from Camera.
 *        Before, write data 2 phase, then read data 2 phase.
 */
uint8_t OV7670_read_reg(uint8_t reg)
{
	uint8_t dat;
/**********************************************************************************/
/*                          write data 2 phase                                    */
/**********************************************************************************/
	__disable_irq();
	I2C_Start();
	I2C_Addr(Slave_WR);
	I2C_WriteByte(reg);
	I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
	delay_ms(1);
/**********************************************************************************/
/*                              Read data 2 phase                                 */
/**********************************************************************************/
	I2C_Start();
	I2C_Addr(Slave_RD);
	dat = I2C_Read(1);
	I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
	delay_ms(1);
	__enable_irq();
	return dat;
}

void OV7670_Init(void)
{
	I2C_Start();
	I2C_Addr(Slave_WR);
	I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
	for(uint8_t i=0;i<(sizeof(OV7670_Setting) / 2);i++)
		OV7670_write_reg(OV7670_Setting[i].reg,OV7670_Setting[i].val);
}

/**
 * @brief      This function get a frame from Camera OV7670
 * @parameter 
 *             + uint16_t *buffer : a array using store value get from D0-D7
 *             + uint8_t w        : width of frame (column)
 *             + uint8_t h        : height of frame (row)
 * @note       when sending data to the screen, the high byte must be sent first,
 *             followed by the low byte
 * @document   OV7670_OmniVisionTechnologies.pdf - p.7
 */

void get_frame(uint8_t w, uint8_t h)
{
    uint8_t x, y, temp[2];
    uint32_t frame_size = w * h * 2; // scale a frame (2 byte - 1 pixel)
    uint8_t *buffer = (uint8_t *)malloc(frame_size); // Buffer store data frame
    uint32_t buffer_index = 0;
    uint32_t bytes_written = 0;
    uint32_t bytes_to_write = 0;

    #ifdef exchange_display
        ST7735S_setWindow(0, 0, w, h);
    #else
        ST7735S_setWindow(0, 0, h, w);
    #endif
    while (!VS_STATUS); // wait for frame finish
    while (VS_STATUS);  // start new frame

    y = h;
    while (y--) {
        x = w;
        while (!HS_STATUS); // wait for rising edge of signal HS
        while (x--) {
            while (PCLK_STATUS); // wait for falling edge of signal PCLK
            temp[0] = GPIOA->IDR; SPI_Write(temp[0]); // byte high
            buffer[buffer_index++] = temp[0];
            while (!PCLK_STATUS);
            while (PCLK_STATUS);
            temp[1] = GPIOA->IDR; SPI_Write(temp[1]); // byte low
            buffer[buffer_index++] = temp[1];
            while (!PCLK_STATUS);
        }	
    }

    while (bytes_written < frame_size) {
        bytes_to_write = (frame_size - bytes_written) >= 512 ? 512 : (frame_size - bytes_written);
        f_write(&fil, buffer + bytes_written, bytes_to_write, NULL);
        bytes_written += bytes_to_write;
    }
    free(buffer);
}

void I2C_ReInit(void){
	I2C1->CR1 |=I2C_CR1_SWRST;
	I2C1->CR1 &=~(I2C_CR1_SWRST);
	I2C1->CR2 = 0x20;
	I2C1->CCR = 0xA0;
	I2C1->TRISE = 0x21;
	I2C1->CR1 |= I2C_CR1_PE;
}

static void I2C_Start(void){
	I2C1->CR1 |= (I2C_CR1_PE | I2C_CR1_START);
	I2C1->CR1 &= ~(I2C_CR1_POS | I2C_CR1_SMBUS);
	while(!(I2C1->SR1 & I2C_SR1_SB));
}

static void I2C_Addr(uint8_t addr){
	do{
		I2C1->DR = addr;
		delay_us(100);
		if(I2C1->SR1 & I2C_SR1_AF){
			I2C1->CR1 |= I2C_CR1_STOP | I2C_CR1_ACK;
			I2C_Start();
			I2C1->SR1 &= ~I2C_SR1_AF;
			I2C_Addr(addr);
		}
	}while(I2C1->SR1 & I2C_SR1_AF);
}

static void I2C_WriteByte(uint8_t data){
	if(I2C1->SR2 != 0x7) error_led();
	I2C1->DR = data;
	delay_us(100);
}

static uint8_t I2C_Read(uint8_t nack) {
	if (nack) {
		// if nack = 1, disable ACK (Don't sebd ACK after receive byte)
		I2C1->CR1 |= I2C_CR1_PE;
		I2C1->CR1 &= ~I2C_CR1_ACK;
		delay_ms(1);

		// I2C status
		if ((I2C1->SR2 & (I2C_SR2_BUSY | I2C_SR2_MSL)) == 0x03) {
			// wait data
			while (!(I2C1->SR1 & I2C_SR1_RXNE));
			return I2C1->DR;
		} else {
			error_led();
			return 0;
		}
	} else {
		// if nack = 0, enable ACK (send ACK after receive byte)
		I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
		delay_ms(1);

		// I2c status
		if ((I2C1->SR2 & (I2C_SR2_BUSY | I2C_SR2_MSL)) == 0x03) {
			// wait data
			while (!(I2C1->SR1 & I2C_SR1_RXNE));
			return I2C1->DR;
		} else {
			error_led();
			return 0;
		}
	}
}
