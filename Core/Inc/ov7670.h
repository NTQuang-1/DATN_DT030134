/********************************************************************************
  * @file   OV7670
  * @author NTQuang
  * @pinout Schematic on board Camera OV7670
  *															3.3v -   GND
  *															SCL  -   SDA
	*															VS   -   HS
	*                             PCLK -   MCLK
  *															D7   -   D6
	*															D5   -   D4
	*															D3   -   D2
	*															D1   -   D0
  *															RST  -   PWNN
  * @abbreviation
	*								AGC -	Automatic Gain Control
	*								AEC -	Automatic Exposure Control
	*								AWB -	Automatic White Balance
	*								DCW	-	Downsamp/crop/window
	*								HF	-	Horizontal Frame
	*								VF	-	Vertical Frame
  *******************************************************************************
	* 3.3v			: Analog Power supply
	* GND				: Ground
	* SCL/SIOC	: I2C_SCL
	* SDA/SIOD	: I2C_SDA
	* VS/VSYNC	: Vertical sysc output
	* HS/HREF		: HREF Output
	* PCLK			: Pixel Clock Output
	* MCLK/XCLK	: System Clock Input
	* D7 - D0		: YUV/RGB Bit[7] - Bit[0]
	* RST				: 0 - Reset Mode / 1 - Normal Mode
	* PWNN/PWDN	: 0 - Nomal Mode / 1 - Power Down Mode (Sleep)
	*******************************************************************************
  */

#ifndef __OV7670_H
#define __OV7670_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "stm32f1xx_hal.h"

/********************************************************Begin marco define****************************************************************************/
/*************************************************Addressed Read Write of Camera OV7670****************************************************************/
#define Slave_WR								0x42	// slave write 
#define Slave_RD								0x43	// slave read
/*****************************************************some pin status**********************************************************************************/
#define HS_STATUS    ((GPIOB->IDR & 0x4000)  >> 14)   // pdf CD001711090 - p.171
#define VS_STATUS    ((GPIOB->IDR & 0x800 )  >> 11)
#define PCLK_STATUS  ((GPIOB->IDR & 0x400 )  >> 10)

#define D0_STATUS    ((GPIOA->IDR & 0x20  )  >>  5)
#define D1_STATUS    ((GPIOA->IDR & 0x40  )  >>  6)
#define D2_STATUS    ((GPIOA->IDR & 0x10  )  >>  4)
#define D3_STATUS    ((GPIOA->IDR & 0x80  )  >>  7)
#define D4_STATUS    ((GPIOA->IDR & 0x8   )  >>  3)
#define D5_STATUS    ((GPIOB->IDR & 0x1   )  >>  0)
#define D6_STATUS    ((GPIOA->IDR & 0x4   )  >>  2)
#define D7_STATUS    ((GPIOB->IDR & 0x2   )  >>  1)




/*****************************************************Resolution Mode**********************************************************************************/

// #define VGA_WIDTH               640
// #define VGA_HEIGHT              480
// #define QVGA_WIDTH              320
// #define QVGA_HEIGHT             240
#define QQVGA_WIDTH             160
#define QQVGA_HEIGHT            120
// #define QQQVGA_WIDTH            80
// #define QQQVGA_HEIGHT           60
// #define CIF_WIDTH               352
// #define CIF_HEIGHT              288
// #define QCIF_WIDTH              176
// #define QCIF_HEIGHT             144
// #define QQCIF_WIDTH             88
// #define QQCIF_HEIGHT            72

/**********************************************************Device Control Register List****************************************************************/
#define DCR_Gain                0x00	/* AGC[7:0] - Gain Control Gain Setting */
#define DCR_Blue                0x01	/* AWB - Blue */
#define DCR_Red                 0x02	/* AWB - Red */
#define DCR_Vref                0x03	/* bit[7:6] is AGC[9:8], bit[3:2] = VF[1:0] end, bit[1:0] = VF[1:0] start */
#define DCR_Com1                0x04	/* bit[6]: CCIR656 format, bit[1:0]: AEC[1:0] low 2 LSB */
#define DCR_Bave                0x05	/* U/B(U : Green+Red, B : Blue) Average Level. Automatically updated base on chip output format */
#define DCR_Gbave               0x06	/* Y/Gb(Y: illuminance, Gb: Green) Average Level. Automatically updated base on chip output format */
#define DCR_Aechh               0x07	/* Bit[5:0] : AEC[15:10] */
#define DCR_Rave                0x08	/* V/R(Vertical/Row - Horizontal/column) Average Level. Automatically updated based on chip output format. */
#define DCR_Com2                0x09	/* bit[4] : soft sleep mode, bit[1:0] : output drive capability(00: 1x, 01: 2x, 10:3x, 11: 4x). */
#define DCR_Pid                 0x0A	/* Read ID (MSB) */
#define DCR_Ver                 0x0B	/* Read ID (LSB) */
#define DCR_Com3                0x0C	/* bit[6] : Output data MSB and LSB swap. bit[3] : scale enable(if enable else COM14[3] = 1). bit[2] : DCW enable(if enable else COM14[3] = 1) */
#define DCR_Com4                0x0D	/* bit[5:4]=COM17[7:6] : Average option(00: full window, 01: 1/2, 10: 1/4, 11: 1/4) */
#define DCR_Com5                0x0E	
#define DCR_Com6                0x0F	/* bit[7] : output of optical black line option. bit[1] : reset all timming while format changes. */
#define DCR_Aech                0x10	/* bit[7:0] = AEC[9:2] */
#define DCR_Clkrc               0x11	/* Internal clock, bit[6] : use external clock directly, bit[5:0] : use internal clock = F(input clock)/(bit[5:0]+1). */
#define DCR_Com7                0x12	/* bit[7] : SCCB register reset(1:reset all register to default values, 0:no change), bit[5] : CIF, bit[4] : QVGA, bit[3] : QCIF, bit[2] and bit[0] (00: YUV, 10: RGB, 01: bayer RA, 11: Processed Bayer RAW) */
#define DCR_Com8                0x13	/* bit[7] : Enable fast AGC/AEC algorithm, bit[6] : AEC-step size limit(0: limit, 1: unlimit),  */
#define DCR_Com9                0x14	/* bit[6:4] (000: 2x, 001: 4x, 010: 8x, 011: 16x, 100: 32x, 101: 64x, 110: 128x, 111: Not allowed), bit[0]: freezed AGC/AEC */
#define DCR_Com10               0x15	/* bit[6]: Href changes to Hsync, bit[5]: Pclk output option, bit[4]: PCLK reverse, bit[3]: HREF reverse, bit[2]: Vsync option, bit[1]: VSYNC negative, bit[0]: HSYNC negative */
#define DCR_Hstart              0x17	/* bit[7:0] = HF[10:3] start */
#define DCR_Hstop               0x18	/* bit[7:0] = HF[10:3] end */
#define DCR_Vstart              0x19	/* bit[7:0] = VF[9:2] start */
#define DCR_Vstop               0x1A	/* bit[7:0] = VF[9:2] end */
#define DCR_Pshft               0x1B	/* Data Format - Pixel Delay Select (0x00(no delay) -> 0xFF(delay 256 pixel)) */
#define DCR_Midh                0x1C	/* Read Manufacturer ID byte - High */
#define DCR_Midl                0x1D	/* Read Manufacturer ID byte - Low */
#define DCR_Mvfp                0x1E	/* Mirror/VFlip Enable, bit[5]: Mirror, bit[4]: VFlip, bit[2]: black sun enable */
//#define DCR_Laec                0x1F
//#define DCR_Adcctr0             0x20	/* ADC control: bit[3]: ADC range adjust, bit[2:0]: ADC reference adjust */
#define DCR_Adcctr1             0x21
#define DCR_Adcctr2             0x22
//#define DCR_Adcctr3             0x23
#define DCR_Aew                 0x24	/* AGC/AEC - stable operating region(upper limit) */
#define DCR_Aeb                 0x25	/* AGC/AEC - stable operating region(Lower limit) */
#define DCR_Vpt                 0x26	/* AGC/AEC Fast mode operating region. Bit[7:4]: upper limit, bit[3:0]: lower limit */
//#define DCR_Bbias               0x27	/* B Channel signal output bias(COM6[3]=1), bit[7]: bias adjustment sign, bit[6:0]: bias value of 10-bit range */
//#define DCR_Gbbias              0x28	/* Gb Channel signal output bias(COM6[3]=1),parameter setting same b channel */
//#define DCR_Exhch               0x2A	/* Dummy Pixel Insert MSB */
//#define DCR_Exhcl               0x2B	/* Dummy Pixel Insert LSB */
//#define DCR_Rbias               0x2C	/* R Channel Signal Output Bias(COM6[3]=1), paramete same b chanel */
//#define DCR_Advfl               0x2D	/* LSB of inset dummy lines in Vertical direction */
//#define DCR_AdvfH               0x2E	/* MSB of inset dummy lines in Vertical direction */
//#define DCR_Yave                0x2F	/* Y/G Channel Average value */
#define DCR_Hsyst               0x30	/* Hsync rising edge delay */
#define DCR_Hsyen               0x31	/* Hsync falling edge delay */
#define DCR_Href                0x32	/* bit[7:6]: Href edge offset to data output, bit[5:3] = HF[2:0] end, bit[2:0] = HF[2:0] start */
#define DCR_Chlf                0x33	/* Array Current Control */
#define DCR_Arblm               0x34	/* Array Reference Control */
#define DCR_Adc                 0x37	/* ADC Control */
#define DCR_Acom                0x38	/* ADC and Analog Common Mode Control */
#define DCR_Ofon                0x39	/* ADC Offset Control */
#define DCR_Tslb                0x3A	/* Line Buffer Test option. Bit[5]: Negative image enable, bit[4]: UV output vale, bit[3]: Output senquence(use with register COM13[0], pdf OV7670_omnivisionTechnologies -p.17), bit[0]: auto output window */
#define DCR_Com11               0x3B	/* bit[7]: night mode, bit[6:5]: minimum frame rate of night mode, bit[4]: D56_auto, bit[3]: banding filter value select(COM11[4]=0), bit[1]: enviroment-dependent exposure */
#define DCR_Com12               0x3C	/* bit[7]: Href option */
#define DCR_Com13               0x3D	/* Bit[7]: Gamma enable, bit[6]: UV saturation level - auto adjust, result is saved in SATCTR[3:0], bit[0]: UV swap(use with reg TSLB[3] */
#define DCR_Com14               0x3E	/* bit[4]: DCW and scaling PCLK enable, bit[3]: manual scalling enable as CIF,QCIF,QVGA, bit[2:0]: PCLK divider(only when COM14[4]=1) */
#define DCR_Edge                0x3F	/* Edge Enhancement adjustment. Bit[4:0]: edge enhancement factor */
#define DCR_Com15               0x40	/* Bit[7:6]: data format-output full range enable, bit[5:4]: RGB 555/565 option(COM7[2]=1 and COM7[0]=0) */
#define DCR_Com16               0x41	/* bit[5]: enable edge enhancement threshold auto-adjustment for YUV output, bit[4]: De-noise threshold auto-adjustment, bit[3]: AWB gain enable, bit[1]: color matrix coefficient double option */
#define DCR_Com17               0x42	/* bit[7:6]: AEC window = COM4[5:4], bit[3]: DSP color bar enable */
#define DCR_AWBC1               0x43
#define DCR_AWBC2               0x44
#define DCR_AWBC3               0x45
#define DCR_AWBC4               0x46
#define DCR_AWBC5               0x47
#define DCR_AWBC6               0x48
#define DCR_Reg4b               0x4B	/* bit[0]: UV average enable */
#define DCR_Dnsth               0x4C	/* De-noise strength */
#define DCR_Mtx1                0x4F	/* Matrix coefficient 1 */
#define DCR_Mtx2                0x50	/* Matrix coefficient 2 */
#define DCR_Mtx3                0x51	/* Matrix coefficient 3 */
#define DCR_Mtx4                0x52	/* Matrix coefficient 4 */
#define DCR_Mtx5                0x53	/* Matrix coefficient 5 */
#define DCR_Mtx6                0x54	/* Matrix coefficient 6 */
#define DCR_Bright              0x55	/* Brightness control */
#define DCR_Contras             0x56	/* Contrast control */
//#define DCR_Contras_Center      0x57	/* Contrast center */
#define DCR_Mtxs                0x58	/* Matrix coefficient sign for coefficient 5 to 0, bit[7]: auto contrast center enable, bit[5:0]: Matrix coefficient sign */
#define DCR_AWB7                0x59  /* AWB Control 7 */
#define DCR_AWB8                0x5A  /* AWB Control 8 */
#define DCR_AWB9                0x5B  /* AWB Control 9 */
#define DCR_AWB10               0x5C  /* AWB Control 10 */
#define DCR_AWB11               0x5D  /* AWB Control 11 */
#define DCR_AWB12               0x5E  /* AWB Control 12 */
#define DCR_AWB13               0x5F  /* AWB Control 13 */
#define DCR_AWB14               0x60  /* AWB Control 14 */
#define DCR_AWB15               0x61  /* AWB Control 15 */
// #define DCR_Lcc1                0x62	/* lens correction option 1-X coordinate of lengs correction center relative to array center */
// #define DCR_Lcc2                0x63	/* lens correction option 2-Y coordinate of lengs correction center relative to array center */ 
// #define DCR_Lcc3                0x64	/* Lens correction option 3 G channel compensation coefficient when LLC5[2]=1, R,G,and B channel compensation coefficient when LCC5[2]=0 */
// #define DCR_Lcc4                0x65	/* Lens Correction option 4 - Radius of the circular section where no compensation applies. */
// #define DCR_Lcc5                0x66	/* Bit[2]: Lens correction control select, bit[0]: Lens correction enable */
// #define DCR_Manu                0x67	/* Manual U value (TSLB[4]=1) */
// #define DCR_Manv                0x68	/* Manual V value (TSLB[4]=1) */
#define DCR_Gfix                0x69	/* Fix gain control */
#define DCR_Ggain               0x6A	/* G channen AWbB gain */
#define DCR_Dblv                0x6B	/* bit[7:6]: PLL control, bit[4]: regulator control */
#define DCR_Awbctr3             0x6C	/* AWB ctrl 3 */
#define DCR_Awbctr2             0x6D	/* AWB ctrl 2 */
#define DCR_Awbctr1             0x6E	/* AWB ctrl 1 */
#define DCR_Awbctr0             0x6F	/* AWB ctrl 0 */
#define DCR_Scaling_xsc         0x70	/* bit[7]: test_pattern[0] - work with test_pattern[1](scaling_xsc[7],scaling_ysc[7]), bit[6:0]: horizontal scale factor */
#define DCR_Scaling_ysc         0x71	/* Bit[7]: active same bit[7] of scaling_xsc , bit[6:0]: vertical scale factor */
#define DCR_Scaling_dcwctr      0x72	/* DCW control. Bit[7]: Vertical average calculation option, bit[6]: vertical down sampling option, bit[5:4]: vertical down sampling rate, bit[3]: Horizontal average calculation option, bit[2]: horizontal down sampling option, bit[1:0]: horizontal down sampling rate */
#define DCR_Scaling_pclk_div    0x73	/* bit[3]: bypass clock divider for DSP scale control, bit[2:0]: clock divider for DSP scale control (COM14[3]=1) */
#define DCR_Reg74               0x74	/* bit[4]: DG_Manu, bit[1:0]: digital gain manual control */
#define DCR_Reg75               0x75	/* bit[4:0] edge enhancement lower limit */
#define DCR_Reg76               0x76	/* bit[7]: black pixel correction enable, bit[6]: white pixel corection enable, bit[4:0]: edge enhancement higher limit */
#define DCR_Reg77               0x77	/* bit[7:0]: de-noise offset */
#define DCR_Slop                0x7A	/* Gamma curve highest segment slope - calculated as follows: slop[7:0] = (0x100 - GAM15[7:0])x 4/3 */
#define DCR_Gam1                0x7B
#define DCR_Gam2                0x7C
#define DCR_Gam3                0x7D
#define DCR_Gam4                0x7E
#define DCR_Gam5                0x7F
#define DCR_Gam6                0x80
#define DCR_Gam7                0x81
#define DCR_Gam8                0x82
#define DCR_Gam9                0x83
#define DCR_Gam10               0x84
#define DCR_Gam11               0x85
#define DCR_Gam12               0x86
#define DCR_Gam13               0x87
#define DCR_Gam14               0x88
#define DCR_Gam15               0x89
#define DCR_Rgb444              0x8C	/* RGB 444 control */
//#define DCR_Dm_lnl							0x92	/* dummy line low 8 bit */
//#define DCR_Dm_lnh							0x93	/* dummy line high 8 bit */
//#define DCR_Lcc6								0x94	/* Lens correction option 6(LCC5[2]=1) */
//#define DCR_Lcc7								0x95	/* Lens correction option 7(LCC5[2]=1) */
#define DCR_Bd50st              0x9D	/* 50Hz banding filter value(COM8[5]=1 and COM11[3]=1) */
#define DCR_Bd60st              0x9E	/* 60Hz banding filter value(COM8[5]=1 and COM11[3]=1) */
#define DCR_Haecc1              0x9F	/* Histogram-based AEC/AGC control 1 */
#define DCR_Haecc2              0xA0	/* Histogram-based AEC/AGC control 2 */
#define DCR_Scaling_pckl_delay  0xA2	/* Pixel clock delay. Bit[6:0] scaling output delay */
#define DCR_Nt_ctrl             0xA4	/* bit[3]: auto frame rate adjustment control , bit[1:0]: Auto frame rate adjustment switch point */
#define DCR_Bd50max             0xA5	/* 50Hz Banding step limit */
#define DCR_Haecc3              0xA6	/* Histogram-based AEC/AGC control 3 */
#define DCR_Haecc4              0xA7	/* Histogram-based AEC/AGC control 4 */
#define DCR_Haecc5              0xA8	/* Histogram-based AEC/AGC control 5 */
#define DCR_Haecc6              0xA9	/* Histogram-based AEC/AGC control 6 */
#define DCR_Haecc7              0xAA	/* bit[7]: AEC algorithm selection */
#define DCR_Bd60max             0xAB	/* 60Hz banding step limit */
//#define DCR_Str_opt             0xAC	/* bit[7]: strobe enable, bit[6]: r/g/b gain controlled, bit[5:4]: xenon mode option, bit[1:0]: mode select */
//#define DCR_Str_r               0xAD	/* R gain for led output frame */
//#define DCR_Str_g               0xAE	/* G gain for led output frame */
//#define DCR_Str_b               0xAF	/* B gain for led output frame */
#define DCR_Ablc1               0xB1	/* bit[2]: ABLC enable */
#define DCR_Thl_st              0xB3	/* ABLC target */
//#define DCR_Thl_dlt             0xB5	/* ABLC stable range */
//#define DCR_Ad_chb              0xBE	/* Blue channel black level compensation. Bit[6]: sign bit, bit[5:0]: blue channel black level compensation */
//#define DCR_Ad_chr              0xBF	/* Red channel black level compensation, bit[6]: sign bit, bit[5:0]: red channel black level compensation */
//#define DCR_Ad_chgb             0xC0	/* Gb channel black level compensation, bit[6]: sign bit, bit[5:0]: Gb channel black level compensation */
//#define DCR_Ad_chgr             0xC1	/* Gr channel black level compensation, bit[6]: sign bit, bit[5:0]: Gr channel black level compensation */
#define DCR_Satctr              0xC9	/* Saturation control, Bit[7:4]: UV saturation control min, bit[3:0]: UV saturation control result */

/**************************************************************Configuration Code**********************************************************************/
//#define DCR_Com1_CCIR656        0x40 /* CCIR656 enable pdf - p.11 */

//#define DCR_Com2_sleep          0x10

#define DCR_Com3_swap           0x40  // Byte swap
#define DCR_Com3_scale_en       0x08  // Enable scaling
#define DCR_Com3_dcw_en         0x04	// Enable downsamp/crop/window

#define DCR_Clkrc_ext           0x40	// External clock
#define DCR_Clkrc_0             0x00	// Internal clock
#define DCR_Clkrc_2             0x01	// Internal clock / 2
#define DCR_Clkrc_3             0x02	// Internal clock / 3
#define DCR_Clkrc_4             0x03	// Internal clock / 4
#define DCR_Clkrc_5             0x04	// Internal clock / 5
#define DCR_Clkrc_6             0x05	// Internal clock / 6
#define DCR_Clkrc_mask          0x3F	// Mask for Internal clock scale
/**********************************************************************
 * @explain
 * purpose hold bit relate internal clock
 * exemple:
 * DCR_Clkrc(0x40) & DCR_Clkrc_mask(0x3F) = 0b01000000 & 0b00111111
 * -> result = 0b00000000 -> use internal clock
 * if use DCR_Clkrc_3 -> result &= DCR_Clkrc_3 
 *															 = 0b00000000 & 0x00000010
 *											 result	 = 0b00000010 = 0x02 (use internal clock /3)
 */
#define DCR_Com7_reset          0x80	// Reset all register
#define DCR_Com7_fmt_VGA        0x00	// VGA(default) format
#define DCR_Com7_fmt_CIF        0x20	// CIF  format
#define DCR_Com7_fmt_QVGA       0x10	// QVGA format
#define DCR_Com7_fmt_QCIF       0x08	// QCIF format
#define DCR_Com7_YUV            0x00	// YUV  format
#define DCR_Com7_RGB            0x04	// RGB  format
#define DCR_Com7_Ba_RAW	        0x01	// Bayer RAW format
#define DCR_Com7_Pro_Ba_RAW     0x05	// Processed Bayer format
#define DCR_Com7_fmt_mask       0x38	// Mask format

#define DCR_Com8_fast_AEC_en    0x80	// Enable fast AGC/AEC algorithm
#define DCR_Com8_AEC_step       0x40	// Unlimited step size
#define DCR_Com8_b_filter_on    0x20	// Banding filter ON
#define DCR_Com8_AGC_en         0x04	// AGC enable
#define DCR_Com8_AWB_en         0x02	// AWB enable
#define DCR_Com8_AEC_en         0x01	// AEC enable

#define DCR_Com10_HS            0x40	// VREF changes to HSYNC
#define DCR_Com10_PCLK_hb       0x20	// PCLK does not toggle during horizontal blank
#define DCR_Com10_PCLK_rev      0x10	// PCLK reverse
#define DCR_Com10_HREF_rev      0x08	// HREF reverse
#define DCR_Com10_VS_ri         0x04	// VSYNC changes on rising edge of PCLK
#define DCR_Com10_VS_neg        0x02	// VSYNC negative
#define DCR_Com10_HS_neg        0x01	// HSYNC negative

#define DCR_Mvfp_Mirror         0x20	// Mirror image
#define DCR_Mvfp_VFlip          0x10	// Vertical Flip enable

#define DCR_Tslb_Y_last         0x04	// UYVY - VYUY -see com13[0]
//#define DCR_Tslb_auto_ow        0x01	// Sensor automatically sets output window when resolution changes

#define DCR_Com11_night         0x80	// Night mode
#define DCR_Com11_min_fr        0x60	// 1/8 of nomal mode frame rate
#define DCR_Com11_Hz_Auto       0x10	// enable 50/60 Hz auto detection
#define DCR_Com11_50Hz          0x08	// 50Hz select
#define DCR_Com11_exp           0x02	// Exposure depends on light itensity

#define DCR_Com12_HR_en         0x80	// Always has HREF

#define DCR_Com13_gamma_en      0x80	// Gamma enable
#define DCR_Com13_UV_sat        0x40	// UV saturation auto adjust
#define DCR_Com13_UV_swap       0x01	// V before U

#define DCR_Com14_DCW_en        0x10	// DCW and scaling PCLK enable
#define DCR_Com14_DCW_mscl      0x08	// Manual scaling
#define DCR_Com14_PCLK_div_2    0x01	// div by 2
#define DCR_Com14_PCLK_div_4    0x02	// div by 4
#define DCR_Com14_PCLK_div_8    0x03	// div by 8
#define DCR_Com14_PCLK_div_16   0x04	// div by 16

#define DCR_Com15_R10F0         0x00	// Data range 10 to F0
#define DCR_Com15_R01FE         0x80	//            01 to FE
#define DCR_Com15_R00FF         0xC0	//            00 to FF
#define DCR_Com15_RGB565        0x10	// RGB565 output 
#define DCR_Com15_RGB555        0x30	// RGB555 output

#define DCR_Com16_AWB_gain_en   0x08	// AWB gain enable

#define DCR_Com17_AEC_win       0xC0	// AEC window must same com4[5:4]
#define DCR_Com17_DSP_en        0x08	// DSP color bar enable

#define DCR_Scaling_dcwctr_2    0x11	// down sample by 2
#define DCR_Scaling_dcwctr_4    0x22	// down sample by 4
#define DCR_Scaling_dcwctr_8    0x33	// down sample by 8

#define DCR_Scaling_pclk_div_1  0x00	// div by 1
#define DCR_Scaling_pclk_div_2  0x01	// div by 2
#define DCR_Scaling_pclk_div_4  0x02	// div by 4
#define DCR_Scaling_pclk_div_8  0x03	// div by 8
#define DCR_Scaling_pclk_div_16 0x04	// div by 16

/**************************************************************Configuration Color*********************************************************************/
#define DCR_Mtx_len 6

#define DCR_Reg76_black         0x80	// Black pixel correction enable
#define DCR_Reg76_white         0x40	// White pixel correction enable

#define DCR_Rgb444_en           0x02	// RGB 444 enable
#define DCR_Rgb444_RGBx         0x01	// RGB 444 word format


/**************************************************************End marco define************************************************************************/
struct Reg_Data {
	uint8_t reg;
	uint8_t val;
};

/***********************************Begin Function**********************************************/
void OV7670_Init(I2C_HandleTypeDef *__i2c);
void OV7670_write(uint8_t *data, uint16_t size, uint32_t timeout);
void get_Data(uint8_t *buffer);
bool rising_edge_check(void);
/*************************************End Function**********************************************/


#ifdef __cplusplus
}
#endif
#endif
