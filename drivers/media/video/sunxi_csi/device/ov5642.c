/*
 * drivers/media/video/sunxi_csi/device/ov5642.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * Based on Omnivision OV7670/OV5640 Camera Driver
 * Copyright (C) 2014 AW-SoM Technologies <src@aw-som.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * A V4L2 driver for OV ov5642 cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include <plat/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"

MODULE_AUTHOR("mmcandrew");
MODULE_DESCRIPTION("A low-level driver for OV ov5642 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		1
#if(DEV_DBG_EN == 1)
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][OV5642]"x,##arg)
#else
#define csi_dev_dbg(x,arg...)
#endif
#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][OV5642]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][OV5642]"x,##arg)

#define MCLK (24*1000*1000)
#define VREF_POL	CSI_LOW
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0 for csi0
#define V4L2_IDENT_SENSOR 0x5642

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0

#define REG_TERM 0xff
#define VAL_TERM 0xff


#define REG_ADDR_STEP 2
#define REG_DATA_STEP 1
#define REG_STEP 			(REG_ADDR_STEP+REG_DATA_STEP)

#define REG_WINDOW_START_X_HIGH		0x3800
#define REG_WINDOW_START_X_LOW		0x3801
#define REG_WINDOW_START_Y_HIGH		0x3802
#define REG_WINDOW_START_Y_LOW		0x3803
#define REG_WINDOW_WIDTH_HIGH		0x3804
#define REG_WINDOW_WIDTH_LOW		0x3805
#define REG_WINDOW_HEIGHT_HIGH 		0x3806
#define REG_WINDOW_HEIGHT_LOW		0x3807
#define REG_OUT_WIDTH_HIGH		0x3808
#define REG_OUT_WIDTH_LOW		0x3809
#define REG_OUT_HEIGHT_HIGH		0x380a
#define REG_OUT_HEIGHT_LOW		0x380b
#define REG_OUT_TOTAL_WIDTH_HIGH	0x380c
#define REG_OUT_TOTAL_WIDTH_LOW		0x380d
#define REG_OUT_TOTAL_HEIGHT_HIGH	0x380e
#define REG_OUT_TOTAL_HEIGHT_LOW	0x380f

#define REG_PIXFMT 			0x4300
#define REG_CLKRC 			0x3011
/*
 * define standard resolution.
 * Works currently only for up to 720 lines
 * eg. 320x240, 640x480, 800x600, 1280x720, 2048x720
 */

#define OV5642_WIDTH		1280
#define OV5642_HEIGHT		720
#define OV5642_TOTAL_WIDTH	3200
#define OV5642_TOTAL_HEIGHT	2000
#define OV5642_SENSOR_SIZE_X	2592
#define OV5642_SENSOR_SIZE_Y	1944

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define QSXGA_WIDTH		2590
#define QSXGA_HEIGHT		1944
#define QXGA_WIDTH 		2048
#define QXGA_HEIGHT		1536
#define P1080P_WIDTH		1920
#define P1080P_HEIGHT		1080
#define UXGA_WIDTH		1600
#define UXGA_HEIGHT		1200
#define P720_WIDTH 		1280
#define P720_HEIGHT		720
//SXGA: 1280*960
#define SXGA_WIDTH		1280
#define SXGA_HEIGHT		960
#define HD720_WIDTH 		1280
#define HD720_HEIGHT		720
//XGA: 1024*768
#define XGA_WIDTH		1024
#define XGA_HEIGHT 		768
#define SVGA_WIDTH		800
#define SVGA_HEIGHT 		600
#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240
#define CIF_WIDTH		352
#define CIF_HEIGHT		288
#define QCIF_WIDTH		176
#define	QCIF_HEIGHT		144

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 30

/*
 * The ov5642 sits on i2c with ID 0x78
 */
#define I2C_ADDR 0x78

/* Registers */


/*
 * Information we maintain about a known sensor.
 */
/*
 * Store information about the video data format.
 */
static struct sensor_format_struct {
	__u8 *desc;
	enum v4l2_mbus_pixelcode mbus_code;
	unsigned char reg;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
		.reg		=0x30,
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,
		.reg		=0x31,
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,
		.reg		=0x32,
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,
		.reg		=0x33,
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,
		.reg		=0x00,
		.bpp		= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)
struct snesor_colorfx_struct; /* coming later */
__csi_subdev_info_t ccm_info_con =
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};

struct sensor_info {
	struct v4l2_subdev sd;
	struct sensor_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	int	width;
	int	height;
	int brightness;
	int	contrast;
	int saturation;
	int hue;
	int hflip;
	int vflip;
	int gain;
	int autogain;
	int exp;
	enum v4l2_exposure_auto_type autoexp;
	int autowb;
	enum v4l2_whiteblance wb;
	enum v4l2_colorfx clrfx;
	enum v4l2_flash_mode flash_mode;
	u8 clkrc;			/* Clock divider value */
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}



/*
 * The default register settings
 *
 */

struct regval_list {
	unsigned char reg_num[REG_ADDR_STEP];
	unsigned char value[REG_DATA_STEP];
};
static struct regval_list ov5642_default_regs_init[] = {
	{{0x31,0x03}, {0x93}},  //PCLK select??  (default 0x01)
	{{0x30,0x08}, {0x82}},  //Software reset
	{{0x30,0x08}, {0x02}},   //Normal Mode
	{{0x30,0x17}, {0x7f}}, //FREX Disable. VSYNC,HREF,PCLK,D9,D8,D7, D6 enable
	{{0x30,0x18}, {0xfc}}, //D5,D4,D3,D2, D1,D0 enable
	{{0x38,0x10}, {0xc2}}, //Horizontal and Vertical offset Setting (default)
	{{0x36,0x15}, {0xf0}}, //Analog control regs?????
	
	{{0x30,0x00}, {0x00}},  // system reset 00 enable all blocks
	{{0x30,0x01}, {0x00}},  // system reset 00 enable all blocks
	{{0x30,0x02}, {0x00}},  // system reset 00 enable all blocks
	{{0x30,0x03}, {0x00}},  // system reset 00 enable all blocks
	
	{{0x30,0x00}, {0xf8}}, //System Reset, reset Blocks, 1 reset block (Default:00)
	{{0x30,0x01}, {0x48}}, //System Reset, reset Blocks, 1 reset block (Default:00)
	{{0x30,0x02}, {0x5c}}, //System Reset, reset Blocks, 1 reset block (Default:00)
	{{0x30,0x03}, {0x02}}, //System Reset, reset Blocks, 1 reset block (Default:00)

	{{0x30,0x04}, {0x07}}, // Clock Enable (Default: DF)
	{{0x30,0x05}, {0xff}},// Clock Enable (Default: FF)
	{{0x30,0x06}, {0xff}},// Clock Enable (Default: FF)
	{{0x30,0x07}, {0x07}},// Clock Enable (Default: 3F)
	
	{{0x30,0x11}, {0x08}}, //{{0x30,0x11}, {0x08}},//320x240 8->15fps 10->30fps 
	{{0x30,0x10}, {0x10}}, 
	//{{0x46,0x0c}, {0x22}}, 
	//{{0x38,0x15}, {0x04}}, 
	
	//Sensor Control Registers
	{{0x37,0x0d}, {0x06}}, //Analog Control D, (Default:05) ??
	{{0x37,0x0c}, {0xa0}}, //Analog Control Registers
	{{0x36,0x02}, {0xfc}}, //Analog Control Registers
	{{0x36,0x12}, {0xff}}, //Analog Control Registers
	{{0x36,0x34}, {0xc0}}, //Analog Control Registers
	{{0x36,0x13}, {0x00}}, //Analog Control Registers
	{{0x36,0x05}, {0x7c}}, //Analog Control Registers
	{{0x36,0x21}, {0x09}}, //Analog Control Registers
	{{0x36,0x22}, {0x00}}, //Analog Control Registers
	{{0x36,0x04}, {0x40}}, //Analog Control Registers
	{{0x36,0x03}, {0xa7}}, //Analog Control Registers
	{{0x36,0x03}, {0x27}}, //Analog Control Registers
	
	//Black Level Control Registers
	{{0x40,0x00}, {0x21}}, //BLC Registers, BLC Enabled ??? (Default: 09)
	{{0x40,0x1d}, {0x02}}, //BLC Frame Ctrl, (Default:00)
	
	//Sensor Control Registers
	{{0x36,0x00}, {0x54}}, //Analog Control Registers
	{{0x36,0x05}, {0x04}}, //Analog Control Registers
	{{0x36,0x06}, {0x3f}}, //Analog Control Registers
	
	//Light Frequency Registers
	{{0x3c,0x01}, {0x80}}, //ALFD_Ctrl1, Auto detection on, (Default: 00)
	
	//ISP Control Registers
	{{0x50,0x00}, {0x4f}}, // ISP Control 00 (Default:DF)
	{{0x50,0x20}, {0x04}}, // ISP Reserved ????
	
	//AWB Registers
	{{0x51,0x81}, {0x79}}, //AWB Control 01, (Default: 58)
	{{0x51,0x82}, {0x00}}, //AWB Control 02, (Default: 11)
	{{0x51,0x85}, {0x22}}, //AWB Control 05, (Default: 24)
	{{0x51,0x97}, {0x01}}, //AWB Control 23, (Default: 02)
	
	//ISP Control Registers (enable UV adjust)
	{{0x50,0x01}, {0xff}}, // ISP Control 01 (Default:4F)
	//UV adjust control registers
	{{0x55,0x00}, {0x0a}}, // UV UVAdjust Control 0 (Default:00)
	{{0x55,0x04}, {0x00}}, // UV adj TH2 (Default: 01)
	{{0x55,0x05}, {0x7f}}, // UV adj TH2 (Default: FF)
	
	//ISP Control Registers
	{{0x50,0x80}, {0x08}}, //Even Ctrl 00, (Default: 40)
	
	
	//System and IO pad control Registers
	{{0x30,0x0e}, {0x18}}, //MIPI Control 00,  (Default: 18)
	
	{{0x46,0x10}, {0x00}}, // ????????
	
	//DVP Registers
	{{0x47,0x1d}, {0x05}}, //DVP Control 1D ????? (Default: 00)
	{{0x47,0x08}, {0x06}}, //DVP Control
	
	
	//Sensor Control Registers
	{{0x37,0x10}, {0x10}}, //Analog Control Registers
	{{0x36,0x32}, {0x41}}, //Analog Control Registers
	{{0x37,0x02}, {0x40}}, //Analog Control Registers
	{{0x36,0x20}, {0x37}}, //Analog Control Registers
	{{0x36,0x31}, {0x01}}, //Analog Control Registers

	
	//Timing Control Regisiters
	// 640x480
	//{{0x38,0x08}, {0x02}}, //DVP output horizontal width high byte[11:8]
	//{{0x38,0x09}, {0x80}}, //DVP output horizontal width low byte[7:0]
	//{{0x38,0x0a}, {0x01}}, //DVP output vertical height high byte[11:8]
	//{{0x38,0x0b}, {0xe0}}, //DVP output vertical height low byte[7:0]
	//1280x720
	{{0x38,0x08},{0x05}},
	{{0x38,0x09},{0x00}},
	{{0x38,0x0a},{0x02}},
	{{0x38,0x0b},{0xd0}},

	//Total horizontal size high byte[11:8](Default: 09)
	//Total horizontal size high byte[7:0] (Default: 49)
	{{0x38,0x0e}, {0x07}}, //Total vertical size high byte[11:8](Default: 06)
	{{0x38,0x0f}, {0xd0}}, //Total vertical size high byte[7:0] (Default: 18)
	
	//ISP Control Registers
	{{0x50,0x1f}, {0x00}}, //Format MUX Control (Default: 04)
	{{0x50,0x00}, {0x4f}}, // ISP Control 00,disable LENC correction and Even odd removing (Default:DF)
	
	//Format Control Registers
	{{0x43,0x00}, {0x30}}, //Set to YUV422_YUYV
	
	//AEC/AEG Registers
	{{0x35,0x03}, {0x07}}, //AEC PK Manual, Maual Enable VTS, AGC, AEC (Default: 00)
	{{0x35,0x01}, {0x73}}, //AEC PK Long Expo middle Bits (Default: 00) 
	{{0x35,0x02}, {0x80}}, //AEC PK Long Expo Low Bits (Default: 00)
	{{0x35,0x0b}, {0x00}}, //AEC PK AGC ADJ, Gain Output to sensor low bits (Default: 00)
	{{0x35,0x03}, {0x07}}, //AEC PK Manual, Maual Enable VTS, AGC, AEC (Default: 00)
	{{0x38,0x24}, {0x11}}, //??????????
	{{0x35,0x01}, {0x1e}}, //AEC PK Long Expo middle Bits (Default: 00)
	{{0x35,0x02}, {0x80}}, //AEC PK Long Expo Low Bits (Default: 00)
	{{0x35,0x0b}, {0x7f}}, //AEC PK AGC ADJ, Gain Output to sensor low bits (Default: 00)
	
	//Timing Control Regisiters
	{{0x38,0x0c}, {0x0c}},//Total horizontal size high byte[11:8](Default: 09)
	{{0x38,0x0d}, {0x80}},//Total horizontal size high byte[7:0] (Default: 49)
	{{0x38,0x0e}, {0x03}},//Total vertical size high byte[11:8](Default: 06)
	{{0x38,0x0f}, {0xe8}},//Total vertical size high byte[7:0] (Default: 18)
	
	
	//Power Down Domain AEC/AGC Registers
	{{0x3a,0x0d}, {0x04}},//AEC CTRL 0D, 60hz Max Bands in one frame(Default: 08)
	{{0x3a,0x0e}, {0x03}},//AEC CTRL 0E, 50hz Max Bands in one frame(Default: 06) 
	
	//Timing Control Regisiters
	{{0x38,0x18}, {0xc1}}, //Timing Control 18, Mirror ON, Vertical subsample 1/2 (Default: 80)
	
	//Sensor Control Registers
	{{0x37,0x05}, {0xdb}},//Analog Control Registers
	{{0x37,0x0a}, {0x81}},//Analog Control Registers
	
	//Timing Control Regisiters
	{{0x38,0x01}, {0x80}},//HREF Horizontal Starting point low byte (Default: B4)
	
	//Sensor Control Registers
	{{0x36,0x21}, {0xc7}},//Analog Control Registers
	
	//Timing Control Regisiters
	{{0x38,0x01}, {0x50}}, //HREF Horizontal Starting point low byte (Default: B4)
	{{0x38,0x03}, {0x08}}, //HREF Vertical Starting point low byte (Default: 0A)
	
	{{0x38,0x27}, {0x08}}, //?????
	
	{{0x38,0x10}, {0xc0}},// Horizontal and Vertical offset setting (Default: C2)
	{{0x38,0x04}, {0x05}},// HREF Horizontal Width High Bits (Default: 08)
	{{0x38,0x05}, {0x00}},//HREF Horizontal Width Low Bits (Default: 00)
	//Scale Average Register
	{{0x56,0x82}, {0x05}}, //AVG X END, Horizontal End Position High bits (Default: 00)
	{{0x56,0x83}, {0x00}}, //AVG X END, Horizontal End Position Low bits (Default: 00)
	//Timing Control Regisiters
	{{0x38,0x06}, {0x03}}, // HREF Vertical Width High Bits (Default: 06)
	{{0x38,0x07}, {0xc0}}, // HREF Vertical Width Low Bits (Default: 06)
	//Scale Average Register
	{{0x56,0x86}, {0x03}}, //AVG Y END, Vertical End Position High bits (Default: 00)
	{{0x56,0x87}, {0xc0}}, //AVG Y END, Vertical End Position Low bits (Default: 00)
	
	//Power Down Domain AEC/AGC Registers
	{{0x3a,0x00}, {0x78}}, // AEC CTRL00,Night Mode Disabled (Default: 7C)
	{{0x3a,0x1a}, {0x04}}, //AEC CTRL Registers
	{{0x3a,0x13}, {0x30}}, //AEC CTRL 13,Pre-gain Enabled (Default:10)
	{{0x3a,0x18}, {0x00}}, //AEC Gain Ceiling High Bits (Default: 03)
	{{0x3a,0x19}, {0x7c}}, //AEC Gain Ceiling Low Bits (Default: E0)
	{{0x3a,0x08}, {0x12}}, //AEC B50 Step, 50 hz Band Width High Bits (Default: 0E)
	{{0x3a,0x09}, {0xc0}}, //AEC B50 Step, 50 hz Band Width Low Bits (Default: A0)
	{{0x3a,0x0a}, {0x0f}}, //AEC B60 Step, 60 hz Band Width High Bits (Default: 0C)
	{{0x3a,0x0b}, {0xa0}}, //AEC B60 Step, 60 hz Band Width Low Bits (Default: 30)
	
	//System Control Registers
	{{0x30,0x04}, {0xff}},// Clock Enable 00 (Default: DF)
	
	//AEC/AEG Registers
	{{0x35,0x0c}, {0x07}},//AEC PK VTS, Output High Bits (Default: 06) 
	{{0x35,0x0d}, {0xd0}},//AEC PK VTS, Output Low Bits (Default: 18) 
	{{0x35,0x00}, {0x00}},//AEC PK Long Expo High Bits (Default: 00)
	{{0x35,0x01}, {0x00}},//AEC PK Long Expo middle Bits (Default: 00) 
	{{0x35,0x02}, {0x00}},//AEC PK Long Expo low Bits (Default: 00)
	{{0x35,0x0a}, {0x00}},//AEC PK AGC ADJ, Gain Output to sensor high bits (Default: 00)
	{{0x35,0x0b}, {0x00}},//AEC PK AGC ADJ, Gain Output to sensor low bits (Default: 00) 
	{{0x35,0x03}, {0x00}},//AEC PK Manual, Auto Enable (Default: 00)
	
	
	//de-noise registers
	{{0x52,0x8a}, {0x02}}, // Noise Y List 0, (Default: 02)
	{{0x52,0x8b}, {0x04}}, // Noise Y List 1, (Default: 04)
	{{0x52,0x8c}, {0x08}}, // Noise Y List 2, (Default: 08)
	{{0x52,0x8d}, {0x08}}, // Noise Y List 3, (Default: 14)
	{{0x52,0x8e}, {0x08}}, // Noise Y List 4, (Default: 1E)
	{{0x52,0x8f}, {0x10}}, // Noise Y List 5, (Default: 28)
	{{0x52,0x90}, {0x10}}, // Noise Y List 6, (Default: 32)
	{{0x52,0x92}, {0x00}}, // Noise UV List 0, (Default: 00)
	{{0x52,0x93}, {0x02}}, // Noise UV List 0, (Default: 02)
	{{0x52,0x94}, {0x00}}, // Noise UV List 1, (Default: 00)
	{{0x52,0x95}, {0x02}}, // Noise UV List 1, (Default: 04)
	{{0x52,0x96}, {0x00}}, // Noise UV List 2, (Default: 00)
	{{0x52,0x97}, {0x02}}, // Noise UV List 2, (Default: 0C)
	{{0x52,0x98}, {0x00}}, // Noise UV List 3, (Default: 00)
	{{0x52,0x99}, {0x02}}, // Noise UV List 3, (Default: 28)
	{{0x52,0x9a}, {0x00}}, // Noise UV List 4, (Default: 00)
	{{0x52,0x9b}, {0x02}}, // Noise UV List 4, (Default: 32)
	{{0x52,0x9c}, {0x00}}, // Noise UV List 5, (Default: 00)
	{{0x52,0x9d}, {0x02}}, // Noise UV List 5, (Default: 3C)
	{{0x52,0x9e}, {0x00}}, // Noise UV List 6, (Default: 00)
	{{0x52,0x9f}, {0x02}}, // Noise UV List 6, (Default: 4C)
	
	
	//Power Down Domain AEC/AGC Registers
	{{0x3a,0x0f}, {0x3c}}, 
	{{0x3a,0x10}, {0x30}}, 
	{{0x3a,0x1b}, {0x3c}}, 
	{{0x3a,0x1e}, {0x30}}, 
	{{0x3a,0x11}, {0x70}}, 
	{{0x3a,0x1f}, {0x10}}, 
	{{0x30,0x30}, {0x0b}}, 
	{{0x3a,0x02}, {0x00}}, 
	{{0x3a,0x03}, {0x7d}}, 
	{{0x3a,0x04}, {0x00}}, 
	{{0x3a,0x14}, {0x00}}, 
	{{0x3a,0x15}, {0x7d}}, 
	{{0x3a,0x16}, {0x00}}, 
	{{0x3a,0x00}, {0x7c}}, 
	{{0x3a,0x08}, {0x09}}, 
	{{0x3a,0x09}, {0x60}}, 
	{{0x3a,0x0a}, {0x07}}, 
	{{0x3a,0x0b}, {0xd0}}, 
	{{0x3a,0x0d}, {0x08}}, 
	{{0x3a,0x0e}, {0x06}}, 
	
	//AWB Regisiter
	{{0x51,0x93}, {0x70}}, //Red limit (Default:F0)
	
	//Sensor Control Registers
	{{0x36,0x20}, {0x57}}, //Analog Control Registers
	{{0x37,0x03}, {0x98}}, //Analog Control Registers
	{{0x37,0x04}, {0x1c}}, //Analog Control Registers
	
	{{0x58,0x9b}, {0x04}}, //??????????
	{{0x58,0x9a}, {0xc5}}, //??????????
	
	//de-noise registers
	{{0x52,0x8a}, {0x00}}, 
	{{0x52,0x8b}, {0x02}}, 
	{{0x52,0x8c}, {0x08}}, 
	{{0x52,0x8d}, {0x10}}, 
	{{0x52,0x8e}, {0x20}}, 
	{{0x52,0x8f}, {0x28}}, 
	{{0x52,0x90}, {0x30}}, 
	{{0x52,0x92}, {0x00}}, 
	{{0x52,0x93}, {0x00}}, 
	{{0x52,0x94}, {0x00}}, 
	{{0x52,0x95}, {0x02}},
	{{0x52,0x96}, {0x00}}, 
	{{0x52,0x97}, {0x08}}, 
	{{0x52,0x98}, {0x00}}, 
	{{0x52,0x99}, {0x10}}, 
	{{0x52,0x9a}, {0x00}}, 
	{{0x52,0x9b}, {0x20}}, 
	{{0x52,0x9c}, {0x00}}, 
	{{0x52,0x9d}, {0x28}}, 
	{{0x52,0x9e}, {0x00}}, 
	{{0x52,0x9f}, {0x30}}, 
	
	{{0x52,0x82}, {0x00}}, //DNS Control 12, de-noise manual disabled
	
	//Color interpolation Control Registers
	{{0x53,0x00}, {0x00}}, 
	{{0x53,0x01}, {0x20}}, 
	{{0x53,0x02}, {0x00}}, 
	{{0x53,0x03}, {0x7c}}, 
	{{0x53,0x0c}, {0x00}}, 
	{{0x53,0x0d}, {0x0c}},
	{{0x53,0x0e}, {0x20}},
	{{0x53,0x0f}, {0x80}},
	{{0x53,0x10}, {0x20}}, 
	{{0x53,0x11}, {0x80}}, 
	{{0x53,0x08}, {0x20}}, 
	{{0x53,0x09}, {0x40}}, 
	{{0x53,0x04}, {0x00}}, 
	{{0x53,0x05}, {0x30}}, 
	{{0x53,0x06}, {0x00}}, 
	{{0x53,0x07}, {0x80}}, 
	{{0x53,0x14}, {0x08}}, 
	{{0x53,0x15}, {0x20}},
	{{0x53,0x19}, {0x30}},
	{{0x53,0x16}, {0x10}}, 
	{{0x53,0x17}, {0x08}}, 
	{{0x53,0x18}, {0x02}}, 
	
	//Colour Matrix Control Registers
	{{0x53,0x80}, {0x01}}, 
	{{0x53,0x81}, {0x00}}, 
	{{0x53,0x82}, {0x00}},
	{{0x53,0x83}, {0x4e}}, 
	{{0x53,0x84}, {0x00}}, 
	{{0x53,0x85}, {0x0f}}, 
	{{0x53,0x86}, {0x00}}, 
	{{0x53,0x87}, {0x00}}, 
	{{0x53,0x88}, {0x01}}, 
	{{0x53,0x89}, {0x15}}, 
	{{0x53,0x8a}, {0x00}}, 
	{{0x53,0x8b}, {0x31}},
	{{0x53,0x8c}, {0x00}}, 
	{{0x53,0x8d}, {0x00}}, 
	{{0x53,0x8e}, {0x00}}, 
	{{0x53,0x8f}, {0x0f}}, 
	{{0x53,0x90}, {0x00}}, 
	{{0x53,0x91}, {0xab}}, 
	{{0x53,0x92}, {0x00}}, 
	{{0x53,0x93}, {0xa2}}, 
	{{0x53,0x94}, {0x08}}, 
	
	// YUV Gamma
	{{0x54,0x80}, {0x14}}, 
	{{0x54,0x81}, {0x21}}, 
	{{0x54,0x82}, {0x36}}, 
	{{0x54,0x83}, {0x57}}, 
	{{0x54,0x84}, {0x65}},
	{{0x54,0x85}, {0x71}}, 
	{{0x54,0x86}, {0x7d}}, 
	{{0x54,0x87}, {0x87}}, 
	{{0x54,0x88}, {0x91}}, 
	{{0x54,0x89}, {0x9a}}, 
	{{0x54,0x8a}, {0xaa}}, 
	{{0x54,0x8b}, {0xb8}}, 
	{{0x54,0x8c}, {0xcd}}, 
	{{0x54,0x8d}, {0xdd}}, 
	{{0x54,0x8e}, {0xea}}, 
	{{0x54,0x8f}, {0x10}}, 
	{{0x54,0x90}, {0x05}}, 
	{{0x54,0x91}, {0x00}}, 
	{{0x54,0x92}, {0x04}}, 
	{{0x54,0x93}, {0x20}}, 
	{{0x54,0x94}, {0x03}}, 
	{{0x54,0x95}, {0x60}}, 
	{{0x54,0x96}, {0x02}}, 
	{{0x54,0x97}, {0xb8}}, 
	{{0x54,0x98}, {0x02}}, 
	{{0x54,0x99}, {0x86}},
	{{0x54,0x9a}, {0x02}}, 
	{{0x54,0x9b}, {0x5b}}, 
	{{0x54,0x9c}, {0x02}}, 
	{{0x54,0x9d}, {0x3b}},
	{{0x54,0x9e}, {0x02}}, 
	{{0x54,0x9f}, {0x1c}},
	{{0x54,0xa0}, {0x02}},
	{{0x54,0xa1}, {0x04}},
	{{0x54,0xa2}, {0x01}},
	{{0x54,0xa3}, {0xed}},
	{{0x54,0xa4}, {0x01}},
	{{0x54,0xa5}, {0xc5}},
	{{0x54,0xa6}, {0x01}},
	{{0x54,0xa7}, {0xa5}},
	{{0x54,0xa8}, {0x01}}, 
	{{0x54,0xa9}, {0x6c}}, 
	{{0x54,0xaa}, {0x01}}, 
	{{0x54,0xab}, {0x41}}, 
	{{0x54,0xac}, {0x01}}, 
	{{0x54,0xad}, {0x20}}, 
	{{0x54,0xae}, {0x00}}, 
	{{0x54,0xaf}, {0x16}}, 
	
	
	//AWB Register
	{{0x34,0x06}, {0x00}},//AWB Manual (Default: 00) 
	
	//AWB Register
	{{0x51,0x92}, {0x04}}, 
	{{0x51,0x91}, {0xf8}}, 
	{{0x51,0x93}, {0x70}}, 
	{{0x51,0x94}, {0xf0}}, 
	{{0x51,0x95}, {0xf0}}, 
	{{0x51,0x8d}, {0x3d}}, 
	{{0x51,0x8f}, {0x54}}, 
	{{0x51,0x8e}, {0x3d}},
	{{0x51,0x90}, {0x54}}, 
	{{0x51,0x8b}, {0xc0}}, 
	{{0x51,0x8c}, {0xbd}}, 
	{{0x51,0x87}, {0x18}}, 
	{{0x51,0x88}, {0x18}}, 
	{{0x51,0x89}, {0x6e}}, 
	{{0x51,0x8a}, {0x68}}, 
	{{0x51,0x86}, {0x1c}}, 
	{{0x51,0x81}, {0x50}}, 
	{{0x51,0x84}, {0x25}},
	{{0x51,0x82}, {0x11}}, 
	{{0x51,0x83}, {0x14}}, 
	{{0x51,0x84}, {0x25}},
	{{0x51,0x85}, {0x24}}, 
	
	//ISP Control Register
	/*{{0x50,0x25}, {0x82}},//ISP Control 37, Average Statistic Selection enable YUVGMA (Default: 80)
	
	//Aec/Agc Control Functions
	{{0x3a,0x0f}, {0x7e}}, //AEC Ctrl 0F, Stable Range High Limit Enter (Default: 78)
	{{0x3a,0x10}, {0x72}}, //AEC Ctrl 10, Stable Range Low Limit Enter(Default: 68)
	{{0x3a,0x1b}, {0x80}}, //AEC Ctrl 1B, Stable Range High Limit Go Out (Default: 78)
	{{0x3a,0x1e}, {0x70}}, //AEC Ctrl 1E, Stable Range Low Limit Go Out (Default: 68)
	{{0x3a,0x11}, {0xd0}}, //AEC Ctrl 11, Step Manual Mode, Fast Zone High Limit (Default: D0)
	{{0x3a,0x1f}, {0x40}}, //AEC Ctrl 1F, Step Manual Mode, Fast Zone Low Limit (Default: 40)
	
	//Special Digital Effects Control Register (Hue/saturation, brightness, contrast...)
	{{0x55,0x83}, {0x40}}, //SDE Control 3, Saturation U (Default: 40)
	{{0x55,0x84}, {0x40}}, //SDE Control 4, Saturation V (Default: 40)
	{{0x55,0x80}, {0x02}}, //SDE Control 0, Saturation Enabled (Default: 00)
	*/
	/*
	//Sensor Control Registers
	{{0x36,0x33}, {0x07}}, //Analog Control Registers
	{{0x37,0x02}, {0x10}}, //Analog Control Registers
	{{0x37,0x03}, {0xb2}}, //Analog Control Registers
	{{0x37,0x04}, {0x18}}, //Analog Control Registers
	{{0x37,0x0b}, {0x40}}, //Analog Control Registers
	{{0x37,0x0d}, {0x02}}, //Analog Control Registers
	{{0x36,0x20}, {0x52}}, //Analog Control Registers

	// 320x240 
	{{0x38,0x08}, {0x01}},
	{{0x38,0x09}, {0x40}},
	{{0x38,0x0a}, {0x00}}, 
	{{0x38,0x0b}, {0xf0}}, 

	// Power Down Domain Aec/Agc Control Functions
	{{0x3a,0x00}, {0x78}},// AEC Ctrl 00, Night Mode Disabled (Default: 7C)

//Timing Control Register
	{{0x38,0x18}, {0x81}},// Timing Control 18, Vertical subsample 1/2 (Default: 80) 
	
	//Binning Related Register
	{{0x36,0x21}, {0xe7}}, // Array control 01, (Default: 10)See reg 0x3818 ??????????

//Special Digital Effects Control Register (Hue/saturation, brightness, contrast...)
//Contrast +1 
	{{0x50,0x01}, {0xff}},  
	{{0x55,0x80}, {0x04}},  
	{{0x55,0x87}, {0x24}}, //y offset  
	{{0x55,0x88}, {0x24}}, //y gain
	{{0x55,0x8a}, {0x00}},  
///Saturation 0 
	{{0x50,0x01}, {0xff}},  
	{{0x55,0x83}, {0x40}}, 
	{{0x55,0x84}, {0x40}}, 
	{{0x55,0x80}, {0x02}},  
//Brightness +1 
	{{0x50,0x01}, {0xff}},  
	{{0x55,0x89}, {0x10}},
	{{0x55,0x80}, {0x04}},  
	{{0x55,0x8a}, {0x00}}, 

//Advanced AWB  
	{{0x34,0x06}, {0x0 }},
	{{0x51,0x92}, {0x04}}, 
	{{0x51,0x91}, {0xf8}}, 
	{{0x51,0x8d}, {0x26}},
	{{0x51,0x8f}, {0x42}},
	{{0x51,0x8e}, {0x2b}}, 
	{{0x51,0x90}, {0x42}}, 
	{{0x51,0x8b}, {0xd0}},
	{{0x51,0x8c}, {0xbd}},                            
	{{0x51,0x87}, {0x18}},
	{{0x51,0x88}, {0x18}},
	{{0x51,0x89}, {0x56}}, 
	{{0x51,0x8a}, {0x5c}}, 
	{{0x51,0x86}, {0x1c}}, 
     	{{0x51,0x81}, {0x50}}, 
     	{{0x51,0x84}, {0x20}},
     	{{0x51,0x82}, {0x11}},
     	{{0x51,0x83}, {0x0 }},
//ev1.3
     	{{0x3a,0x0f}, {0x58}}, 
     	{{0x3a,0x10}, {0x50}}, 
     	{{0x3a,0x11}, {0x91}}, 
     	{{0x3a,0x1b}, {0x58}},
     	{{0x3a,0x1e}, {0x50}}, 
     	{{0x3a,0x1f}, {0x20}}, 

     //hue +30 degree      
     	{{0x55,0x80}, {0x01}}, 
     	{{0x55,0x81}, {0x6f}}, 
     	{{0x55,0x82}, {0x20}}, 
     	{{0x55,0x8a}, {0x01}},
      ////RGB565
     	//{{0x50,0x1e}, {0x2a}}, // ISP Dither Control
     	//{{0x50,0x02}, {0xf8}}, // ISP Control
     	//{{0x50,0x1f}, {0x01}}, // ISP MUX Control
     	//{{0x43,0x00}, {0x61}}, // Pixel Format 
   //qvga out
	// 320x240      	
	//{{0x38,0x08}, {0x01}}, 
     	//{{0x38,0x09}, {0x40}},
     	//{{0x38,0x0a}, {0x00}},
     	//{{0x38,0x0b}, {0xf0}},
	{{0x38,0x08}, {0x02}}, //DVP output horizontal width high byte[11:8]
	{{0x38,0x09}, {0x80}}, //DVP output horizontal width low byte[7:0]
	{{0x38,0x0a}, {0x01}}, //DVP output vertical height high byte[11:8]
	{{0x38,0x0b}, {0xe0}}, //DVP output vertical height low byte[7:0] 
*/
};

static struct regval_list sensor_oe_disable_regs[] = {
	{{0x30,0x17},{0x00}},
	{{0x30,0x18},{0x00}},
};

/*
 * The white balance settings
 * Here only tune the R G B channel gain.
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {
	{{0x34,0x06},{0x0}},
	//{{0x51,0x83},{0x94}},
	//{{0x51,0x91},{0xff}},
	//{{0x51,0x92},{0x00}},
};

static struct regval_list sensor_wb_cloud_regs[] = {
	{{0x34,0x06},{0x1 }},
	{{0x34,0x00},{0x6 }},
	{{0x34,0x01},{0x48}},
	{{0x34,0x02},{0x4 }},
	{{0x34,0x03},{0x0 }},
	{{0x34,0x04},{0x4 }},
	{{0x34,0x05},{0xd3}},
};

static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
	{{0x34,0x06},{0x1 }},
	{{0x34,0x00},{0x6 }},
	{{0x34,0x01},{0x1c}},
	{{0x34,0x02},{0x4 }},
	{{0x34,0x03},{0x0 }},
	{{0x34,0x04},{0x4 }},
	{{0x34,0x05},{0xf3}},
};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang
	{{0x34,0x06},{0x1 }},
	{{0x34,0x00},{0x4 }},
	{{0x34,0x01},{0x10}},
	{{0x34,0x02},{0x4 }},
	{{0x34,0x03},{0x0 }},
	{{0x34,0x04},{0x8 }},
	{{0x34,0x05},{0xb6}},
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
	{{0x34,0x06},{0x1 }},
	{{0x34,0x00},{0x5 }},
	{{0x34,0x01},{0x48}},
	{{0x34,0x02},{0x4 }},
	{{0x34,0x03},{0x0 }},
	{{0x34,0x04},{0x7 }},
	{{0x34,0x05},{0xcf}},
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng

};

/*
 * The color effect settings
 */
static struct regval_list sensor_colorfx_none_regs[] = {
	{{0x50,0x01},{0x7f}},
	{{0x55,0x80},{0x04}},
};

static struct regval_list sensor_colorfx_bw_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x18}},
	{{0x55,0x83},{0x80}},
	{{0x55,0x84},{0x80}},
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x18}},
	{{0x55,0x83},{0x40}},
	{{0x55,0x84},{0xa0}},
};

static struct regval_list sensor_colorfx_negative_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x40}},
};

static struct regval_list sensor_colorfx_emboss_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x18}},
	{{0x55,0x83},{0x80}},
	{{0x55,0x84},{0xc0}},
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x18}},
	{{0x55,0x83},{0x80}},
	{{0x55,0x84},{0xc0}},
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x18}},
	{{0x55,0x83},{0xa0}},
	{{0x55,0x84},{0x40}},
};

static struct regval_list sensor_colorfx_grass_green_regs[] = {
	{{0x50,0x01},{0xff}},
	{{0x55,0x80},{0x18}},
	{{0x55,0x83},{0x60}},
	{{0x55,0x84},{0x60}},
};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_vivid_regs[] = {
//NULL
};

/*
 * The brightness setttings
 */
static struct regval_list sensor_brightness_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_zero_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos4_regs[] = {
//NULL
};

/*
 * The contrast setttings
 */
static struct regval_list sensor_contrast_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_zero_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos4_regs[] = {
//NULL
};

/*
 * The saturation setttings
 */
static struct regval_list sensor_saturation_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_zero_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos4_regs[] = {
//NULL
};

/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_neg4_regs[] = {
	{{0x3a,0x0f},{0x10}},	//-1.7EV
	{{0x3a,0x10},{0x08}},
	{{0x3a,0x1b},{0x10}},
	{{0x3a,0x1e},{0x08}},
	{{0x3a,0x11},{0x20}},
	{{0x3a,0x1f},{0x10}},
};

static struct regval_list sensor_ev_neg3_regs[] = {
	{{0x3a,0x0f},{0x18}},	//-1.3EV
	{{0x3a,0x10},{0x10}},
	{{0x3a,0x1b},{0x18}},
	{{0x3a,0x1e},{0x10}},
	{{0x3a,0x11},{0x30}},
	{{0x3a,0x1f},{0x10}},
};

static struct regval_list sensor_ev_neg2_regs[] = {
	{{0x3a,0x0f},{0x20}},	//-1.0EV
	{{0x3a,0x10},{0x18}},
	{{0x3a,0x11},{0x41}},
	{{0x3a,0x1b},{0x20}},
	{{0x3a,0x1e},{0x18}},
	{{0x3a,0x1f},{0x10}},
};

static struct regval_list sensor_ev_neg1_regs[] = {
	{{0x3a,0x0f},{0x28}},	//-0.7EV
	{{0x3a,0x10},{0x20}},
	{{0x3a,0x11},{0x51}},
	{{0x3a,0x1b},{0x28}},
	{{0x3a,0x1e},{0x20}},
	{{0x3a,0x1f},{0x10}},
};

static struct regval_list sensor_ev_zero_regs[] = {
	{{0x3a,0x0f},{0x38}},		//default
	{{0x3a,0x10},{0x30}},
	{{0x3a,0x11},{0x61}},
	{{0x3a,0x1b},{0x38}},
	{{0x3a,0x1e},{0x30}},
	{{0x3a,0x1f},{0x10}},
};

static struct regval_list sensor_ev_pos1_regs[] = {
	{{0x3a,0x0f},{0x48}},	//0.7EV
	{{0x3a,0x10},{0x40}},
	{{0x3a,0x11},{0x80}},
	{{0x3a,0x1b},{0x48}},
	{{0x3a,0x1e},{0x40}},
	{{0x3a,0x1f},{0x20}},
};

static struct regval_list sensor_ev_pos2_regs[] = {
	{{0x3a,0x0f},{0x50}},	//1.0EV
	{{0x3a,0x10},{0x48}},
	{{0x3a,0x11},{0x90}},
	{{0x3a,0x1b},{0x50}},
	{{0x3a,0x1e},{0x48}},
	{{0x3a,0x1f},{0x20}},
};

static struct regval_list sensor_ev_pos3_regs[] = {
	{{0x3a,0x0f},{0x58}},	//1.3EV
	{{0x3a,0x10},{0x50}},
	{{0x3a,0x11},{0x91}},
	{{0x3a,0x1b},{0x58}},
	{{0x3a,0x1e},{0x50}},
	{{0x3a,0x1f},{0x20}},
};

static struct regval_list sensor_ev_pos4_regs[] = {
	{{0x3a,0x0f},{0x60}},	//1.7EV
	{{0x3a,0x10},{0x58}},
	{{0x3a,0x11},{0xa0}},
	{{0x3a,0x1b},{0x60}},
	{{0x3a,0x1e},{0x58}},
	{{0x3a,0x1f},{0x20}},
};

/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret,i;

	for(i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];

	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
		data[i] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */

	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for(i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i+REG_ADDR_STEP];
		ret = 0;
	}
	else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret,i;

	for(i = 0; i < REG_ADDR_STEP; i++)
			data[i] = reg[i];
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
			data[i] = value[i-REG_ADDR_STEP];

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;


	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}

/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
	struct sensor_info *info = to_state(sd);
	//struct sensor_format_struct *sensor_fmt=info->fmt;
	int i,ret;
//	unsigned char rd;
	if (size == 0)
		return -EINVAL;

	for(i = 0; i < size ; i++)
	{
		if(vals->reg_num[0] == 0xff && vals->reg_num[1] == 0xff) {
			//msleep(*(vals->value));
		}
		else {
			unsigned char *val=vals->value;
			unsigned int reg=(*(vals->reg_num)<<8)+(*(vals->reg_num+1));
			if(info->width<320 || info->height<240) {info->width=640; info->height=480; info->clkrc=2;}
			switch(reg){
				case 0x3011: //fps
					if(info->clkrc<=2)
						*val=0x08;
					else
						*val=0x09;
					printk("%s: write reg=0x%x pclk val=0x%x\n", __func__, reg, *val);
					break;
				/*case REG_OUT_WIDTH_HIGH: //width high
					*val=((info->width>>8)&0xFF);
					csi_dev_dbg("%s: write reg=0x%x width(%d) high val=0x%x\n", __func__, reg, info->width, *val);
					break;
				case REG_OUT_WIDTH_LOW: // width low
					*val=(info->width&0xFF);
					csi_dev_dbg("%s: write reg=0x%x width(%d) low val=0x%x\n", __func__, reg, info->width, *val);
					break;
				case REG_OUT_HEIGHT_HIGH: // height high
					*val=((info->height>>8)&0xFF);
					csi_dev_dbg("%s: write reg=0x%x height(%d) high val=0x%x\n", __func__, reg, info->height, *val);
					break;
				case REG_OUT_HEIGHT_LOW: // height low
					*val=(info->height&0xFF);
					csi_dev_dbg("%s: write reg=0x%x height(%d) low val=0x%x\n", __func__, reg, info->height, *val);
					break;*/
				case REG_PIXFMT:
					*val=info->fmt->reg;
					csi_dev_dbg("%s: write reg=0x%x pix_fmt val=0x%x\n", __func__, reg, *val);					
					
			}

			//csi_dev_dbg("%s: write reg=0x%x val=0x%x\n", __func__, reg, *val);
			//ret = sensor_write(sd, vals->reg_num, val);
			//csi_dev_dbg("%s: write reg=0x%x val=0x%x\n", __func__, (*(vals->reg_num)<<8)+(*(vals->reg_num+1)), (*(vals->value)));
			ret = sensor_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
			{
				csi_dev_err("sensor_write_err!\n");
				return ret;
			}
		}
		vals++;
	}

	return 0;
}


/*
 * Stuff that knows about the sensor.
 */

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct sensor_info *info = to_state(sd);
	char csi_stby_str[32],csi_power_str[32],csi_reset_str[32];
	int ret;

	if(info->ccm_info->iocfg == 0) {
		strcpy(csi_stby_str,"csi_stby");
		strcpy(csi_power_str,"csi_power_en");
		strcpy(csi_reset_str,"csi_reset");
	} else if(info->ccm_info->iocfg == 1) {
	  strcpy(csi_stby_str,"csi_stby_b");
	  strcpy(csi_power_str,"csi_power_en_b");
	  strcpy(csi_reset_str,"csi_reset_b");
	}

  switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			//reset off io
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			//active mclk before stadby in
			clk_enable(dev->csi_module_clk);
			msleep(100);
			//disable io oe
			csi_dev_print("disalbe oe!\n");
			ret = sensor_write_array(sd, sensor_oe_disable_regs , ARRAY_SIZE(sensor_oe_disable_regs));
			if(ret < 0)
				csi_dev_err("disalbe oe falied!\n");
			//standby on io
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			msleep(100);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);

//			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
//			msleep(10);
			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			msleep(10);
			//reset off io
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			msleep(10);
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//inactive mclk before power on
			clk_disable(dev->csi_module_clk);
			//power on reset
			gpio_set_one_pin_io_status(dev->csi_pin_hd,1,csi_stby_str);//set the gpio to output
			gpio_set_one_pin_io_status(dev->csi_pin_hd,1,csi_reset_str);//set the gpio to output
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(1);
			//power supply
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_PWR_ON,csi_power_str);
			msleep(10);
			if(dev->dvdd) {
				regulator_enable(dev->dvdd);
				msleep(10);
			}
			if(dev->avdd) {
				regulator_enable(dev->avdd);
				msleep(10);
			}
			if(dev->iovdd) {
				regulator_enable(dev->iovdd);
				msleep(10);
			}
			//active mclk before power on
			clk_enable(dev->csi_module_clk);
			//reset after power on
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			msleep(10);
			break;

		case CSI_SUBDEV_PWR_OFF:
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			//power supply off
			if(dev->iovdd) {
				regulator_disable(dev->iovdd);
				msleep(10);
			}
			if(dev->avdd) {
				regulator_disable(dev->avdd);
				msleep(10);
			}
			if(dev->dvdd) {
				regulator_disable(dev->dvdd);
				msleep(10);
			}
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_PWR_OFF,csi_power_str);
			msleep(10);

			//inactive mclk after power off
			clk_disable(dev->csi_module_clk);

			//set the io to hi-z
			gpio_set_one_pin_io_status(dev->csi_pin_hd,0,csi_reset_str);//set the gpio to input
			gpio_set_one_pin_io_status(dev->csi_pin_hd,0,csi_stby_str);//set the gpio to input
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct sensor_info *info = to_state(sd);
	char csi_reset_str[32];

	if(info->ccm_info->iocfg == 0) {
		strcpy(csi_reset_str,"csi_reset");
	} else if(info->ccm_info->iocfg == 1) {
	  strcpy(csi_reset_str,"csi_reset_b");
	}

	switch(val)
	{
		case CSI_SUBDEV_RST_OFF:
			csi_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			break;
		case CSI_SUBDEV_RST_ON:
			csi_dev_dbg("CSI_SUBDEV_RST_ON\n");
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(10);
			break;
		case CSI_SUBDEV_RST_PUL:
			csi_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	struct regval_list regs;

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x0A;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}

	if(regs.value[0] != 0x56)
		return -ENODEV;

	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	csi_dev_dbg("sensor_init!!!\n");

	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		csi_dev_err("chip found is not an target chip.\n");
		return ret;
	}

	//return sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
	return sensor_write_array(sd, ov5642_default_regs_init , ARRAY_SIZE(ov5642_default_regs_init));
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;

	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO:
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;

			csi_dev_dbg("CSI_SUBDEV_CMD_GET_INFO\n");

			ccm_info->mclk 	=	info->ccm_info->mclk ;
			ccm_info->vref 	=	info->ccm_info->vref ;
			ccm_info->href 	=	info->ccm_info->href ;
			ccm_info->clock	=	info->ccm_info->clock;
			ccm_info->iocfg	=	info->ccm_info->iocfg;

			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			break;
		}
		case CSI_SUBDEV_CMD_SET_INFO:
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;

			csi_dev_dbg("CSI_SUBDEV_CMD_SET_INFO\n");

			info->ccm_info->mclk 	=	ccm_info->mclk 	;
			info->ccm_info->vref 	=	ccm_info->vref 	;
			info->ccm_info->href 	=	ccm_info->href 	;
			info->ccm_info->clock	=	ccm_info->clock	;
			info->ccm_info->iocfg	=	ccm_info->iocfg	;

			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);

			break;
		}
		default:
			return -EINVAL;
	}
		return ret;
}

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

static struct sensor_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size) (struct v4l2_subdev *sd);
/* h/vref stuff */
} sensor_win_sizes[] = {
	/* qsxga: 2590*1944 */
	/*{
		.width			= QSXGA_WIDTH,
		.height 		= QSXGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.regs_size		= 4,
		.set_size		= NULL,
	},*/
	/* qxga: 2048*1536 */
	/*{
		.width			= QXGA_WIDTH,
		.height 		= QXGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},*/
	/* 1080P */
	/*{
		.width			= P1080P_WIDTH,
		.height			= P1080P_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},*/
	/* UXGA */
	/*{
		.width			= UXGA_WIDTH,
		.height			= UXGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},*/
	/* SXGA */
	/*{
		.width			= SXGA_WIDTH,
		.height 		= SXGA_HEIGHT,
		.regs			= QVGA_rgb565_Preview,
		.regs_size		= ARRAY_SIZE(QVGA_rgb565_Preview),
		//.regs			= sensor_sxga_regs,
		//.regs_size	= ARRAY_SIZE(sensor_sxga_regs),
		.set_size		= NULL,
	},*/
	/* 720p */
	{
		.width			= P720_WIDTH,
		.height			= P720_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},
	/* XGA */
	{
		.width			= XGA_WIDTH,
		.height 		= XGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},
	/* SVGA */
	{
		.width			= SVGA_WIDTH,
		.height			= SVGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},
	/* VGA */
	{
		.width			= VGA_WIDTH,
		.height			= VGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.regs_size		= 4,
		.set_size		= NULL,
	},
	/* QVGA */
	{
		.width			= QVGA_WIDTH,
		.height			= QVGA_HEIGHT,
		.regs			= ov5642_default_regs_init,
		.regs_size		= ARRAY_SIZE(ov5642_default_regs_init),
		.set_size		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))


static int reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}
static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)
{
//	struct sensor_format_struct *ofmt;

	if (index >= N_FMTS)
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;
//	ofmt = sensor_formats + fmt->index;
//	fmt->flags = 0;
//	strcpy(fmt->description, ofmt->desc);
//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}
static int sensor_enum_intervals(struct v4l2_subdev *sd, struct v4l2_frmivalenum *ivalnum)
{

	//printk("\n\n\nenum_intervals! index=%d %dx%d code=%d\n\n\n\n",ivalnum->index, ivalnum->width,ivalnum->height,ivalnum->pixel_format);
	if(ivalnum->index>1 || (ivalnum->index>0 && (ivalnum->width>800 && ivalnum->height>600)))
		return -EINVAL;

	if(ivalnum->index==0 && (ivalnum->width<800 && ivalnum->height<600))
		ivalnum->discrete.denominator=30;
	else if(ivalnum->index==0 && (ivalnum->width>800 || ivalnum->height>600))
		ivalnum->discrete.denominator=10;
	else
		ivalnum->discrete.denominator=15;

	ivalnum->type=V4L2_FRMIVAL_TYPE_DISCRETE;
	ivalnum->discrete.numerator=1;

	/*if (ivalnum->index)
		return -EINVAL;
	ivalnum->type = V4L2_FRMIVAL_TYPE_STEPWISE; // V4L2_FRMIVAL_TYPE_CONTINUOUS; //

	ivalnum->stepwise.min.numerator = 1;
	ivalnum->stepwise.min.denominator = 10;

	ivalnum->stepwise.max.numerator = 1;
	ivalnum->stepwise.max.denominator = 30;
 
	ivalnum->stepwise.step.numerator = 1;
	ivalnum->stepwise.step.denominator = 15;*/
	//printk("enum_intervals\n%dx%d\n",ivalnum->width,ivalnum->height);
	return 0;
}
static int sensor_enum_size(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
  if(fsize->index > N_WIN_SIZES-1)
  	return -EINVAL;
  
  fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
  fsize->discrete.width = sensor_win_sizes[fsize->index].width;
  fsize->discrete.height = sensor_win_sizes[fsize->index].height;
  
  return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)
			break;

	if (index >= N_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = sensor_formats[0].mbus_code;
	}

	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;


	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;

	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;
	fmt->height = wsize->height;
	
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;
	//pix->sizeimage = pix->height*pix->bytesperline;

	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

static int sensor_set_exposure(struct v4l2_subdev *sd,
	unsigned char *gain,unsigned char *exposurelow,unsigned char *exposuremid,unsigned char *exposurehigh)
{
	#define CAPTURE_FRAMERATE 375
	#define PREVIEW_FRAMERATE 1500

	int ret;
	unsigned char ogain,oexposurelow,oexposuremid,oexposurehigh;
	int lines_10ms;
	int capture_maxLines;
	int preview_maxlines;
	long capture_Exposure;
	long capture_exposure_gain;
	long previewExposure;
	long capture_gain;

	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x03;
	regs.value[0] = 0x07;
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err !\n");
		return ret;
	}

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x0b;
	regs.value[0] = 0;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	ogain = regs.value[0];

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x02;
	regs.value[0] = 0;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	oexposurelow = regs.value[0];

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x01;
	regs.value[0] = 0;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	oexposuremid = regs.value[0];

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x00;
	regs.value[0] = 0;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	oexposurehigh = regs.value[0];

	preview_maxlines = 984;
	capture_maxLines = 1968;
	lines_10ms = CAPTURE_FRAMERATE * capture_maxLines/10000*13/12;

	previewExposure = ((unsigned int)(oexposurehigh))<<12 ;
	previewExposure += ((unsigned int)oexposuremid)<<4 ;
	previewExposure += (oexposurelow >>4);
	if(0 == preview_maxlines || 0== lines_10ms)
	{
		return 0;
	}

	capture_Exposure =
		((previewExposure*(CAPTURE_FRAMERATE)*(capture_maxLines))/
	(((preview_maxlines)*(PREVIEW_FRAMERATE))))*6/5;

	capture_gain = ogain ;
	//if(AWB_NIGHT == m_CurrentEnvi)
	//{
	//capture_exposure_gain = ulCapture_Exposure * capture_gain*2;
	//*2
	//}else
	//{
	capture_exposure_gain = capture_Exposure * capture_gain;
	//}
	if(capture_exposure_gain <
		((signed int)(capture_maxLines)*16))
	{
		capture_Exposure = capture_exposure_gain/16;
		if (capture_Exposure > lines_10ms)
		{
			capture_Exposure /= lines_10ms;
			capture_Exposure *= lines_10ms;
		}
	} else {
		capture_Exposure = capture_maxLines;
	}
	if(capture_Exposure == 0)
	{
		capture_Exposure = 1;
	}
	capture_gain =(capture_exposure_gain*2/capture_Exposure + 1)/2;
	*exposurelow = ((unsigned char)capture_Exposure)<<4;
	*exposuremid = (unsigned char)(capture_Exposure >> 4) & 0xff;
	*exposurehigh = (unsigned char)(capture_Exposure >> 12);
	*gain =(unsigned char) capture_gain;

	return 0;
}

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	unsigned char gain,exposurelow,exposuremid,exposurehigh;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct regval_list regs;
	struct sensor_info *info = to_state(sd);
	csi_dev_dbg("sensor_s_fmt\n");

	gain = 0;
	exposurelow = 0;
	exposuremid = 0;
	exposurehigh = 0;

	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;
	/*

	if(!((wsize->width == VGA_WIDTH)&&(wsize->height == VGA_HEIGHT)))
	{
		ret = sensor_set_exposure(sd,&gain,&exposurelow,&exposuremid,&exposurehigh);
		if (ret)
		{
			csi_dev_err("sensor_set_exposure err !\n");
			return ret;
		}
	}

	sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);

	ret = 0;
	if (wsize->regs)
	{
		ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
		if (ret < 0)
			return ret;
	}


	{
		printk("gain:0x%x,exposurelow:0x%x,exposuremid:0x%x,exposurehigh:0x%x\n",gain,exposurelow,exposuremid,exposurehigh);
		regs.reg_num[0] = 0x35;
		regs.reg_num[1] = 0x0b;
		regs.value[0] = gain;
		ret = sensor_write(sd, regs.reg_num, regs.value);
		if (ret < 0) {
			csi_dev_err("sensor_write gain err !\n");
			return ret;
		}

		regs.reg_num[0] = 0x35;
		regs.reg_num[1] = 0x02;
		regs.value[0] = exposurelow;
		ret = sensor_write(sd, regs.reg_num, regs.value);
		if (ret < 0) {
			csi_dev_err("sensor_write exposurelow err !\n");
			return ret;
		}

		regs.reg_num[0] = 0x35;
		regs.reg_num[1] = 0x01;
		regs.value[0] = exposuremid;
		ret = sensor_write(sd, regs.reg_num, regs.value);
		if (ret < 0) {
			csi_dev_err("sensor_write exposuremid err !\n");
			return ret;
		}

		regs.reg_num[0] = 0x35;
		regs.reg_num[1] = 0x00;
		regs.value[0] = exposurehigh;
		ret = sensor_write(sd, regs.reg_num, regs.value);
		if (ret < 0) {
			csi_dev_err("sensor_write exposurehigh err !\n");
			return ret;
		}

		if (wsize->set_size)
		{
			ret = wsize->set_size(sd);
			if (ret < 0)
				return ret;
		}
	}*/
	sensor_write_array(sd, ov5642_default_regs_init,ARRAY_SIZE(ov5642_default_regs_init));

	//sensor_write_array(sd, ov5642_default_regs_init,ARRAY_SIZE(ov5642_default_regs_init));
	//sensor_set_resolution(sd);
	//sensor_write_array(sd, ov5642_default_regs_finalise,ARRAY_SIZE(ov5642_default_regs_finalise));
	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
	csi_dev_dbg("%s: %dx%d sensor_fmt=%s\n", __func__, info->width,info->height,sensor_fmt->desc);
	
	msleep(600);

	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;

	if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT ) {
		if(info->clkrc==2)
			cp->timeperframe.denominator = 15; //(SENSOR_FRAME_RATE*1000)/2;
		else
			cp->timeperframe.denominator = 10; //(SENSOR_FRAME_RATE*1000)/3;
	}
	else {
		if(info->clkrc==2)
			cp->timeperframe.denominator = 15; //(SENSOR_FRAME_RATE*1000)/2;
		else
			cp->timeperframe.denominator = 30; //(SENSOR_FRAME_RATE*1000);
	}
	csi_dev_dbg("%s: clkrc=%d %dx%d denominator=%d\n", __func__, info->clkrc, info->width,info->height,cp->timeperframe.denominator);
	struct regval_list regs;

	regs.reg_num[0] = 0x30;
	regs.reg_num[1] = 0x11;
	int ret = sensor_read(sd, regs.reg_num, regs.value);
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct sensor_info *info = to_state(sd);
	int div;
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else {
		if (info->width >= SVGA_WIDTH && info->height >= SVGA_HEIGHT) {
			div = ((tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator); //(tpf->numerator*SENSOR_FRAME_RATE/2)/tpf->denominator;
			csi_dev_dbg("%s: div=%d %d/%d\n", __func__, div, (tpf->numerator*SENSOR_FRAME_RATE/2),tpf->denominator);
		}
		else {
			div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
		}
	}
	if (div == 0)
		div = 1;
	else if (div > 8)
		div = 8;

	//switch()

	info->clkrc = (info->clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = SENSOR_FRAME_RATE/div;
	csi_dev_dbg("%s: set framerate div=%d! %d/%d info->clkrc=0x%x\n", __func__, div, tpf->numerator,tpf->denominator,info->clkrc);
	return sensor_write_array(sd, ov5642_default_regs_init,ARRAY_SIZE(ov5642_default_regs_init)); //reg_write(sd, REG_CLKRC, info->clkrc+7);
}


/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */

	switch (qc->id) {
	/*case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);*/
//	case V4L2_CID_CONTRAST:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_SATURATION:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_HUE:
//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
	case V4L2_CID_AUTOGAIN:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_DO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 1, 4, 1, 0);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	/*case V4L2_CID_COLORFX:
		return v4l2_ctrl_query_fill(qc, 0, 9, 1, 0);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);*/
	}
	return -EINVAL;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x38;
	regs.reg_num[1] = 0x18;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	//regs.value[0] &= (1<<1);

	*value = (regs.value[0]>>6)&1;
	csi_dev_dbg("%s: value=0x%x\n", __func__,*value);
	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x38;
	regs.reg_num[1] = 0x18;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
		  regs.value[0] &= 0xBF;
			break;
		case 1:
			regs.value[0] |= 1<<6;
			break;
		default:
			return -EINVAL;
	}
	csi_dev_dbg("%s: value=0x%x\n", __func__,regs.value);
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	msleep(100);

	info->hflip = value;

	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x38;
	regs.reg_num[1] = 0x18;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_vflip!\n");
		return ret;
	}

	*value = (regs.value[0]>>5)&1;

	info->vflip = *value;

	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x38;
	regs.reg_num[1] = 0x18;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
		  regs.value[0] &= 0xDF;
			break;
		case 1:
			regs.value[0] |= 1<<5;
			break;
		default:
			return -EINVAL;
	}

	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}

	msleep(100);

	info->vflip = value;

	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x03;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at %s!\n",__func__);
		return ret;
	}

	regs.value[0] &= 0x02;
	if (regs.value[0] == 0x00) {
		*value = 1;
	}
	else
	{
		*value = 0;
	}

	info->autoexp = *value;
	
	return 0;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x03;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at %s!\n",__func__);
		return ret;
	}

	switch (value) {
		case 1:
		  	regs.value[0] &= 0xfd;
			break;
		case 0:
			regs.value[0] |= 0x02;
			break;
		default:
			return -EINVAL;
	}

	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at %s!\n",__func__);
		return ret;
	}
	msleep(100);
	info->autoexp = value;
	
	return 0;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x03;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at %s!\n",__func__);
		return ret;
	}

	regs.value[0] &= 0x01;
	if (regs.value[0] == 0x00) {
		*value = V4L2_EXPOSURE_AUTO;
	}
	else
	{
		*value = V4L2_EXPOSURE_MANUAL;
	}

	info->autoexp = *value;
	
	return 0;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x03;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at %s!\n",__func__);
		return ret;
	}

	switch (value) {
		case V4L2_EXPOSURE_AUTO:
		  	regs.value[0] &= 0xfe;
			break;
		case V4L2_EXPOSURE_MANUAL:
			regs.value[0] |= 0x01;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}

	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at %s!\n",__func__);
		return ret;
	}
	msleep(100);
	info->autoexp = value;
	
	return 0;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x34;
	regs.reg_num[1] = 0x06;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autowb!\n");
		return ret;
	}

	regs.value[0] &= 1;

	*value = regs.value[0];
	info->autowb = *value;

	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}

	regs.reg_num[0] = 0x34;
	regs.reg_num[1] = 0x06;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
	}

	switch(value) {
	case 0:
		regs.value[0] |= 0x01;
		break;
	case 1:
		regs.value[0] &= 0xfe;
		break;
	default:
		break;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}
	msleep(10);
	info->autowb = value;

	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x0B;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_g_gain err!\n");
		return ret;
	}
	*value=regs.value;
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;

	regs.reg_num[0] = 0x35;
	regs.reg_num[1] = 0x0B;
	regs.value[0]   = 0xFF&value;
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}

	return 0;
}
/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->brightness;
	return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
		case -4:
		  	ret = sensor_write_array(sd, sensor_brightness_neg4_regs, ARRAY_SIZE(sensor_brightness_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_brightness_neg3_regs, ARRAY_SIZE(sensor_brightness_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_brightness_neg2_regs, ARRAY_SIZE(sensor_brightness_neg2_regs));
			break;
		case -1:
			ret = sensor_write_array(sd, sensor_brightness_neg1_regs, ARRAY_SIZE(sensor_brightness_neg1_regs));
			break;
		case 0:
			ret = sensor_write_array(sd, sensor_brightness_zero_regs, ARRAY_SIZE(sensor_brightness_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_brightness_pos1_regs, ARRAY_SIZE(sensor_brightness_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_brightness_pos2_regs, ARRAY_SIZE(sensor_brightness_pos2_regs));
			break;
		case 3:
			ret = sensor_write_array(sd, sensor_brightness_pos3_regs, ARRAY_SIZE(sensor_brightness_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_brightness_pos4_regs, ARRAY_SIZE(sensor_brightness_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_brightness!\n");
		return ret;
	}
	msleep(10);
	info->brightness = value;
	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->contrast;
	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_contrast_neg4_regs, ARRAY_SIZE(sensor_contrast_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_contrast_neg3_regs, ARRAY_SIZE(sensor_contrast_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_contrast_neg2_regs, ARRAY_SIZE(sensor_contrast_neg2_regs));
			break;
		case -1:
			ret = sensor_write_array(sd, sensor_contrast_neg1_regs, ARRAY_SIZE(sensor_contrast_neg1_regs));
			break;
		case 0:
			ret = sensor_write_array(sd, sensor_contrast_zero_regs, ARRAY_SIZE(sensor_contrast_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_contrast_pos1_regs, ARRAY_SIZE(sensor_contrast_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_contrast_pos2_regs, ARRAY_SIZE(sensor_contrast_pos2_regs));
			break;
		case 3:
			ret = sensor_write_array(sd, sensor_contrast_pos3_regs, ARRAY_SIZE(sensor_contrast_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_contrast_pos4_regs, ARRAY_SIZE(sensor_contrast_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_contrast!\n");
		return ret;
	}
	msleep(10);
	info->contrast = value;
	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->saturation;
	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	static struct regval_list sat[] ={{{0x50,0x01}, {0xff}},  
					{{0x55,0x83}, {0x40}}, 
					{{0x55,0x84}, {0x40}}, 
					{{0x55,0x80}, {0x02}}};
	int ret=0;
	struct sensor_info *info = to_state(sd);

	if( value > 64 || value < -64 ){
		csi_dev_err("saturation !\n");
		return -EINVAL;
	}
	*(sat[1].value)=value+64;
	*(sat[2].value)=value+64;
	sensor_write_array(sd, sat, ARRAY_SIZE(sat));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_saturation!\n");
		return ret;
	}
	msleep(10);
	info->saturation = value;
	return ret;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_ev_neg4_regs, ARRAY_SIZE(sensor_ev_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_ev_neg3_regs, ARRAY_SIZE(sensor_ev_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_ev_neg2_regs, ARRAY_SIZE(sensor_ev_neg2_regs));
			break;
		case -1:
			ret = sensor_write_array(sd, sensor_ev_neg1_regs, ARRAY_SIZE(sensor_ev_neg1_regs));
			break;
		case 0:
			ret = sensor_write_array(sd, sensor_ev_zero_regs, ARRAY_SIZE(sensor_ev_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_ev_pos1_regs, ARRAY_SIZE(sensor_ev_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_ev_pos2_regs, ARRAY_SIZE(sensor_ev_pos2_regs));
			break;
		case 3:
			ret = sensor_write_array(sd, sensor_ev_pos3_regs, ARRAY_SIZE(sensor_ev_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_ev_pos4_regs, ARRAY_SIZE(sensor_ev_pos4_regs));
			break;
		default:
			return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_exp!\n");
		return ret;
	}
	msleep(10);
	info->exp = value;
	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_whiteblance *wb_type = (enum v4l2_whiteblance*)value;

	*wb_type = info->wb;

	return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
		enum v4l2_whiteblance value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	if (value == V4L2_WB_AUTO) {
		ret = sensor_s_autowb(sd, 1);
		return ret;
	}
	else {
		ret = sensor_s_autowb(sd, 0);
		if(ret < 0) {
			csi_dev_err("sensor_s_autowb error, return %x!\n",ret);
			return ret;
		}

		switch (value) {
			case V4L2_WB_CLOUD:
			  ret = sensor_write_array(sd, sensor_wb_cloud_regs, ARRAY_SIZE(sensor_wb_cloud_regs));
				break;
			case V4L2_WB_DAYLIGHT:
				ret = sensor_write_array(sd, sensor_wb_daylight_regs, ARRAY_SIZE(sensor_wb_daylight_regs));
				break;
			case V4L2_WB_INCANDESCENCE:
				ret = sensor_write_array(sd, sensor_wb_incandescence_regs, ARRAY_SIZE(sensor_wb_incandescence_regs));
				break;
			case V4L2_WB_FLUORESCENT:
				ret = sensor_write_array(sd, sensor_wb_fluorescent_regs, ARRAY_SIZE(sensor_wb_fluorescent_regs));
				break;
			case V4L2_WB_TUNGSTEN:
				ret = sensor_write_array(sd, sensor_wb_tungsten_regs, ARRAY_SIZE(sensor_wb_tungsten_regs));
				break;
			default:
				return -EINVAL;
		}
	}

	if (ret < 0) {
		csi_dev_err("sensor_s_wb error, return %x!\n",ret);
		return ret;
	}

	msleep(10);
	info->wb = value;
	return 0;
}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx*)value;

	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
		enum v4l2_colorfx value)
{
	int ret;
	struct sensor_info *info = to_state(sd);

	switch (value) {
	case V4L2_COLORFX_NONE:
	  ret = sensor_write_array(sd, sensor_colorfx_none_regs, ARRAY_SIZE(sensor_colorfx_none_regs));
		break;
	case V4L2_COLORFX_BW:
		ret = sensor_write_array(sd, sensor_colorfx_bw_regs, ARRAY_SIZE(sensor_colorfx_bw_regs));
		break;
	case V4L2_COLORFX_SEPIA:
		ret = sensor_write_array(sd, sensor_colorfx_sepia_regs, ARRAY_SIZE(sensor_colorfx_sepia_regs));
		break;
	case V4L2_COLORFX_NEGATIVE:
		ret = sensor_write_array(sd, sensor_colorfx_negative_regs, ARRAY_SIZE(sensor_colorfx_negative_regs));
		break;
	case V4L2_COLORFX_EMBOSS:
		ret = sensor_write_array(sd, sensor_colorfx_emboss_regs, ARRAY_SIZE(sensor_colorfx_emboss_regs));
		break;
	case V4L2_COLORFX_SKETCH:
		ret = sensor_write_array(sd, sensor_colorfx_sketch_regs, ARRAY_SIZE(sensor_colorfx_sketch_regs));
		break;
	case V4L2_COLORFX_SKY_BLUE:
		ret = sensor_write_array(sd, sensor_colorfx_sky_blue_regs, ARRAY_SIZE(sensor_colorfx_sky_blue_regs));
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		ret = sensor_write_array(sd, sensor_colorfx_grass_green_regs, ARRAY_SIZE(sensor_colorfx_grass_green_regs));
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		ret = sensor_write_array(sd, sensor_colorfx_skin_whiten_regs, ARRAY_SIZE(sensor_colorfx_skin_whiten_regs));
		break;
	case V4L2_COLORFX_VIVID:
		ret = sensor_write_array(sd, sensor_colorfx_vivid_regs, ARRAY_SIZE(sensor_colorfx_vivid_regs));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0) {
		csi_dev_err("sensor_s_colorfx error, return %x!\n",ret);
		return ret;
	}
	msleep(10);
	info->clrfx = value;

	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_flash_mode *flash_mode = (enum v4l2_flash_mode*)value;

	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_mode value)
{
	struct sensor_info *info = to_state(sd);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	char csi_flash_str[32];
	int flash_on,flash_off;

	if(info->ccm_info->iocfg == 0) {
		strcpy(csi_flash_str,"csi_flash");
	} else if(info->ccm_info->iocfg == 1) {
	  strcpy(csi_flash_str,"csi_flash_b");
	}

	flash_on = (dev->flash_pol!=0)?1:0;
	flash_off = (flash_on==1)?0:1;

	switch (value) {
	case V4L2_FLASH_MODE_OFF:
	  gpio_write_one_pin_value(dev->csi_pin_hd,flash_off,csi_flash_str);
		break;
	case V4L2_FLASH_MODE_AUTO:
		return -EINVAL;
		break;
	case V4L2_FLASH_MODE_ON:
		gpio_write_one_pin_value(dev->csi_pin_hd,flash_on,csi_flash_str);
		break;
	case V4L2_FLASH_MODE_TORCH:
		return -EINVAL;
		break;
	case V4L2_FLASH_MODE_RED_EYE:
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	info->flash_mode = value;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_g_wb(sd, &ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_g_autowb(sd, &ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_g_colorfx(sd,	&ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_s_autoexp(sd,
				(enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_s_wb(sd,
				(enum v4l2_whiteblance) ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_s_autowb(sd, ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_s_colorfx(sd,
				(enum v4l2_colorfx) ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return sensor_s_flash_mode(sd,
	      (enum v4l2_flash_mode) ctrl->value);
	}
	return -EINVAL;
}


static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident	= sensor_g_chip_ident,
	.g_ctrl 	= sensor_g_ctrl,
	.s_ctrl 	= sensor_s_ctrl,
	.queryctrl 	= sensor_queryctrl,
	.reset 		= sensor_reset,
	.init		= sensor_init,
	.s_power	= sensor_power,
	.ioctl		= sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt		= sensor_enum_fmt,
	.enum_framesizes	= sensor_enum_size,
	.enum_frameintervals	= sensor_enum_intervals,
	.try_mbus_fmt		= sensor_try_fmt,
	.s_mbus_fmt		= sensor_s_fmt,
	.s_parm 		= sensor_s_parm,
	.g_parm			= sensor_g_parm,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->ccm_info = &ccm_info_con;

	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = 0;
	info->clrfx = 0;
	info->clkrc = 1;	/* 30fps */

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "ov5642", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);


static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	.name = "ov5642",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);

