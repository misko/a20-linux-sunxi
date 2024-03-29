/*
 * A V4L2 driver for OmniVision OV7670 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
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
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>
#include <plat/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov7670 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		1
#if(DEV_DBG_EN == 1)
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][OV7670]"x,##arg)
#else
#define csi_dev_dbg(x,arg...)
#endif
#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][OV7670]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][OV7670]"x,##arg)
#define MCLK (24*1000*1000)
#define VREF_POL	CSI_LOW
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0 for csi0

//define the voltage level of control signal
#define CSI_STBY_ON		1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON		0
#define CSI_RST_OFF		1
#define CSI_PWR_ON		1
#define CSI_PWR_OFF		0

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240

/*
 * Our nominal (default) frame rate.
 */
#define OV7670_FRAME_RATE 30

/*
 * The 7670 sits on i2c with ID 0x42
 */
#define OV7670_I2C_ADDR 0x42

/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58


#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */

#define REG_GFIX	0x69	/* Fix gain control */

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */



/*
 * Information we maintain about a known sensor.
 */
struct ov7670_format_struct;  /* coming later */
__csi_subdev_info_t ccm_info_con =
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};
struct ov7670_info {
	struct v4l2_subdev sd;
	struct ov7670_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
	enum v4l2_flash_mode flash_mode;
	u8 clkrc;			/* Clock divider value */
};

static inline struct ov7670_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7670_info, sd);
}



/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

static struct regval_list ov7670_default_regs[] = {
	{ REG_COM7, COM7_RESET },
/*
 * Clock scale: 3 = 15fps
 *              2 = 20fps
 *              1 = 30fps
 */
	{ REG_CLKRC, 0x1 },	/* OV: clock scale (30 fps) */
	{ REG_TSLB,  0x04 },	/* OV */
	{ REG_COM7, 0 },	/* VGA */
	/*
	 * Set the hardware window.  These values from OV don't entirely
	 * make sense - hstop is less than hstart.  But they work...
	 */
	{ REG_HSTART, 0x13 },	{ REG_HSTOP, 0x01 },
	{ REG_HREF, 0xb6 },	{ REG_VSTART, 0x02 },
	{ REG_VSTOP, 0x7a },	{ REG_VREF, 0x0a },

	{ REG_COM3, 0 },	{ REG_COM14, 0 },
	/* Mystery scaling numbers */
	{ 0x70, 0x3a },		{ 0x71, 0x35 },
	{ 0x72, 0x11 },		{ 0x73, 0xf0 },
	{ 0xa2, 0x02 },		{ REG_COM10, 0x0 },

	/* Gamma curve values */
	{ 0x7a, 0x20 },		{ 0x7b, 0x10 },
	{ 0x7c, 0x1e },		{ 0x7d, 0x35 },
	{ 0x7e, 0x5a },		{ 0x7f, 0x69 },
	{ 0x80, 0x76 },		{ 0x81, 0x80 },
	{ 0x82, 0x88 },		{ 0x83, 0x8f },
	{ 0x84, 0x96 },		{ 0x85, 0xa3 },
	{ 0x86, 0xaf },		{ 0x87, 0xc4 },
	{ 0x88, 0xd7 },		{ 0x89, 0xe8 },

	/* AGC and AEC parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
	{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
	{ REG_GAIN, 0 },	{ REG_AECH, 0 },
	{ REG_COM4, 0x40 }, /* magic reserved bit */
	{ REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
	{ REG_BD50MAX, 0x05 },	{ REG_BD60MAX, 0x07 },
	{ REG_AEW, 0x95 },	{ REG_AEB, 0x33 },
	{ REG_VPT, 0xe3 },	{ REG_HAECC1, 0x78 },
	{ REG_HAECC2, 0x68 },	{ 0xa1, 0x03 }, /* magic */
	{ REG_HAECC3, 0xd8 },	{ REG_HAECC4, 0xd8 },
	{ REG_HAECC5, 0xf0 },	{ REG_HAECC6, 0x90 },
	{ REG_HAECC7, 0x94 },
	{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC },

	/* Almost all of these are magic "reserved" values.  */
	{ REG_COM5, 0x61 },	{ REG_COM6, 0x4b },
	{ 0x16, 0x02 },		{ REG_MVFP, 0x07 },
	{ 0x21, 0x02 },		{ 0x22, 0x91 },
	{ 0x29, 0x07 },		{ 0x33, 0x0b },
	{ 0x35, 0x0b },		{ 0x37, 0x1d },
	{ 0x38, 0x71 },		{ 0x39, 0x2a },
	{ REG_COM12, 0x78 },	{ 0x4d, 0x40 },
	{ 0x4e, 0x20 },		{ REG_GFIX, 0 },
	{ 0x6b, 0x4a },		{ 0x74, 0x10 },
	{ 0x8d, 0x4f },		{ 0x8e, 0 },
	{ 0x8f, 0 },		{ 0x90, 0 },
	{ 0x91, 0 },		{ 0x96, 0 },
	{ 0x9a, 0 },		{ 0xb0, 0x84 },
	{ 0xb1, 0x0c },		{ 0xb2, 0x0e },
	{ 0xb3, 0x82 },		{ 0xb8, 0x0a },

	/* More reserved magic, some of which tweaks white balance */
	{ 0x43, 0x0a },		{ 0x44, 0xf0 },
	{ 0x45, 0x34 },		{ 0x46, 0x58 },
	{ 0x47, 0x28 },		{ 0x48, 0x3a },
	{ 0x59, 0x88 },		{ 0x5a, 0x88 },
	{ 0x5b, 0x44 },		{ 0x5c, 0x67 },
	{ 0x5d, 0x49 },		{ 0x5e, 0x0e },
	{ 0x6c, 0x0a },		{ 0x6d, 0x55 },
	{ 0x6e, 0x11 },		{ 0x6f, 0x9f }, /* "9e for advance AWB" */
	{ 0x6a, 0x40 },		{ REG_BLUE, 0x40 },
	{ REG_RED, 0x60 },
	{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB },

	/* Matrix coefficients */
	{ 0x4f, 0x80 },		{ 0x50, 0x80 },
	{ 0x51, 0 },		{ 0x52, 0x22 },
	{ 0x53, 0x5e },		{ 0x54, 0x80 },
	{ 0x58, 0x9e },

	{ REG_COM16, COM16_AWBGAIN },	{ REG_EDGE, 0 },
	{ 0x75, 0x05 },		{ 0x76, 0xe1 },
	{ 0x4c, 0 },		{ 0x77, 0x01 },
	{ REG_COM13, 0xc3 },	{ 0x4b, 0x09 },
	{ 0xc9, 0x60 },		{ REG_COM16, 0x38 },
	{ 0x56, 0x40 },

	{ 0x34, 0x11 },		{ REG_COM11, COM11_EXP|COM11_HZAUTO },
	{ 0xa4, 0x88 },		{ 0x96, 0 },
	{ 0x97, 0x30 },		{ 0x98, 0x20 },
	{ 0x99, 0x30 },		{ 0x9a, 0x84 },
	{ 0x9b, 0x29 },		{ 0x9c, 0x03 },
	{ 0x9d, 0x4c },		{ 0x9e, 0x3f },
	{ 0x78, 0x04 },

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{ 0x79, 0x01 },		{ 0xc8, 0xf0 },
	{ 0x79, 0x0f },		{ 0xc8, 0x00 },
	{ 0x79, 0x10 },		{ 0xc8, 0x7e },
	{ 0x79, 0x0a },		{ 0xc8, 0x80 },
	{ 0x79, 0x0b },		{ 0xc8, 0x01 },
	{ 0x79, 0x0c },		{ 0xc8, 0x0f },
	{ 0x79, 0x0d },		{ 0xc8, 0x20 },
	{ 0x79, 0x09 },		{ 0xc8, 0x80 },
	{ 0x79, 0x02 },		{ 0xc8, 0xc0 },
	{ 0x79, 0x03 },		{ 0xc8, 0x40 },
	{ 0x79, 0x05 },		{ 0xc8, 0x30 },
	{ 0x79, 0x26 },

	{ 0xff, 0xff },	/* END MARKER */
};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see ov7670_s_fmt for why.
 */


static struct regval_list ov7670_fmt_yuv422_yuyv[] = {
	{ REG_COM7, 0x0 },  /* Selects YUV mode */
	{ REG_TSLB,  0x00 },	/* bit[3:2] YUYV */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0 },	/* CCIR601 */
	{ REG_COM15, COM15_R00FF },
	{ REG_COM9, 0x18 }, /* 4x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_yuv422_yvyu[] = {
	{ REG_COM7, 0x0 },  /* Selects YUV mode */
	{ REG_TSLB,  0x04 },	/* bit[3:2] YVYU */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0 },	/* CCIR601 */
	{ REG_COM15, COM15_R00FF },
	{ REG_COM9, 0x18 }, /* 4x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT|COM13_UVSWAP },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_yuv422_vyuy[] = {
	{ REG_COM7, 0x0 },  /* Selects YUV mode */
	{ REG_TSLB,  0x08 },	/* bit[3:2] VYUY */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0 },	/* CCIR601 */
	{ REG_COM15, COM15_R00FF },
	{ REG_COM9, 0x18 }, /* 4x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT|TSLB_YLAST },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_yuv422_uyvy[] = {
	{ REG_COM7, 0x0 },  /* Selects YUV mode */
	{ REG_TSLB,  0x0c },	/* bit[3:2] UYVY */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0 },	/* CCIR601 */
	{ REG_COM15, COM15_R00FF },
	{ REG_COM9, 0x18 }, /* 4x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0x80 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0x80 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x22 }, 	/* "matrix coefficient 4" */
	{ 0x53, 0x5e }, 	/* "matrix coefficient 5" */
	{ 0x54, 0x80 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT|COM13_UVSWAP|TSLB_YLAST },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_rgb565[] = {
	{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0x0 },	/* CCIR601 */
	{ REG_COM15, COM15_RGB565 },
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_rgb444[] = {
	{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
	{ REG_RGB444, R444_ENABLE },	/* Enable xxxxrrrr ggggbbbb */
	{ REG_COM1, 0x0 },	/* CCIR601 */
	{ REG_COM15, COM15_R01FE|COM15_RGB565 }, /* Data range needed? */
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT|0x2 },  /* Magic rsvd bit */
	{ 0xff, 0xff },
};

static struct regval_list ov7670_fmt_raw[] = {
	{ REG_COM7, COM7_BAYER },
	{ REG_COM13, 0x08 }, /* No gamma, magic rsvd bit */
	{ REG_COM16, 0x3d }, /* Edge enhancement, denoise */
	{ REG_REG76, 0xe1 }, /* Pix correction, magic rsvd */
	{ 0xff, 0xff },
};



/*
 * Low-level register I/O.
 *
 * Note that there are two versions of these.  On the XO 1, the
 * i2c controller only does SMBUS, so that's what we use.  The
 * ov7670 is not really an SMBUS device, though, so the communication
 * is not always entirely reliable.
 */
#ifdef CONFIG_OLPC_XO_1
static int ov7670_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		*value = (unsigned char)ret;
		ret = 0;
	}
	return ret;
}


static int ov7670_write(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if (reg == REG_COM7 && (value & COM7_RESET))
		msleep(5);  /* Wait for reset to run */
	return ret;
}

#else /* ! CONFIG_OLPC_XO_1 */
/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int ov7670_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data = reg;
	struct i2c_msg msg;
	int ret;

	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		printk(KERN_ERR "Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	msg.flags = I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		*value = data;
		ret = 0;
	}
	return ret;
}


static int ov7670_write(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[2] = { reg, value };
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;
	if (reg == REG_COM7 && (value & COM7_RESET))
		msleep(5);  /* Wait for reset to run */
	return ret;
}
#endif /* CONFIG_OLPC_XO_1 */


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7670_write_array(struct v4l2_subdev *sd, struct regval_list *vals)
{
	csi_dev_dbg("%s: called!\n",__func__);
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = ov7670_write(sd, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */

static int ov7670_power(struct v4l2_subdev *sd, int on)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7670_info *info = to_state(sd);
	char csi_stby_str[32],csi_power_str[32],csi_reset_str[32];

	if(info->ccm_info->iocfg == 0) {
		strcpy(csi_stby_str,"csi_stby");
		strcpy(csi_power_str,"csi_power_en");
		strcpy(csi_reset_str,"csi_reset");
	} else if(info->ccm_info->iocfg == 1) {
	  strcpy(csi_stby_str,"csi_stby_b");
	  strcpy(csi_power_str,"csi_power_en_b");
	  strcpy(csi_reset_str,"csi_reset_b");
	}

  //make sure that no device can access i2c bus during sensor initial or power down
  //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
  i2c_lock_adapter(client->adapter);
  
  //insure that clk_disable() and clk_enable() are called in pair 
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
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
			//standby on io
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			msleep(100);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);

			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(10);
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
				csi_dev_dbg("regulator_enable dvdd\n");
				regulator_enable(dev->dvdd);
				msleep(10);
			}
			if(dev->avdd) {
				csi_dev_dbg("regulator_enable avdd\n");
				regulator_enable(dev->avdd);
				msleep(10);
			}
			if(dev->iovdd) {
				csi_dev_dbg("regulator_enable iovdd\n");
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
			i2c_unlock_adapter(client->adapter);
			return -EINVAL;
	}

	//remember to unlock i2c adapter, so the device can access the i2c bus again
	i2c_unlock_adapter(client->adapter);	
	return 0;
}

static int ov7670_reset(struct v4l2_subdev *sd, u32 val)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct ov7670_info *info = to_state(sd);
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

//	ov7670_write(sd, REG_COM7, COM7_RESET);
//	msleep(1);
	return 0;
}

static int ov7670_detect(struct v4l2_subdev *sd)
{
	csi_dev_dbg("%s: called!\n",__func__);
	unsigned char v;
	int ret;

//	ret = ov7670_init(sd, 0);
//	if (ret < 0)
//		return ret;
	ret = ov7670_read(sd, REG_MIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x7f) /* OV manuf. id. */
		return -ENODEV;
	ret = ov7670_read(sd, REG_MIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0xa2)
		return -ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7670_read(sd, REG_PID, &v);
	if (ret < 0)
		return ret;
	if (v != 0x76)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	ret = ov7670_read(sd, REG_VER, &v);
	if (ret < 0)
		return ret;
	if (v != 0x73)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	return 0;
}

static int ov7670_init(struct v4l2_subdev *sd, u32 val)
{
	csi_dev_dbg("%s: called!\n",__func__);
	int ret;
	csi_dev_dbg("ov7670_init\n");

	/* Make sure it's an ov7670 */
	ret = ov7670_detect(sd);
	if (ret) {
		csi_err("chip found is not an ov7670 chip.\n");
		return ret;
	}
	return ov7670_write_array(sd, ov7670_default_regs);
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	csi_dev_dbg("%s: called!\n",__func__);
	int ret=0;

	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO:
		{
			struct ov7670_info *info = to_state(sd);
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
			struct ov7670_info *info = to_state(sd);
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
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov7670_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* Bytes per pixel */
} ov7670_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= ov7670_fmt_yuv422_yuyv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= ov7670_fmt_yuv422_yvyu,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= ov7670_fmt_yuv422_uyvy,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= ov7670_fmt_yuv422_vyuy,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 2,
	},
	{
		.desc		= "RGB 444",
		.mbus_code	= V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE,//linux-3.0
		.regs		= ov7670_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "RGB 565",
		.mbus_code	= V4L2_MBUS_FMT_RGB565_2X8_LE,//linux-3.0
		.regs		= ov7670_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
		.regs 		= ov7670_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
		.bpp		= 1
	},
};
#define N_OV7670_FMTS ARRAY_SIZE(ov7670_formats)


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

/*
 * QCIF mode is done (by OV) in a very strange way - it actually looks like
 * VGA with weird scaling options - they do *not* use the canned QCIF mode
 * which is allegedly provided by the sensor.  So here's the weird register
 * settings.
 */
static struct regval_list ov7670_qcif_regs[] = {
	{ REG_COM3, COM3_SCALEEN|COM3_DCWEN },
	{ REG_COM3, COM3_DCWEN },
	{ REG_COM14, COM14_DCWEN | 0x01},
	{ 0x73, 0xf1 },
	{ 0xa2, 0x52 },
	{ 0x7b, 0x1c },
	{ 0x7c, 0x28 },
	{ 0x7d, 0x3c },
	{ 0x7f, 0x69 },
	{ REG_COM9, 0x38 },
	{ 0xa1, 0x0b },
	{ 0x74, 0x19 },
	{ 0x9a, 0x80 },
	{ 0x43, 0x14 },
	{ REG_COM13, 0xc0 },
	{ 0xff, 0xff },
};

static struct ov7670_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
/* h/vref stuff */
} ov7670_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,		/* These values from */
		.hstop		=  14,		/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.com7_bit	= COM7_FMT_QVGA,
		.hstart		= 164,		/* Empirically determined */
		.hstop		=  20,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov7670_win_sizes))


/*
 * Store a set of start/stop values into the camera.
 */
static int ov7670_set_hw(struct v4l2_subdev *sd, int hstart, int hstop,
		int vstart, int vstop)
{
	csi_dev_dbg("%s: called!\n",__func__);
	int ret;
	unsigned char v;
/*
 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
 * a mystery "edge offset" value in the top two bits of href.
 */
	ret =  ov7670_write(sd, REG_HSTART, (hstart >> 3) & 0xff);
	ret += ov7670_write(sd, REG_HSTOP, (hstop >> 3) & 0xff);
	ret += ov7670_read(sd, REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov7670_write(sd, REG_HREF, v);
/*
 * Vertical: similar arrangement, but only 10 bits.
 */
	ret += ov7670_write(sd, REG_VSTART, (vstart >> 2) & 0xff);
	ret += ov7670_write(sd, REG_VSTOP, (vstop >> 2) & 0xff);
	ret += ov7670_read(sd, REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov7670_write(sd, REG_VREF, v);
	return ret;
}


static int ov7670_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)//linux-3.0
{
	csi_dev_dbg("%s: called!\n",__func__);
//	struct ov7670_format_struct *ofmt;

	if (index >= N_OV7670_FMTS)//linux-3.0
		return -EINVAL;

	*code = ov7670_formats[index].mbus_code;//linux-3.0

//	ofmt = ov7670_formats + fmt->index;
//	fmt->flags = 0;
//	strcpy(fmt->description, ofmt->desc);
//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}
static int ov7670_frame_rates[] = { 30, 15, 10, 5, 1 };

static int sensor_enum_intervals(struct v4l2_subdev *sd,
		struct v4l2_frmivalenum *interval)
{
	csi_dev_dbg("%s: called!\n",__func__);
	if (interval->index >= ARRAY_SIZE(ov7670_frame_rates))
		return -EINVAL;
	interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	interval->discrete.numerator = 1;
	interval->discrete.denominator = ov7670_frame_rates[interval->index];
	return 0;
}
static int sensor_enum_size(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	csi_dev_dbg("%s: called!\n",__func__);
  if(fsize->index > N_WIN_SIZES-1)
  	return -EINVAL;
  
  fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
  fsize->discrete.width = ov7670_win_sizes[fsize->index].width;
  fsize->discrete.height = ov7670_win_sizes[fsize->index].height;
  
  return 0;
}

static int ov7670_try_fmt_internal(struct v4l2_subdev *sd,
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct ov7670_format_struct **ret_fmt,
		struct ov7670_win_size **ret_wsize)
{
	csi_dev_dbg("%s: called! %dx%d code=0x%2x\n",__func__,fmt->width,fmt->height,fmt->code);
	int index;
	struct ov7670_win_size *wsize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0
	csi_dev_dbg("ov7670_try_fmt_internal\n");
	for (index = 0; index < N_OV7670_FMTS; index++)
		if (ov7670_formats[index].mbus_code == fmt->code)//linux-3.0
			break;
	if (index >= N_OV7670_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = ov7670_formats[0].mbus_code;//linux-3.0
	}
	if (ret_fmt != NULL)
		*ret_fmt = ov7670_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */


	for (wsize = ov7670_win_sizes; wsize < ov7670_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;



	if (wsize >= ov7670_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
//pix->bytesperline = pix->width*ov7670_formats[index].bpp;//linux-3.0
//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0

		return 0;
}

static int ov7670_try_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	csi_dev_dbg("%s: called!\n",__func__);
	return ov7670_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int ov7670_s_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	csi_dev_dbg("%s: called!\n",__func__);
	int ret;
	struct ov7670_format_struct *ovfmt;
	struct ov7670_win_size *wsize;
	struct ov7670_info *info = to_state(sd);
	unsigned char com7;
	csi_dev_dbg("ov7670_s_fmt\n");
	ret = ov7670_try_fmt_internal(sd, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	/*
	 * COM7 is a pain in the ass, it doesn't like to be read then
	 * quickly written afterward.  But we have everything we need
	 * to set it absolutely here, as long as the format-specific
	 * register sets list it first.
	 */
	com7 = ovfmt->regs[0].value;
	com7 |= wsize->com7_bit;
	ov7670_write(sd, REG_COM7, com7);
	/*
	 * Now write the rest of the array.  Also store start/stops
	 */
	ov7670_write_array(sd, ovfmt->regs + 1);
	ov7670_set_hw(sd, wsize->hstart, wsize->hstop, wsize->vstart,
			wsize->vstop);
	ret = 0;
	if (wsize->regs)
		ret = ov7670_write_array(sd, wsize->regs);
	info->fmt = ovfmt;

	/*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 * (Update) Now that we retain clkrc state, we should be able
	 * to write it unconditionally, and that will make the frame
	 * rate persistent too.
	 */
	if (ret == 0)
		ret = ov7670_write(sd, REG_CLKRC, info->clkrc);
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov7670_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct ov7670_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = OV7670_FRAME_RATE;
	if ((info->clkrc & CLK_EXT) == 0 && (info->clkrc & CLK_SCALE) > 1)
		cp->timeperframe.denominator /= (info->clkrc & CLK_SCALE);
	return 0;
}

static int ov7670_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct ov7670_info *info = to_state(sd);
	int div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*OV7670_FRAME_RATE)/tpf->denominator;
	if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	info->clkrc = (info->clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = OV7670_FRAME_RATE/div;
	return ov7670_write(sd, REG_CLKRC, info->clkrc);
}



/*
 * Code for dealing with controls.
 */





static int ov7670_store_cmatrix(struct v4l2_subdev *sd,
		int matrix[CMATRIX_LEN])
{
	csi_dev_dbg("%s: called!\n",__func__);
	int i, ret;
	unsigned char signbits = 0;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	ret = ov7670_read(sd, REG_CMATRIX_SIGN, &signbits);
	signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += ov7670_write(sd, REG_CMATRIX_BASE + i, raw);
	}
	ret += ov7670_write(sd, REG_CMATRIX_SIGN, signbits);
	return ret;
}


/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#define SIN_STEP 5
static const int ov7670_sin_table[] = {
	   0,	 87,   173,   258,   342,   422,
	 499,	573,   642,   707,   766,   819,
	 866,	906,   939,   965,   984,   996,
	1000
};

static int ov7670_sine(int theta)
{
	csi_dev_dbg("%s: called!\n",__func__);
	int chs = 1;
	int sine;

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = ov7670_sin_table[theta/SIN_STEP];
	else {
		theta -= 90;
		sine = 1000 - ov7670_sin_table[theta/SIN_STEP];
	}
	return sine*chs;

}

static int ov7670_cosine(int theta)
{
	csi_dev_dbg("%s: called!\n",__func__);
	theta = 90 - theta;
	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return ov7670_sine(theta);
}




static void ov7670_calc_cmatrix(struct ov7670_info *info,
		int matrix[CMATRIX_LEN])
{
	csi_dev_dbg("%s: called!\n",__func__);
	int i;
	/*
	 * Apply the current saturation setting first.
	 */
	for (i = 0; i < CMATRIX_LEN; i++)
		matrix[i] = (info->fmt->cmatrix[i]*info->sat) >> 7;
	/*
	 * Then, if need be, rotate the hue value.
	 */
	if (info->hue != 0) {
		int sinth, costh, tmpmatrix[CMATRIX_LEN];

		memcpy(tmpmatrix, matrix, CMATRIX_LEN*sizeof(int));
		sinth = ov7670_sine(info->hue);
		costh = ov7670_cosine(info->hue);

		matrix[0] = (matrix[3]*sinth + matrix[0]*costh)/1000;
		matrix[1] = (matrix[4]*sinth + matrix[1]*costh)/1000;
		matrix[2] = (matrix[5]*sinth + matrix[2]*costh)/1000;
		matrix[3] = (matrix[3]*costh - matrix[0]*sinth)/1000;
		matrix[4] = (matrix[4]*costh - matrix[1]*sinth)/1000;
		matrix[5] = (matrix[5]*costh - matrix[2]*sinth)/1000;
	}
}



static int ov7670_s_sat(struct v4l2_subdev *sd, int value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct ov7670_info *info = to_state(sd);
	int matrix[CMATRIX_LEN];
	int ret;

	info->sat = value;
	ov7670_calc_cmatrix(info, matrix);
	ret = ov7670_store_cmatrix(sd, matrix);
	return ret;
}

static int ov7670_g_sat(struct v4l2_subdev *sd, __s32 *value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct ov7670_info *info = to_state(sd);

	*value = info->sat;
	return 0;
}

static int ov7670_s_hue(struct v4l2_subdev *sd, int value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct ov7670_info *info = to_state(sd);
	int matrix[CMATRIX_LEN];
	int ret;

	if (value < -180 || value > 180)
		return -EINVAL;
	info->hue = value;
	ov7670_calc_cmatrix(info, matrix);
	ret = ov7670_store_cmatrix(sd, matrix);
	return ret;
}


static int ov7670_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct ov7670_info *info = to_state(sd);

	*value = info->hue;
	return 0;
}


/*
 * Some weird registers seem to store values in a sign/magnitude format!
 */
static unsigned char ov7670_sm_to_abs(unsigned char v)
{
	if ((v & 0x80) == 0)
		return v + 128;
	return 128 - (v & 0x7f);
}


static unsigned char ov7670_abs_to_sm(unsigned char v)
{
	csi_dev_dbg("%s: called!\n",__func__);
	if (v > 127)
		return v & 0x7f;
	return (128 - v) | 0x80;
}

static int ov7670_s_brightness(struct v4l2_subdev *sd, int value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	unsigned char com8 = 0, v;
	int ret;

	ov7670_read(sd, REG_COM8, &com8);
	com8 &= ~COM8_AEC;
	ov7670_write(sd, REG_COM8, com8);
	v = ov7670_abs_to_sm(value);
	ret = ov7670_write(sd, REG_BRIGHT, v);
	return ret;
}

static int ov7670_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	unsigned char v = 0;
	int ret = ov7670_read(sd, REG_BRIGHT, &v);

	*value = ov7670_sm_to_abs(v);
	return ret;
}

static int ov7670_s_contrast(struct v4l2_subdev *sd, int value)
{
	return ov7670_write(sd, REG_CONTRAS, (unsigned char) value);
}

static int ov7670_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	unsigned char v = 0;
	int ret = ov7670_read(sd, REG_CONTRAS, &v);

	*value = v;
	return ret;
}

static int ov7670_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char v = 0;

	ret = ov7670_read(sd, REG_MVFP, &v);
	*value = (v & MVFP_MIRROR) == MVFP_MIRROR;
	return ret;
}


static int ov7670_s_hflip(struct v4l2_subdev *sd, int value)
{
	unsigned char v = 0;
	int ret;

	ret = ov7670_read(sd, REG_MVFP, &v);
	if (value)
		v |= MVFP_MIRROR;
	else
		v &= ~MVFP_MIRROR;
	msleep(10);  /* FIXME */
	ret += ov7670_write(sd, REG_MVFP, v);
	return ret;
}



static int ov7670_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char v = 0;

	ret = ov7670_read(sd, REG_MVFP, &v);
	*value = (v & MVFP_FLIP) == MVFP_FLIP;
	return ret;
}


static int ov7670_s_vflip(struct v4l2_subdev *sd, int value)
{
	unsigned char v = 0;
	int ret;

	ret = ov7670_read(sd, REG_MVFP, &v);
	if (value)
		v |= MVFP_FLIP;
	else
		v &= ~MVFP_FLIP;
	msleep(10);  /* FIXME */
	ret += ov7670_write(sd, REG_MVFP, v);
	return ret;
}

/*
 * GAIN is split between REG_GAIN and REG_VREF[7:6].  If one believes
 * the data sheet, the VREF parts should be the most significant, but
 * experience shows otherwise.  There seems to be little value in
 * messing with the VREF bits, so we leave them alone.
 */
static int ov7670_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char gain;

	ret = ov7670_read(sd, REG_GAIN, &gain);
	*value = gain;
	return ret;
}

static int ov7670_s_gain(struct v4l2_subdev *sd, int value)
{
	int ret;
	unsigned char com8;

	ret = ov7670_write(sd, REG_GAIN, value & 0xff);
	/* Have to turn off AGC as well */
	if (ret == 0) {
		ret = ov7670_read(sd, REG_COM8, &com8);
		ret = ov7670_write(sd, REG_COM8, com8 & ~COM8_AGC);
	}
	return ret;
}

/*
 * Tweak autogain.
 */
static int ov7670_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char com8;

	ret = ov7670_read(sd, REG_COM8, &com8);
	*value = (com8 & COM8_AGC) != 0;
	return ret;
}

static int ov7670_s_autogain(struct v4l2_subdev *sd, int value)
{
	int ret;
	unsigned char com8;

	ret = ov7670_read(sd, REG_COM8, &com8);
	if (ret == 0) {
		if (value)
			com8 |= COM8_AGC;
		else
			com8 &= ~COM8_AGC;
		ret = ov7670_write(sd, REG_COM8, com8);
	}
	return ret;
}

/*
 * Exposure is spread all over the place: top 6 bits in AECHH, middle
 * 8 in AECH, and two stashed in COM1 just for the hell of it.
 */
static int ov7670_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char com1, aech, aechh;

	ret = ov7670_read(sd, REG_COM1, &com1) +
		ov7670_read(sd, REG_AECH, &aech) +
		ov7670_read(sd, REG_AECHH, &aechh);
	*value = ((aechh & 0x3f) << 10) | (aech << 2) | (com1 & 0x03);
	return ret;
}

static int ov7670_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	unsigned char com1, com8, aech, aechh;

	ret = ov7670_read(sd, REG_COM1, &com1) +
		ov7670_read(sd, REG_COM8, &com8);
		ov7670_read(sd, REG_AECHH, &aechh);
	if (ret)
		return ret;

	com1 = (com1 & 0xfc) | (value & 0x03);
	aech = (value >> 2) & 0xff;
	aechh = (aechh & 0xc0) | ((value >> 10) & 0x3f);
	ret = ov7670_write(sd, REG_COM1, com1) +
		ov7670_write(sd, REG_AECH, aech) +
		ov7670_write(sd, REG_AECHH, aechh);
	/* Have to turn off AEC as well */
	if (ret == 0)
		ret = ov7670_write(sd, REG_COM8, com8 & ~COM8_AEC);
	return ret;
}

/*
 * Tweak autoexposure.
 */
static int ov7670_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	unsigned char com8;
	enum v4l2_exposure_auto_type *atype = (enum v4l2_exposure_auto_type *) value;

	ret = ov7670_read(sd, REG_COM8, &com8);
	if (com8 & COM8_AEC)
		*atype = V4L2_EXPOSURE_AUTO;
	else
		*atype = V4L2_EXPOSURE_MANUAL;
	return ret;
}

static int ov7670_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	int ret;
	unsigned char com8;

	ret = ov7670_read(sd, REG_COM8, &com8);
	if (ret == 0) {
		if (value == V4L2_EXPOSURE_AUTO)
			com8 |= COM8_AEC;
		else
			com8 &= ~COM8_AEC;
		ret = ov7670_write(sd, REG_COM8, com8);
	}
	return ret;
}



static int ov7670_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
	case V4L2_CID_CONTRAST:
		return v4l2_ctrl_query_fill(qc, 0, 127, 1, 64);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//	case V4L2_CID_SATURATION:
//		return v4l2_ctrl_query_fill(qc, 0, 256, 1, 128);
//	case V4L2_CID_HUE:
//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
	case V4L2_CID_AUTOGAIN:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 0, 500, 1, 256);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	/*case V4L2_CID_CAMERA_FLASH_MODE:
	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);*/
	}
	return -EINVAL;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct ov7670_info *info = to_state(sd);
	enum v4l2_flash_mode *flash_mode = (enum v4l2_flash_mode*)value;

	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_mode value)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct ov7670_info *info = to_state(sd);
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

static int ov7670_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	csi_dev_dbg("%s: called!\n",__func__);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov7670_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return ov7670_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return ov7670_g_sat(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return ov7670_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return ov7670_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return ov7670_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return ov7670_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return ov7670_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return ov7670_g_exp(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return ov7670_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int ov7670_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	csi_dev_dbg("%s: called!\n",__func__);
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return ov7670_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return ov7670_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return ov7670_s_sat(sd, ctrl->value);
	case V4L2_CID_HUE:
		return ov7670_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return ov7670_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return ov7670_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return ov7670_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return ov7670_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return ov7670_s_exp(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return ov7670_s_autoexp(sd,
				(enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return sensor_s_flash_mode(sd,
	      (enum v4l2_flash_mode) ctrl->value);
	}
	return -EINVAL;
}

static int ov7670_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_OV7670, 0);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov7670_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val = 0;
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ret = ov7670_read(sd, reg->reg & 0xff, &val);
	reg->val = val;
	reg->size = 1;
	return ret;
}

static int ov7670_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	ov7670_write(sd, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ov7670_core_ops = {
	.g_chip_ident = ov7670_g_chip_ident,
	.g_ctrl = ov7670_g_ctrl,
	.s_ctrl = ov7670_s_ctrl,
	.queryctrl = ov7670_queryctrl,
	.reset = ov7670_reset,
	.init = ov7670_init,
	.s_power = ov7670_power,
	.ioctl = sensor_ioctl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov7670_g_register,
	.s_register = ov7670_s_register,
#endif
};

static const struct v4l2_subdev_video_ops ov7670_video_ops = {
	.enum_mbus_fmt = ov7670_enum_fmt,//linux-3.0
	.enum_framesizes	= sensor_enum_size,
	.enum_frameintervals	= sensor_enum_intervals,
	.try_mbus_fmt = ov7670_try_fmt,//linux-3.0
	.s_mbus_fmt = ov7670_s_fmt,//linux-3.0
	.s_parm = ov7670_s_parm,
	.g_parm = ov7670_g_parm,
};

static const struct v4l2_subdev_ops ov7670_ops = {
	.core = &ov7670_core_ops,
	.video = &ov7670_video_ops,
};

/* ----------------------------------------------------------------------- */

static int ov7670_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct v4l2_subdev *sd;
	struct ov7670_info *info;
//	int ret;

	info = kzalloc(sizeof(struct ov7670_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &ov7670_ops);

	info->fmt = &ov7670_formats[0];
	info->ccm_info = &ccm_info_con;
	info->sat = 128;	/* Review this */
	info->clkrc = 1;	/* 30fps */

	return 0;
}


static int ov7670_remove(struct i2c_client *client)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id ov7670_id[] = {
	{ "ov7670", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov7670_id);

//linux-3.0
static struct i2c_driver ov7670_driver = {
	.driver = {
		.owner = THIS_MODULE,
	  .name = "ov7670",
	 },
	.probe = ov7670_probe,
	.remove = ov7670_remove,
	.id_table = ov7670_id,
};

static __init int init_ov7670(void)
{
	return i2c_add_driver(&ov7670_driver);
}

static __exit void exit_ov7670(void)
{
  i2c_del_driver(&ov7670_driver);
}

module_init(init_ov7670);
module_exit(exit_ov7670);
