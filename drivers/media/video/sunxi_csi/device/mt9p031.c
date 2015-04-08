/*
 * Driver for MT9P031 CMOS Image Sensor from Aptina 
 * on Allwinner (sunxi) CSI Camera Interface
 *
 * Copyright (C) 2014 AW-SoM Technologies <src@aw-som.com>
 *
 * Driver for MT9P031 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2011, Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 * Copyright (C) 2011, Javier Martin <javier.martin@vista-silicon.com>
 * Copyright (C) 2011, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on the MT9V032 driver and Bastian Hecht's code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <linux/io.h>
#include <plat/sys_config.h>
#include <mach/system.h>
#include "../include/sunxi_csi_core.h"
#include "../include/sunxi_dev_csi.h"

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

//for internel driver debug
#define DEV_DBG_EN   		0
#if(DEV_DBG_EN == 1)
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][MT9P031]"x,##arg)
#else
#define csi_dev_dbg(x,arg...)
#endif
#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][MT9P031]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][MT9P031]"x,##arg)

#define MCLK (24*1000*1000)
#define VREF_POL	CSI_HIGH
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0 for csi0

//define the voltage level of control signal
#define CSI_STBY_ON		0
#define CSI_STBY_OFF 		1
#define CSI_RST_ON		0
#define CSI_RST_OFF		1
#define CSI_PWR_ON		1
#define CSI_PWR_OFF		0

#define SENSOR_FRAME_RATE 30

#define MT9P031_PIXEL_ARRAY_WIDTH                       2752
#define MT9P031_PIXEL_ARRAY_HEIGHT                      2004

#define MT9P031_CHIP_VERSION                            0x00
#define         MT9P031_CHIP_VERSION_VALUE              0x1801
#define MT9P031_ROW_START                               0x01
#define         MT9P031_ROW_START_MIN                   0
#define         MT9P031_ROW_START_MAX                   2004
#define         MT9P031_ROW_START_DEF                   54
#define MT9P031_COLUMN_START                            0x02
#define         MT9P031_COLUMN_START_MIN                0
#define         MT9P031_COLUMN_START_MAX                2750
#define         MT9P031_COLUMN_START_DEF                16
#define MT9P031_WINDOW_HEIGHT                           0x03
#define         MT9P031_WINDOW_HEIGHT_MIN               2
#define         MT9P031_WINDOW_HEIGHT_MAX               2006
#define         MT9P031_WINDOW_HEIGHT_DEF               1944
#define MT9P031_WINDOW_WIDTH                            0x04
#define         MT9P031_WINDOW_WIDTH_MIN                2
#define         MT9P031_WINDOW_WIDTH_MAX                2752
#define         MT9P031_WINDOW_WIDTH_DEF                2592
#define MT9P031_HORIZONTAL_BLANK                        0x05
#define         MT9P031_HORIZONTAL_BLANK_MIN            0
#define         MT9P031_HORIZONTAL_BLANK_MAX            4095
#define MT9P031_VERTICAL_BLANK                          0x06
#define         MT9P031_VERTICAL_BLANK_MIN              1
#define         MT9P031_VERTICAL_BLANK_MAX              4096
#define         MT9P031_VERTICAL_BLANK_DEF              26
#define MT9P031_OUTPUT_CONTROL                          0x07
#define         MT9P031_OUTPUT_CONTROL_CEN              2
#define         MT9P031_OUTPUT_CONTROL_SYN              1
#define         MT9P031_OUTPUT_CONTROL_DEF              0x1f82
#define MT9P031_SHUTTER_WIDTH_UPPER                     0x08
#define MT9P031_SHUTTER_WIDTH_LOWER                     0x09
#define         MT9P031_SHUTTER_WIDTH_MIN               1
#define         MT9P031_SHUTTER_WIDTH_MAX               1048575
#define         MT9P031_SHUTTER_WIDTH_DEF               1943
#define MT9P031_PLL_CONTROL                             0x10
#define         MT9P031_PLL_CONTROL_PWROFF              0x0050
#define         MT9P031_PLL_CONTROL_PWRON               0x0051
#define         MT9P031_PLL_CONTROL_USEPLL              0x0052
#define MT9P031_PLL_CONFIG_1                            0x11
#define MT9P031_PLL_CONFIG_2                            0x12
#define MT9P031_PIXEL_CLOCK_CONTROL                     0x0a
#define         MT9P031_PIXEL_CLOCK_INVERT              (1 << 15)
#define         MT9P031_PIXEL_CLOCK_SHIFT(n)            ((n) << 8)
#define         MT9P031_PIXEL_CLOCK_DIVIDE(n)           ((n) << 0)
#define MT9P031_FRAME_RESTART                           0x0b
#define MT9P031_SHUTTER_DELAY                           0x0c
#define MT9P031_RST                                     0x0d
#define         MT9P031_RST_ENABLE                      1
#define         MT9P031_RST_DISABLE                     0
#define MT9P031_READ_MODE_1                             0x1e
#define         MT9P031_READ_MODE_1_INV_TRIG		(1 << 9)
#define         MT9P031_READ_MODE_1_SNAPSHOT		(1 << 8)
#define         MT9P031_READ_MODE_1_GLOBAL_RESET	(1 << 7)
#define         MT9P031_READ_MODE_1_BULB_EXP		(1 << 6)
#define         MT9P031_READ_MODE_1_INV_STROBE		(1 << 5)
#define         MT9P031_READ_MODE_1_STROBE		(1 << 4)
#define         MT9P031_READ_MODE_1_STROBE_START	(1 << 2)
#define         MT9P031_READ_MODE_1_STROBE_END		(1 << 0)

#define MT9P031_READ_MODE_2                             0x20
#define         MT9P031_READ_MODE_2_ROW_MIR             (1 << 15)
#define         MT9P031_READ_MODE_2_COL_MIR             (1 << 14)
#define         MT9P031_READ_MODE_2_ROW_BLC             (1 << 6)
#define MT9P031_ROW_ADDRESS_MODE                        0x22
#define MT9P031_COLUMN_ADDRESS_MODE                     0x23
#define MT9P031_GLOBAL_GAIN                             0x35
#define         MT9P031_GLOBAL_GAIN_MIN                 8
#define         MT9P031_GLOBAL_GAIN_MAX                 65536
#define         MT9P031_GLOBAL_GAIN_DEF                 8
#define         MT9P031_GLOBAL_GAIN_MULT                (1 << 6)
#define MT9P031_ROW_BLACK_TARGET                        0x49
#define MT9P031_ROW_BLACK_DEF_OFFSET                    0x4b
#define MT9P031_GREEN1_OFFSET                           0x60
#define MT9P031_GREEN2_OFFSET                           0x61
#define MT9P031_BLACK_LEVEL_CALIBRATION                 0x62
#define         MT9P031_BLC_MANUAL_BLC                  (1 << 0)
#define MT9P031_RED_OFFSET                              0x63
#define MT9P031_BLUE_OFFSET                             0x64
#define MT9P031_TEST_PATTERN                            0xa0
#define         MT9P031_TEST_PATTERN_SHIFT              3
#define         MT9P031_TEST_PATTERN_ENABLE             (1 << 0)
#define         MT9P031_TEST_PATTERN_DISABLE            (0 << 0)
#define MT9P031_TEST_PATTERN_GREEN                      0xa1
#define MT9P031_TEST_PATTERN_RED                        0xa2
#define MT9P031_TEST_PATTERN_BLUE                       0xa3

enum mt9p031_model {
        MT9P031_MODEL_COLOR,
        MT9P031_MODEL_MONOCHROME,
};

__csi_subdev_info_t ccm_info_con =
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};
struct mt9p031 {
        struct v4l2_subdev subdev;
        struct media_pad pad;
        struct v4l2_rect crop;  /* Sensor window */
        struct v4l2_mbus_framefmt format;
	struct sensor_format_struct *fmt;
	__csi_subdev_info_t *ccm_info;
        struct mt9p031_platform_data *pdata;
        struct mutex power_lock; /* lock to protect power_count */
        int power_count;

        //struct clk *clk;
        struct regulator_bulk_data regulators[3];

        enum mt9p031_model model;
        unsigned int clk_div;
        int reset;

        struct v4l2_ctrl_handler ctrls;
        struct v4l2_ctrl *blc_auto;
        struct v4l2_ctrl *blc_offset;

	int debug_reg;
        /* Registers cache */
	int oc;
        u16 output_control;
        u16 mode1, mode2;
	int hflip;
	int vflip;
	int gain;
	int exp;
};

static struct mt9p031 *to_mt9p031(struct v4l2_subdev *sd)
{
        return container_of(sd, struct mt9p031, subdev);
}

static int mt9p031_read(struct i2c_client *client, u8 reg)
{
        return i2c_smbus_read_word_swapped(client, reg);
}

static int mt9p031_write(struct i2c_client *client, u8 reg, u16 data)
{
        return i2c_smbus_write_word_swapped(client, reg, data);
}

static int mt9p031_set_output_control(struct mt9p031 *mt9p031, u16 clear,
                                      u16 set)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	csi_dev_dbg("%s: clear=0x%00x set=0x%00x\n",__func__, clear, set);
        u16 value = (mt9p031->output_control & ~clear) | set;
        int ret;
	csi_dev_dbg("%s: MT9P031_OUTPUT_CONTROL (0x%x) value=0x%00x\n",__func__, MT9P031_OUTPUT_CONTROL, value);
        ret = mt9p031_write(client, MT9P031_OUTPUT_CONTROL, value);
        if (ret < 0)
                return ret;

        mt9p031->output_control = value;
        return 0;
}

static int mt9p031_set_mode2(struct mt9p031 *mt9p031, u16 clear, u16 set)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
        u16 value = (mt9p031->mode2 & ~clear) | set;
        int ret;
	csi_dev_dbg("%s: clear=0x%00x set=0x%00x value=0x%00x\n",__func__, clear, set, value);
        ret = mt9p031_write(client, MT9P031_READ_MODE_2, value);
        if (ret < 0)
                return ret;

        mt9p031->mode2 = value;
        return 0;
}
static int mt9p031_set_mode1(struct mt9p031 *mt9p031, u16 clear, u16 set)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
        u16 value = (mt9p031->mode1 & ~clear) | set;
        int ret;
	csi_dev_dbg("%s: clear=0x%00x set=0x%00x value=0x%00x\n",__func__, clear, set, value);
        ret = mt9p031_write(client, MT9P031_READ_MODE_1, value);
        if (ret < 0)
                return ret;

        mt9p031->mode1 = value;
        return 0;
}
static int mt9p031_reset(struct mt9p031 *mt9p031)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
        int ret;

        /* Disable chip output, synchronous option update */
	csi_dev_dbg("%s: set MT9P031_RST (0x%00x) val=0x%00x\n",__func__,MT9P031_RST, MT9P031_RST_ENABLE);
        ret = mt9p031_write(client, MT9P031_RST, MT9P031_RST_ENABLE);
        if (ret < 0)
                return ret;
	csi_dev_dbg("%s: set MT9P031_RST (0x%00x) val=0x%00x\n",__func__,MT9P031_RST, MT9P031_RST_DISABLE);
        ret = mt9p031_write(client, MT9P031_RST, MT9P031_RST_DISABLE);
        if (ret < 0)
                return ret;

	csi_dev_dbg("%s: set MT9P031_PIXEL_CLOCK_CONTROL (0x%00x) val=(1<<0x%00x)\n",__func__,MT9P031_PIXEL_CLOCK_CONTROL,mt9p031->clk_div);
        ret = mt9p031_write(client, MT9P031_PIXEL_CLOCK_CONTROL, mt9p031->clk_div );
        if (ret < 0)
                return ret;

        return 0;
}
static int mt9p031_pll_enable(struct mt9p031 *info)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&info->subdev);
        ret = mt9p031_write(client, MT9P031_PLL_CONTROL,
                            MT9P031_PLL_CONTROL_PWRON);
        if (ret < 0)
                return ret;

        ret = mt9p031_write(client, MT9P031_PLL_CONFIG_1, 0x1003-info->oc); //10/03+1=96Mhz x1002=120Mhz 1001=144Mhz 
        if (ret < 0)
                return ret;

        /*ret = mt9p031_write(client, MT9P031_PLL_CONFIG_2, 0x0000);
        if (ret < 0)
                return ret;*/

        msleep(100);
        ret = mt9p031_write(client, MT9P031_PLL_CONTROL,
                            MT9P031_PLL_CONTROL_PWRON |
                            MT9P031_PLL_CONTROL_USEPLL);
        return ret;
}
/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static int mt9p031_set_params(struct mt9p031 *mt9p031)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
        struct v4l2_mbus_framefmt *format = &mt9p031->format;
        const struct v4l2_rect *crop = &mt9p031->crop;
        unsigned int hblank;
        unsigned int vblank;
        unsigned int xskip;
        unsigned int yskip;
        unsigned int xbin;
        unsigned int ybin;
        int ret;

        /* Windows position and size.
         *
         * TODO: Make sure the start coordinates and window size match the
         * skipping, binning and mirroring (see description of registers 2 and 4
         * in table 13, and Binning section on page 41).
         */
	csi_dev_dbg("%s: set MT9P031_COLUMN_START (0x%00x) val=(%d) 0x%00x\n",
			__func__,MT9P031_COLUMN_START,crop->left,crop->left);
        ret = mt9p031_write(client, MT9P031_COLUMN_START, crop->left);
        if (ret < 0)
                return ret;
	csi_dev_dbg("%s: set MT9P031_ROW_START (0x%00x) val=(%d) 0x%00x\n",
			__func__,MT9P031_ROW_START,crop->top,crop->top);
        ret = mt9p031_write(client, MT9P031_ROW_START, crop->top);
        if (ret < 0)
                return ret;
	csi_dev_dbg("%s: set MT9P031_WINDOW_WIDTH (0x%00x) val=(%d) 0x%00x\n",
			__func__,MT9P031_WINDOW_WIDTH,crop->width,crop->width);
        ret = mt9p031_write(client, MT9P031_WINDOW_WIDTH, crop->width - 1);
        if (ret < 0)
                return ret;
	csi_dev_dbg("%s: set MT9P031_WINDOW_HEIGHT (0x%00x) val=(%d) 0x%00x\n",
			__func__,MT9P031_WINDOW_HEIGHT,crop->height,crop->height);
        ret = mt9p031_write(client, MT9P031_WINDOW_HEIGHT, crop->height - 1);
        if (ret < 0)
                return ret;

        /* Row and column binning and skipping. Use the maximum binning value
         * compatible with the skipping settings.
         */
        xskip = DIV_ROUND_CLOSEST(crop->width, format->width);
        yskip = DIV_ROUND_CLOSEST(crop->height, format->height);
        xbin = 1 << (ffs(xskip) - 1);
        ybin = 1 << (ffs(yskip) - 1);

	csi_dev_dbg("%s: set MT9P031_COLUMN_ADDRESS_MODE (0x%00x) val=0x%00x\n",__func__,MT9P031_COLUMN_ADDRESS_MODE,((xbin - 1) << 4) | (xskip - 1));
        ret = mt9p031_write(client, MT9P031_COLUMN_ADDRESS_MODE,
                            ((xbin - 1) << 4) | (xskip - 1));
        if (ret < 0)
                return ret;
	csi_dev_dbg("%s: set MT9P031_ROW_ADDRESS_MODE (0x%00x) val=0x%00x\n",__func__,MT9P031_ROW_ADDRESS_MODE,((ybin - 1) << 4) | (yskip - 1));
        ret = mt9p031_write(client, MT9P031_ROW_ADDRESS_MODE,
                            ((ybin - 1) << 4) | (yskip - 1));
        if (ret < 0)
                return ret;

        /* Blanking - use minimum value for horizontal blanking and default
         * value for vertical blanking.
         */
        hblank = 346 * ybin + 64 + (80 >> min_t(unsigned int, xbin, 3));
        vblank = MT9P031_VERTICAL_BLANK_DEF;

        ret = mt9p031_write(client, MT9P031_HORIZONTAL_BLANK, hblank - 1);
        if (ret < 0)
                return ret;
        ret = mt9p031_write(client, MT9P031_VERTICAL_BLANK, vblank - 1);
        if (ret < 0)
                return ret;

        return ret;
}
static int mt9p031_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_fh *fh,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(sd);

        if (code->pad || code->index)
                return -EINVAL;

        code->code = mt9p031->format.code;
        return 0;
}
static int mt9p031_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(sd);

        if (index)
                return -EINVAL;

        *code = mt9p031->format.code;
        return 0;
}

static int mt9p031_enum_frame_size(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_fh *fh,
                                   struct v4l2_subdev_frame_size_enum *fse)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(sd);

        if (fse->index >= 8 /*|| fse->code != mt9p031->format.code*/)
                return -EINVAL;

        fse->min_width = MT9P031_WINDOW_WIDTH_DEF
                       / min_t(unsigned int, 7, fse->index + 1);
        fse->max_width = fse->min_width;
        fse->min_height = MT9P031_WINDOW_HEIGHT_DEF / (fse->index + 1);
        fse->max_height = fse->min_height;

	csi_dev_dbg("%s: min_h=%d max_h=%d min_w=%d max_w=%d called!\n",
			__func__,fse->min_height,fse->max_height,fse->min_width,fse->max_width);

        return 0;
}
static int mt9p031_enum_framesize(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_frame_size_enum *fse)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(sd);

        if (fse->index >= 8 || fse->code != mt9p031->format.code)
                return -EINVAL;

        fse->min_width = MT9P031_WINDOW_WIDTH_DEF
                       / min_t(unsigned int, 7, fse->index + 1);
        fse->max_width = fse->min_width;
        fse->min_height = MT9P031_WINDOW_HEIGHT_DEF / (fse->index + 1);
        fse->max_height = fse->min_height;

        return 0;
}
static int mt9p031_frame_rates[] = { 30, 15, 10, 5, 1 };

static int mt9p031_enum_intervals(struct v4l2_subdev *sd,
		struct v4l2_frmivalenum *interval)
{
	csi_dev_dbg("%s: called!\n",__func__);
	if (interval->index >= ARRAY_SIZE(mt9p031_frame_rates))
		return -EINVAL;
	interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	interval->discrete.numerator = 1;
	interval->discrete.denominator = mt9p031_frame_rates[interval->index];
	return 0;
}
static struct v4l2_mbus_framefmt *
__mt9p031_get_pad_format(struct mt9p031 *mt9p031, struct v4l2_subdev_fh *fh,
                         unsigned int pad, u32 which)
{
	csi_dev_dbg("%s: called!\n",__func__);
        switch (which) {
        case V4L2_SUBDEV_FORMAT_TRY:
                return v4l2_subdev_get_try_format(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
                return &mt9p031->format;
        default:
                return NULL;
        }
}

static struct v4l2_rect *
__mt9p031_get_pad_crop(struct mt9p031 *mt9p031, struct v4l2_subdev_fh *fh,
                     unsigned int pad, u32 which)
{
	csi_dev_dbg("%s: called!\n",__func__);
        switch (which) {
        case V4L2_SUBDEV_FORMAT_TRY:
                return v4l2_subdev_get_try_crop(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
                return &mt9p031->crop;
        default:
                return NULL;
        }
}

static int mt9p031_get_format(struct v4l2_subdev *subdev,
                              struct v4l2_subdev_fh *fh,
                              struct v4l2_subdev_format *fmt)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);

        fmt->format = *__mt9p031_get_pad_format(mt9p031, fh, fmt->pad,
                                                fmt->which);
        return 0;
}

static int mt9p031_set_format(struct v4l2_subdev *subdev,
                              struct v4l2_subdev_fh *fh,
                              struct v4l2_subdev_format *format)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);
        struct v4l2_mbus_framefmt *__format;
        struct v4l2_rect *__crop;
        unsigned int width;
        unsigned int height;
        unsigned int hratio;
        unsigned int vratio;

        __crop = __mt9p031_get_pad_crop(mt9p031, fh, format->pad,
                                        format->which);

        /* Clamp the width and height to avoid dividing by zero. */
        width = clamp_t(unsigned int, ALIGN(format->format.width, 2),
                        max_t(unsigned int, __crop->width / 7,
                              MT9P031_WINDOW_WIDTH_MIN),
                        __crop->width);
        height = clamp_t(unsigned int, ALIGN(format->format.height, 2),
                         max_t(unsigned int, __crop->height / 8,
                               MT9P031_WINDOW_HEIGHT_MIN),
                         __crop->height);

        hratio = DIV_ROUND_CLOSEST(__crop->width, width);
        vratio = DIV_ROUND_CLOSEST(__crop->height, height);

        __format = __mt9p031_get_pad_format(mt9p031, fh, format->pad,
                                            format->which);
        __format->width = __crop->width / hratio;
        __format->height = __crop->height / vratio;

        format->format = *__format;

        return 0;
}

static int mt9p031_get_crop(struct v4l2_subdev *subdev, struct v4l2_crop *crop)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);

	struct v4l2_crop c;
	crop->c=mt9p031->crop;
    return 0;
}

static int mt9p031_set_crop(struct v4l2_subdev *subdev, struct v4l2_crop *crop)
{
		csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);
        struct v4l2_mbus_framefmt *__format;
        struct v4l2_rect rect;

        /* Clamp the crop rectangle boundaries and align them to a multiple of 2
         * pixels to ensure a GRBG Bayer pattern.
         */
		csi_dev_dbg("request crop (%d, %d, %d, %d)\n",crop->c.left,crop->c.top,crop->c.width,crop->c.height);
        rect.left = clamp(ALIGN(crop->c.left, 2), MT9P031_COLUMN_START_MIN,
                          MT9P031_COLUMN_START_MAX);
        rect.top = clamp(ALIGN(crop->c.top, 2), MT9P031_ROW_START_MIN,
                         MT9P031_ROW_START_MAX);
        rect.width = clamp_t(unsigned int, ALIGN(crop->c.width, 2),
                             MT9P031_WINDOW_WIDTH_MIN,
                             MT9P031_WINDOW_WIDTH_MAX);
        rect.height = clamp_t(unsigned int, ALIGN(crop->c.height, 2),
                              MT9P031_WINDOW_HEIGHT_MIN,
                              MT9P031_WINDOW_HEIGHT_MAX);

        rect.width = min_t(unsigned int, rect.width,
                           MT9P031_PIXEL_ARRAY_WIDTH - rect.left);
        rect.height = min_t(unsigned int, rect.height,
                            MT9P031_PIXEL_ARRAY_HEIGHT - rect.top);

		mt9p031->crop = rect;
    	crop->c = rect;
		return mt9p031_set_params(mt9p031);
}


/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */
	switch (qc->id) {
	case V4L2_CID_REG_RW_ADDR:
		return v4l2_ctrl_query_fill(qc, -1, 0xffff, 1, 1);
	case V4L2_CID_REG_RW:
		return v4l2_ctrl_query_fill(qc, 0, 0xffff, 1, 1);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, MT9P031_GLOBAL_GAIN_MIN,
                          MT9P031_GLOBAL_GAIN_MAX, 1, MT9P031_GLOBAL_GAIN_DEF);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, MT9P031_SHUTTER_WIDTH_MIN,
                          MT9P031_SHUTTER_WIDTH_MAX, 1,
                          MT9P031_SHUTTER_WIDTH_DEF);
	case V4L2_CID_BLC_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_BLC_TARGET_LEVEL:
		return v4l2_ctrl_query_fill(qc, 0, 4095, 1, 168);
	case V4L2_CID_BLC_ANALOG_OFFSET:
		return v4l2_ctrl_query_fill(qc, -255, 255, 1, 32);
	case V4L2_CID_BLC_DIGITAL_OFFSET:
		return v4l2_ctrl_query_fill(qc, -2048, 2047, 1, 40);
	case V4L2_CID_TEST_PATTERN:
		return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
	case V4L2_CID_PIXEL_RATE:
		return v4l2_ctrl_query_fill(qc, 0, 3, 1, 0);		
	case V4L2_CID_OVERCLOCK_CAM:
		return v4l2_ctrl_query_fill(qc, 0, 2, 1, 0);
	case V4L2_CID_SNAPSHOT_ENABLED:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_GLOBAL_RESET:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_BULB_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_SNAPSHOT_TRIGGER:
		return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
	case V4L2_CID_SNAPSHOT_TRIGGER_INV:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_STROBE_ENABLED:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_STROBE_INV:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_STROBE_START:
		return v4l2_ctrl_query_fill(qc, 0, 3, 1, 1);
	case V4L2_CID_STROBE_END:
		return v4l2_ctrl_query_fill(qc, 0, 3, 1, 2);

	}
	return -EINVAL;
}

static int mt9p031_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct mt9p031 *mt9p031 = container_of(sd, struct mt9p031, subdev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret=-1;
	switch (ctrl->id) {
	case V4L2_CID_REG_RW_ADDR:
		ctrl->value = mt9p031->debug_reg;
		return 0;
	case V4L2_CID_REG_RW:
		if(mt9p031->debug_reg>=0)
			ctrl->value = mt9p031_read(client, mt9p031->debug_reg);
		else{
			printk("Ignore REG_RW_ADDR=-1!\n");
			ctrl->value=0;
		}
		return 0;
	case V4L2_CID_VFLIP:
		ctrl->value = ((mt9p031->mode2&MT9P031_READ_MODE_2_ROW_MIR)>0);
		csi_dev_dbg("%s: V4L2_CID_VFLIP ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_HFLIP:
		ctrl->value = ((mt9p031->mode2&MT9P031_READ_MODE_2_COL_MIR)>0);
		csi_dev_dbg("%s: V4L2_CID_HFLIP ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_GAIN:
		ctrl->value = mt9p031_read(client, MT9P031_GLOBAL_GAIN);
		csi_dev_dbg("%s: V4L2_CID_GAIN ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_EXPOSURE:
		ctrl->value = (mt9p031_read(client, MT9P031_SHUTTER_WIDTH_UPPER)<<16)+mt9p031_read(client, MT9P031_SHUTTER_WIDTH_LOWER);
		csi_dev_dbg("%s: V4L2_CID_EXPOSURE ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_BLC_AUTO:
		ctrl->value = mt9p031_read(client, MT9P031_BLACK_LEVEL_CALIBRATION) ? 0 : MT9P031_BLC_MANUAL_BLC;
		return 0;
	case V4L2_CID_BLC_TARGET_LEVEL:
		ctrl->value = mt9p031_read(client, MT9P031_ROW_BLACK_TARGET);
		return 0;
	case V4L2_CID_BLC_ANALOG_OFFSET:
		ctrl->value = mt9p031_read(client, MT9P031_GREEN1_OFFSET );
		return 0;
	case V4L2_CID_BLC_DIGITAL_OFFSET:
		ctrl->value = mt9p031_read(client, MT9P031_ROW_BLACK_DEF_OFFSET);
		return 0;
        case V4L2_CID_TEST_PATTERN:
		ret = mt9p031_read(client, MT9P031_TEST_PATTERN);
		ctrl->value = (ret&MT9P031_TEST_PATTERN_ENABLE)?( ret >> MT9P031_TEST_PATTERN_SHIFT):MT9P031_TEST_PATTERN_DISABLE;
		csi_dev_dbg("%s: V4L2_CID_TEST_PATTERN ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_PIXEL_RATE:
		ctrl->value = mt9p031->clk_div;
		csi_dev_dbg("%s: V4L2_CID_PIXEL_RATE ctrl->value=0x%00x\n",__func__, ctrl->value);		
		return 0;
	case V4L2_CID_OVERCLOCK_CAM:
		ctrl->value = mt9p031->oc;
		csi_dev_dbg("%s: V4L2_CID_OVERCLOCK_CAM ctrl->value=0x%00x\n",__func__, ctrl->value);	
		return 0;
	case V4L2_CID_SNAPSHOT_ENABLED:
		ctrl->value = ( ( mt9p031_read(client, MT9P031_READ_MODE_1) & MT9P031_READ_MODE_1_SNAPSHOT ) > 0 );
		csi_dev_dbg("%s: V4L2_CID_SNAPSHOT_ENABLED ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_GLOBAL_RESET:
		ctrl->value = ( ( mt9p031_read(client, MT9P031_READ_MODE_1) & MT9P031_READ_MODE_1_GLOBAL_RESET ) > 0 );
		csi_dev_dbg("%s: V4L2_CID_GLOBAL_RESET ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_BULB_EXPOSURE:
		ctrl->value = ( ( mt9p031_read(client, MT9P031_READ_MODE_1) & MT9P031_READ_MODE_1_BULB_EXP ) > 0 );
		csi_dev_dbg("%s: BULB_EXPOSURE ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_SNAPSHOT_TRIGGER:
		ctrl->value = mt9p031_read(client, MT9P031_FRAME_RESTART);
		csi_dev_dbg("%s: V4L2_CID_SNAPSHOT_TRIGGER ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_SNAPSHOT_TRIGGER_INV:
		ctrl->value = ( ( mt9p031_read(client, MT9P031_READ_MODE_1) & MT9P031_READ_MODE_1_INV_TRIG ) > 0 );
		csi_dev_dbg("%s: V4L2_CID_SNAPSHOT_TRIGGER_INV ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_STROBE_ENABLED:
		ctrl->value = ( ( mt9p031_read(client, MT9P031_READ_MODE_1) & MT9P031_READ_MODE_1_STROBE ) > 0 );
		csi_dev_dbg("%s: V4L2_CID_STROBE_ENABLED ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_STROBE_INV:
		ctrl->value = ( ( mt9p031_read(client, MT9P031_READ_MODE_1) & MT9P031_READ_MODE_1_INV_STROBE ) > 0 );
		csi_dev_dbg("%s: V4L2_CID_STROBE_ENABLED ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_STROBE_START:
		ctrl->value = ( mt9p031_read(client, MT9P031_READ_MODE_1) & (3<<2) )>>2;
		csi_dev_dbg("%s: V4L2_CID_STROBE_ENABLED ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	case V4L2_CID_STROBE_END:
		ctrl->value = ( mt9p031_read(client, MT9P031_READ_MODE_1) & 3 );
		csi_dev_dbg("%s: V4L2_CID_STROBE_ENABLED ctrl->value=0x%00x\n",__func__, ctrl->value);
		return 0;
	}
	csi_dev_dbg("%s: no match for control 0x%00X\n",__func__, ctrl->id);
	return -EINVAL;
}
static int mt9p031_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = container_of(sd, struct mt9p031, subdev);//container_of(ctrl->handler, struct mt9p031, ctrls);
        //struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 data;
        int ret=-1;

        switch (ctrl->id) {
	case V4L2_CID_REG_RW_ADDR:
		mt9p031->debug_reg=(ctrl->value & 0xFFFF);
		return 0;
	case V4L2_CID_REG_RW:
		if(mt9p031->debug_reg > 0)
			mt9p031_write(client, (mt9p031->debug_reg & 0xFFFF), (ctrl->value & 0xFFFF));
		return 0;
        case V4L2_CID_EXPOSURE:
		csi_dev_dbg("%s: V4L2_CID_EXPOSURE 0x%00x\n",__func__,ctrl->value);
                ret = mt9p031_write(client, MT9P031_SHUTTER_WIDTH_UPPER,
                                    (ctrl->value >> 16) & 0xffff);
                if (ret < 0)
                        return ret;

                return mt9p031_write(client, MT9P031_SHUTTER_WIDTH_LOWER,
                                     ctrl->value & 0xffff);

        case V4L2_CID_GAIN:
		csi_dev_dbg("%s: V4L2_CID_GAIN 0x%00x\n",__func__,ctrl->value);
                /* Gain is controlled by 2 analog stages and a digital stage.
                 * Valid values for the 3 stages are
                 *
                 * Stage                Min     Max     Step
                 * ------------------------------------------
                 * First analog stage   x1      x2      1
                 * Second analog stage  x1      x4      0.125
                 * Digital stage        x1      x16     0.125
                 *
                 * To minimize noise, the gain stages should be used in the
                 * second analog stage, first analog stage, digital stage order.
                 * Gain from a previous stage should be pushed to its maximum
                 * value before the next stage is used.
                 */
                if (ctrl->value <= 32) {
                        data = ctrl->value;
                } else if (ctrl->value <= 64) {
                        ctrl->value &= ~1;
                        data = (1 << 6) | (ctrl->value >> 1);
                } else {
                        ctrl->value &= ~7;
                        data = ((ctrl->value - 64) << 5) | (1 << 6) | 32;
                }
		csi_dev_dbg("%s: V4L2_CID_GAIN 0x%00x\n",__func__, data);
                return mt9p031_write(client, MT9P031_GLOBAL_GAIN, data);

        case V4L2_CID_HFLIP:
		csi_dev_dbg("%s: V4L2_CID_HFLIP 0x%00x\n",__func__,ctrl->value);
                if (ctrl->value)
                        ret = mt9p031_set_mode2(mt9p031,
                                        0, MT9P031_READ_MODE_2_COL_MIR);
                else
                        ret = mt9p031_set_mode2(mt9p031,
                                        MT9P031_READ_MODE_2_COL_MIR, 0);
		csi_dev_dbg("%s: V4L2_CID_HFLIP ret=0x%00x\n",__func__, ret);
		return ret;

        case V4L2_CID_VFLIP:
		csi_dev_dbg("%s: V4L2_CID_VFLIP 0x%00x\n",__func__,ctrl->value);
                if (ctrl->value)
                        ret = mt9p031_set_mode2(mt9p031,
                                        0, MT9P031_READ_MODE_2_ROW_MIR);
                else
                        ret = mt9p031_set_mode2(mt9p031,
                                        MT9P031_READ_MODE_2_ROW_MIR, 0);
		csi_dev_dbg("%s: V4L2_CID_HFLIP ret=0x%00x\n",__func__, ret);
		return ret;

        case V4L2_CID_TEST_PATTERN:
                if (!ctrl->value) {
                        // Restore the black level compensation settings.
                        if (mt9p031->blc_auto->cur.val != 0) {
                                ret = mt9p031_s_ctrl(sd, mt9p031->blc_auto);
                                if (ret < 0)
                                        return ret;
                        }
                        if (mt9p031->blc_offset->cur.val != 0) {
                                ret = mt9p031_s_ctrl(sd, mt9p031->blc_offset);
                                if (ret < 0)
                                        return ret;
                        }
                        return mt9p031_write(client, MT9P031_TEST_PATTERN,
                                             MT9P031_TEST_PATTERN_DISABLE);
                }

                ret = mt9p031_write(client, MT9P031_TEST_PATTERN_GREEN, 0x05a0);
                if (ret < 0)
                        return ret;
                ret = mt9p031_write(client, MT9P031_TEST_PATTERN_RED, 0x0a50);
                if (ret < 0)
                        return ret;
                ret = mt9p031_write(client, MT9P031_TEST_PATTERN_BLUE, 0x0aa0);
                if (ret < 0)
                        return ret;

                 //Disable digital black level compensation when using a test
                 //pattern.
                 
                ret = mt9p031_set_mode2(mt9p031, MT9P031_READ_MODE_2_ROW_BLC,0);
                if (ret < 0)
                        return ret;

                ret = mt9p031_write(client, MT9P031_ROW_BLACK_DEF_OFFSET, 0);
                if (ret < 0)
                        return ret;

                return mt9p031_write(client, MT9P031_TEST_PATTERN,
                                ((ctrl->value - 1) << MT9P031_TEST_PATTERN_SHIFT)
                                | MT9P031_TEST_PATTERN_ENABLE);
        case V4L2_CID_BLC_AUTO:
		csi_dev_dbg("%s: V4L2_CID_BLC_AUTO\n",__func__);
                ret = mt9p031_set_mode2(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_2_ROW_BLC,
                                ctrl->value ? MT9P031_READ_MODE_2_ROW_BLC : 0);
                if (ret < 0)
                        return ret;

                return mt9p031_write(client, MT9P031_BLACK_LEVEL_CALIBRATION,
                                     ctrl->value ? 0 : MT9P031_BLC_MANUAL_BLC);

        case V4L2_CID_BLC_TARGET_LEVEL:
		csi_dev_dbg("%s: V4L2_CID_BLC_TARGET_LEVEL\n",__func__);
                return mt9p031_write(client, MT9P031_ROW_BLACK_TARGET,
                                     ctrl->value);

        case V4L2_CID_BLC_ANALOG_OFFSET:
		csi_dev_dbg("%s: V4L2_CID_BLC_ANALOG_OFFSET\n",__func__);
                data = ctrl->value & ((1 << 9) - 1);

                ret = mt9p031_write(client, MT9P031_GREEN1_OFFSET, data);
                if (ret < 0)
                        return ret;
                ret = mt9p031_write(client, MT9P031_GREEN2_OFFSET, data);
                if (ret < 0)
                        return ret;
                ret = mt9p031_write(client, MT9P031_RED_OFFSET, data);
                if (ret < 0)
                        return ret;
                return mt9p031_write(client, MT9P031_BLUE_OFFSET, data);

        case V4L2_CID_BLC_DIGITAL_OFFSET:
		csi_dev_dbg("%s: V4L2_CID_BLC_DIGITAL_OFFSET\n",__func__);
                return mt9p031_write(client, MT9P031_ROW_BLACK_DEF_OFFSET,
                                     ctrl->value & ((1 << 12) - 1));
	case V4L2_CID_PIXEL_RATE:
		mt9p031->clk_div = ( ctrl->value & 3 ); 
		mt9p031_write(client, MT9P031_PIXEL_CLOCK_CONTROL, mt9p031->clk_div);
                return 0; /*mt9p031_write(client, xxx, ctrl->value & 3);*/
	case V4L2_CID_OVERCLOCK_CAM:
		csi_dev_dbg("%s: V4L2_CID_OVERCLOCK_CAM %d\n",__func__, ctrl->value);
		mt9p031->oc = ctrl->value & 3;
		return mt9p031_pll_enable(mt9p031);
	case V4L2_CID_SNAPSHOT_ENABLED:
		csi_dev_dbg("%s: V4L2_CID_SNAPSHOT_ENABLED %d\n",__func__, ctrl->value);
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_1_SNAPSHOT,
                                ctrl->value ? MT9P031_READ_MODE_1_SNAPSHOT : 0);
	case V4L2_CID_GLOBAL_RESET:
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_1_GLOBAL_RESET,
                                ctrl->value ? MT9P031_READ_MODE_1_GLOBAL_RESET : 0);
		return 0;
	case V4L2_CID_BULB_EXPOSURE:
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_1_BULB_EXP,
                                ctrl->value ? MT9P031_READ_MODE_1_BULB_EXP : 0);
		return 0;
	case V4L2_CID_SNAPSHOT_TRIGGER://FIXME
		csi_dev_dbg("%s: V4L2_CID_SNAPSHOT_TRIGGER %d\n",__func__, ctrl->value);
		mt9p031_write(client, MT9P031_FRAME_RESTART, ctrl->value);
                return; 
	case V4L2_CID_SNAPSHOT_TRIGGER_INV:
		csi_dev_dbg("%s: V4L2_CID_SNAPSHOT_TRIGGER_INV %d\n",__func__, ctrl->value);
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_1_INV_TRIG,
                                ctrl->value ? MT9P031_READ_MODE_1_INV_TRIG : 0);
	case V4L2_CID_STROBE_ENABLED:
		csi_dev_dbg("%s: V4L2_CID_STROBE_ENABLED %d\n",__func__, ctrl->value);
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_1_STROBE,
                                ctrl->value ? MT9P031_READ_MODE_1_STROBE : 0);
	case V4L2_CID_STROBE_INV:
		csi_dev_dbg("%s: V4L2_CID_STROBE_INV %d\n",__func__, ctrl->value);
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : MT9P031_READ_MODE_1_INV_STROBE,
                                ctrl->value ? MT9P031_READ_MODE_1_INV_STROBE : 0);
	case V4L2_CID_STROBE_START://FIXME
		csi_dev_dbg("%s: V4L2_CID_STROBE_START %d\n",__func__, ctrl->value);
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : 3<<2,
                                ctrl->value ? (ctrl->value&3)<<2 : 0);
	case V4L2_CID_STROBE_END://FIXME
		csi_dev_dbg("%s: V4L2_CID_STROBE_END %d\n",__func__, ctrl->value);
                return mt9p031_set_mode1(mt9p031,
                                ctrl->value ? 0 : 3,
                                ctrl->value ? (ctrl->value&3) : 0);
        }
	csi_dev_dbg("%s: no match for control 0x%00X\n",__func__, ctrl->id);
        return -EINVAL;
}
static int mt9p031_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct mt9p031 *mt9p031 = container_of(sd, struct mt9p031, subdev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, last=0;
	for(i=0; i<ctrls->count; i++){
		//printk("Add ext control %d of %d %X",i,ctrls->count,ctrls->controls->id);
		struct v4l2_ext_control *ctrl = ctrls->controls + i;
		if (last) {
			ctrls->error_idx = i;
			return 0;
		}
	}
	return 0;
}
static int mt9p031_s_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = container_of(sd, struct mt9p031, subdev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 data;
        int ret=-1;

        /*switch (ctrls->controls->id) {
        }*/
	csi_dev_dbg("%s: no match for control 0x%00X\n",__func__, ctrls->controls->id);
        return -EINVAL;
}
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct mt9p031 *info = to_mt9p031(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;

	if (info->format.width > 1280 && info->format.height > 720) {
		cp->timeperframe.denominator = SENSOR_FRAME_RATE/2;
	}
	else {
		cp->timeperframe.denominator = SENSOR_FRAME_RATE;
	}

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	return 0;
}
static struct sensor_format_struct {
	__u8 *desc;
	enum v4l2_mbus_pixelcode mbus_code;
	int bpp;
} sensor_formats[] = {
	{
		.desc		= "SBGGR8_1X8",
		.mbus_code	= V4L2_MBUS_FMT_SGRBG8_1X8,
		.bpp		= 2,
	}
};
static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt,
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct mt9p031 *info = to_mt9p031(sd);
	if(fmt->code != info->format.code){
		printk("format not supported!\n");
		return -EINVAL;
	}

	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats;
	csi_dev_dbg("%s: called!\n",__func__);
	fmt->field = V4L2_FIELD_NONE;
	csi_dev_dbg("%s: called!\n",__func__);
	if(fmt->width%2!=0)
		fmt->width++;
	if(fmt->height%2!=0)
		fmt->height++;
	csi_dev_dbg("%s: req size %dx%d\n",__func__, fmt->width,fmt->height);
	if(fmt->width<MT9P031_WINDOW_WIDTH_MIN)
		fmt->width=MT9P031_WINDOW_WIDTH_MIN;
	if(fmt->width>MT9P031_WINDOW_WIDTH_MAX)
		fmt->width=MT9P031_WINDOW_WIDTH_MAX;
	if(fmt->height<MT9P031_WINDOW_HEIGHT_MIN)
		fmt->height=MT9P031_WINDOW_HEIGHT_MIN;
	if(fmt->height>MT9P031_WINDOW_HEIGHT_MAX)
		fmt->height=MT9P031_WINDOW_HEIGHT_MAX;

	info->crop.height=fmt->height;
	info->crop.width=fmt->width;
	info->format.height=fmt->height;
	info->format.width=fmt->width;
	csi_dev_dbg("%s: got size %dx%d\n",__func__, fmt->width,fmt->height);

	return mt9p031_set_params(info);
}

static int sensor_try_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}
static int sensor_s_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

static struct v4l2_ctrl_ops mt9p031_ctrl_ops = {
        .s_ctrl = mt9p031_s_ctrl,
};

static const char * const mt9p031_test_pattern_menu[] = {
        "Disabled",
        "Color Field",
        "Horizontal Gradient",
        "Vertical Gradient",
        "Diagonal Gradient",
        "Classic Test Pattern",
        "Walking 1s",
        "Monochrome Horizontal Bars",
        "Monochrome Vertical Bars",
        "Vertical Color Bars",
};
static const struct v4l2_ctrl_config mt9p031_ctrls[] = {
        {
                .ops            = &mt9p031_ctrl_ops,
                .id             = V4L2_CID_BLC_AUTO,
                .type           = V4L2_CTRL_TYPE_BOOLEAN,
                .name           = "BLC, Auto",
                .min            = 0,
                .max            = 1,
                .step           = 1,
                .def            = 1,
                .flags          = 0,
        }, {
                .ops            = &mt9p031_ctrl_ops,
                .id             = V4L2_CID_BLC_TARGET_LEVEL,
                .type           = V4L2_CTRL_TYPE_INTEGER,
                .name           = "BLC Target Level",
                .min            = 0,
                .max            = 4095,
                .step           = 1,
                .def            = 168,
                .flags          = 0,
        }, {
                .ops            = &mt9p031_ctrl_ops,
                .id             = V4L2_CID_BLC_ANALOG_OFFSET,
                .type           = V4L2_CTRL_TYPE_INTEGER,
                .name           = "BLC Analog Offset",
                .min            = -255,
                .max            = 255,
                .step           = 1,
                .def            = 32,
                .flags          = 0,
        }, {
                .ops            = &mt9p031_ctrl_ops,
                .id             = V4L2_CID_BLC_DIGITAL_OFFSET,
                .type           = V4L2_CTRL_TYPE_INTEGER,
                .name           = "BLC Digital Offset",
                .min            = -2048,
                .max            = 2047,
                .step           = 1,
                .def            = 40,
                .flags          = 0,
        }, {
                .ops            = &mt9p031_ctrl_ops,
                .id             = V4L2_CID_TEST_PATTERN,
                .type           = V4L2_CTRL_TYPE_INTEGER,
                .name           = "Enable Test Pattern",
                .min            = 0,
                .max            = 10,
                .step           = 1,
                .def            = 0,
                .flags          = 0,
        }
};


/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */
static int mt9p031_set_power(struct v4l2_subdev *subdev, int on)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(subdev->v4l2_dev->dev);
	struct mt9p031 *info = to_mt9p031(subdev);
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
	csi_dev_dbg("%s: called!\n",__func__);
  switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_reset_str);
			msleep(100);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			msleep(100);
			clk_disable(dev->csi_module_clk);

			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			msleep(10);
			//reset off io
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			msleep(10);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);

			ret = mt9p031_reset(info);
        		if (ret < 0){
				csi_dev_err("mt9p031_reset failed!\n");
                		return ret;
			}

        		ret = mt9p031_set_params(info);
        		if (ret < 0){
				csi_dev_err("mt9p031_set_params failed!\n");
                		return ret;
			}

			//enable pll
			mt9p031_pll_enable(info);
			ret = mt9p031_set_mode2(info, MT9P031_READ_MODE_2_ROW_BLC, 0);
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");

			clk_disable(dev->csi_module_clk);
			//power on reset
			gpio_set_one_pin_io_status(dev->csi_pin_hd,1,csi_stby_str);//set the gpio to output
			gpio_set_one_pin_io_status(dev->csi_pin_hd,1,csi_reset_str);//set the gpio to output
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_ON,csi_stby_str);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_ON,csi_stby_str);

			clk_enable(dev->csi_module_clk);
			msleep(10);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			msleep(10);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
			msleep(10);
			break;

		case CSI_SUBDEV_PWR_OFF:
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
			gpio_write_one_pin_value(dev->csi_pin_hd,CSI_RST_OFF,csi_stby_str);
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
	csi_dev_dbg("%s: called!\n",__func__);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct mt9p031 *info = to_mt9p031(sd);
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
static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	csi_dev_dbg("%s: called!\n",__func__);
	int ret=0;

	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO:
		{
			struct mt9p031 *info = to_mt9p031(sd);
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
			struct mt9p031 *info = to_mt9p031(sd);
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
static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	csi_dev_dbg("%s: called!\n",__func__);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, MT9P031_CHIP_VERSION_VALUE, 0);
}
/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int mt9p031_registered(struct v4l2_subdev *subdev)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct i2c_client *client = v4l2_get_subdevdata(subdev);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);
        s32 data;
        int ret;

        ret = mt9p031_set_power(subdev, CSI_SUBDEV_PWR_ON); //mt9p031_power_on(mt9p031);
        if (ret < 0) {
                dev_err(&client->dev, "MT9P031 power up failed\n");
                return ret;
        }


        /* Read out the chip version register */
        data = mt9p031_read(client, MT9P031_CHIP_VERSION);

        if (data != MT9P031_CHIP_VERSION_VALUE) {
                dev_err(&client->dev, "%s: CSI MT9P031 not detected, wrong version "
                        "0x%04x\n", __func__, data);
                return -ENODEV;
        }

        dev_info(&client->dev, "%s: CSI MT9P031 camera detected at address 0x%02x\n",
        		__func__, client->addr);

        return 0;
}

static int mt9p031_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);
        struct v4l2_mbus_framefmt *format;
        struct v4l2_rect *crop;

        crop = v4l2_subdev_get_try_crop(fh, 0);
        crop->left = MT9P031_COLUMN_START_DEF;
        crop->top = MT9P031_ROW_START_DEF;
        crop->width = MT9P031_WINDOW_WIDTH_DEF;
        crop->height = MT9P031_WINDOW_HEIGHT_DEF;

        format = v4l2_subdev_get_try_format(fh, 0);

        format->code = V4L2_MBUS_FMT_Y8_1X8;

        format->width = MT9P031_WINDOW_WIDTH_DEF;
        format->height = MT9P031_WINDOW_HEIGHT_DEF;
        format->field = V4L2_FIELD_NONE;
        format->colorspace = V4L2_COLORSPACE_SRGB;

        return mt9p031_set_power(subdev, CSI_SUBDEV_PWR_ON);
}

static int mt9p031_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	csi_dev_dbg("%s: called!\n",__func__);
        return mt9p031_set_power(subdev, CSI_SUBDEV_PWR_OFF);
}

static struct v4l2_subdev_core_ops mt9p031_subdev_core_ops = {
        .s_power        = mt9p031_set_power,

	.g_chip_ident	= sensor_g_chip_ident,
	.g_ctrl 	= mt9p031_g_ctrl,
	.s_ctrl 	= mt9p031_s_ctrl,
	.queryctrl 	= sensor_queryctrl,
	.reset 		= sensor_reset,
	.init		= mt9p031_registered, //sensor_init,
	.ioctl		= sensor_ioctl,
};

static struct v4l2_subdev_video_ops mt9p031_subdev_video_ops = {
	.enum_mbus_fmt		= mt9p031_enum_fmt,
	.enum_framesizes	= mt9p031_enum_framesize,
	.enum_frameintervals	= mt9p031_enum_intervals,
	.g_crop			= mt9p031_get_crop,
	.s_crop			= mt9p031_set_crop,
	.try_mbus_fmt		= sensor_try_fmt,
	.s_mbus_fmt		= sensor_s_fmt,
	.s_parm 		= sensor_s_parm,
	.g_parm		= sensor_g_parm,
};

static struct v4l2_subdev_pad_ops mt9p031_subdev_pad_ops = {
        .enum_mbus_code = mt9p031_enum_mbus_code,
        .enum_frame_size = mt9p031_enum_frame_size,
        .get_fmt = mt9p031_get_format,
        .set_fmt = mt9p031_set_format,
        .get_crop = mt9p031_get_crop,
        .set_crop = mt9p031_set_crop,
};

static struct v4l2_subdev_ops mt9p031_subdev_ops = {
        .core   = &mt9p031_subdev_core_ops,
        .video  = &mt9p031_subdev_video_ops,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */
static int mt9p031_probe(struct i2c_client *client,
                         const struct i2c_device_id *did)
{
		csi_dev_dbg("%s: called!\n",__func__);
        //struct mt9p031_platform_data *pdata = mt9p031_get_pdata(client);
        struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
        struct mt9p031 *mt9p031;
        unsigned int i;
		int data;
        int ret=0;
	

        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
                dev_warn(&client->dev,
                        "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
                return -EIO;
        }

        mt9p031 = devm_kzalloc(&client->dev, sizeof(*mt9p031), GFP_KERNEL);
        if (mt9p031 == NULL)
                return -ENOMEM;

		mt9p031->fmt = &sensor_formats[0];
		mt9p031->ccm_info = &ccm_info_con;
    	mt9p031->output_control = MT9P031_OUTPUT_CONTROL_DEF;
    	mt9p031->oc = 0;
		mt9p031->debug_reg=-1;
    	mt9p031->mode1 = 0;
    	mt9p031->mode2 = MT9P031_READ_MODE_2_ROW_BLC & MT9P031_READ_MODE_2_ROW_MIR & MT9P031_READ_MODE_2_COL_MIR;
		mt9p031->clk_div=0;
        mt9p031->model = MT9P031_MODEL_MONOCHROME; //did->driver_data;
        mt9p031->reset = -1;

        v4l2_ctrl_handler_init(&mt9p031->ctrls, ARRAY_SIZE(mt9p031_ctrls) + 6);

        v4l2_ctrl_new_std(&mt9p031->ctrls, &mt9p031_ctrl_ops,
                          V4L2_CID_EXPOSURE, MT9P031_SHUTTER_WIDTH_MIN,
                          MT9P031_SHUTTER_WIDTH_MAX, 1,
                          MT9P031_SHUTTER_WIDTH_DEF);
        v4l2_ctrl_new_std(&mt9p031->ctrls, &mt9p031_ctrl_ops,
                          V4L2_CID_GAIN, MT9P031_GLOBAL_GAIN_MIN,
                          MT9P031_GLOBAL_GAIN_MAX, 1, MT9P031_GLOBAL_GAIN_DEF);
        v4l2_ctrl_new_std(&mt9p031->ctrls, &mt9p031_ctrl_ops,
                          V4L2_CID_HFLIP, 0, 1, 1, 0);
        v4l2_ctrl_new_std(&mt9p031->ctrls, &mt9p031_ctrl_ops,
                          V4L2_CID_VFLIP, 0, 1, 1, 0);


        v4l2_i2c_subdev_init(&mt9p031->subdev, client, &mt9p031_subdev_ops);

        mt9p031->crop.width = MT9P031_WINDOW_WIDTH_DEF;
        mt9p031->crop.height = MT9P031_WINDOW_HEIGHT_DEF;
        mt9p031->crop.left = MT9P031_COLUMN_START_DEF;
        mt9p031->crop.top = MT9P031_ROW_START_DEF;

        mt9p031->format.code = V4L2_MBUS_FMT_Y8_1X8;

        mt9p031->format.width = MT9P031_WINDOW_WIDTH_DEF;
        mt9p031->format.height = MT9P031_WINDOW_HEIGHT_DEF;
        mt9p031->format.field = V4L2_FIELD_NONE;
        mt9p031->format.colorspace = V4L2_COLORSPACE_SRGB;

done:
        return ret;
}

static int mt9p031_remove(struct i2c_client *client)
{
	csi_dev_dbg("%s: called!\n",__func__);
        struct v4l2_subdev *subdev = i2c_get_clientdata(client);
        struct mt9p031 *mt9p031 = to_mt9p031(subdev);
        v4l2_device_unregister_subdev(subdev);
		kfree(mt9p031);
        return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
        { "mt9p031", MT9P031_MODEL_COLOR },
        { "mt9p031m", MT9P031_MODEL_MONOCHROME },
        { }
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt9p031_of_match[] = {
        { .compatible = "aptina,mt9p031", },
        { .compatible = "aptina,mt9p031m", },
        { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt9p031_of_match);
#endif

static struct i2c_driver mt9p031_i2c_driver = {
        .driver = {
                .of_match_table = of_match_ptr(mt9p031_of_match),
                .name = "mt9p031",
        },
        .probe          = mt9p031_probe,
        .remove         = mt9p031_remove,
        .id_table       = mt9p031_id,
};

module_i2c_driver(mt9p031_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9P031 Camera driver");
MODULE_AUTHOR("Bastian Hecht <hechtb@gmail.com>");
MODULE_LICENSE("GPL v2");


