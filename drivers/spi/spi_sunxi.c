/*
 * (C) Copyright 2010-2015
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * Pan Nan <pannan@allwinnertech.com>
 * Victor Wei <weiziheng@allwinnertech.com>
 *
 * SUNXI SPI Platform Driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/io.h>
#include <plat/dma_compat.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <plat/sys_config.h>
#include <mach/spi.h>
#include <asm/cacheflush.h>

#define SUNXI_SPI_DEBUG 1

#ifdef SUNXI_SPI_DEBUG
#define spi_wrn(...)		do {printk("[spi]: %s(L%d) ", __FILE__, __LINE__); printk(__VA_ARGS__);} while(0)
#define spi_msg(...)		printk("[spi]: "__VA_ARGS__)
#else
#define spi_wrn(...)
#define spi_msg(...)
#endif


#if 0
static int hex_dump(char* str, void __iomem* buf, u32 len, u32 mode/*0-8bit, 1-16bit, 2-32bit*/)
{
    u32 i;
    u32 step[3] = {1, 2, 4};
    char* mod[3] = {"%02x ", "%04x ", "%08x "};

    printk("\nDump %s:", str);
    for (i=0; i<len; i+=step[mode])
    {
        if (!(i&0xf))
            printk("\n0x%p : ", buf + i);
        printk(mod[mode], readl(buf + i));
    }
    printk("\n");

    return 0;
}
#endif

#define SYS_SPI_PIN
#ifndef SYS_SPI_PIN
static void* __iomem gpio_addr = NULL;

/* gpio base address */
#define _PIO_BASE_ADDRESS    (0x01c20800)

/* gpio spi1 */
#define _Pn_CFG1(n) ( (n)*0x24 + 0x04 + gpio_addr )
#define _Pn_DRV1(n) ( (n)*0x24 + 0x14 + gpio_addr )
#define _Pn_PUL1(n) ( (n)*0x24 + 0x1C + gpio_addr )
#endif

//#define AW1623_FPGA

struct sunxi_spi {
    struct platform_device *pdev;
	struct spi_master *master;/* kzalloc */
#ifdef	CONFIG_SPI_SLAVE
	struct spi_slave *slave;/* kzalloc */
#endif

	void __iomem *base_addr; /* register */
	struct clk *hclk;  /* ahb spi gating bit */
	struct clk *mclk;  /* ahb spi gating bit */
	unsigned long gpio_hdle;

	#if defined(CONFIG_SUN4I_SPI_NDMA) || defined(CONFIG_SUN5I_SPI_NDMA)
		#if defined CONFIG_SUNXI_SPI_NDMA 
		enum sw_dma_ch dma_id;
		enum sw_dmadir dma_dir;
		int dma_hdle;
		#else
		enum sw_dma_ch dma_tx_id;
		enum sw_dma_ch dma_rx_id;
		int dma_tx_hdle;
		int dma_rx_hdle;
		#endif
	#elif defined CONFIG_ARCH_SUN7I 
		#if defined CONFIG_SUNXI_SPI_NDMA 
		int dma_id; 
		enum sw_dmadir dma_dir; 
		dma_hdl_t dma_hdle; 
		#else 
		//FIXME: align name with old sun4I: dma_tx_xx (not for now to ensure correct port/typesafe) ? 
		int dma_tx_id; 
		int dma_rx_id; 
		dma_hdl_t dma_tx_hdle; 
		dma_hdl_t dma_rx_hdle; 
		#endif 	
	#endif

	unsigned int irq; /* irq NO. */

	int busy;
#define SPI_FREE   (1<<0)
#define SPI_SUSPND (1<<1)
#define SPI_BUSY   (1<<2)
#define SLAVE_BUS 1

	int result; /* 0: succeed -1:fail */

	struct workqueue_struct *workqueue;
	struct work_struct work;

	struct list_head queue; /* spi messages */
	spinlock_t lock;

	struct completion done;  /* wakup another spi transfer */

	/* keep select during one message */
	void (*cs_control)(struct spi_device *spi, bool on);

/*
 * (1) enable cs1,    cs_bitmap = SPI_CHIP_SELECT_CS1;
 * (2) enable cs0&cs1,cs_bitmap = SPI_CHIP_SELECT_CS0|SPI_CHIP_SELECT_CS1;
 *
 */
#define SPI_CHIP_SELECT_CS0 (0x01)
#define SPI_CHIP_SELECT_CS1 (0x02)

	int cs_bitmap;/* cs0- 0x1; cs1-0x2, cs0&cs1-0x3. */
};

#ifdef CONFIG_HARDWARE_LIB_SPI
/* keep unused burst */
static void aw_spi_clear_dhb(void *base_addr)
{
	u32 reg_val = readl(base_addr+SPI_CTL_REG);
	reg_val &= ~SPI_CTL_DHB;
	writel(reg_val, base_addr + SPI_CTL_REG);
}
#endif

static void aw_reg_dump(void *base_addr, int len)
{
	printk("%s: spi called!\n",__func__);
	int x;
	u32 reg_val;
	printk("Dump from base_addr=%X to %X\n",base_addr,base_addr+(len*4));
	for(x=0; x<len*4; x+=4){
		reg_val = readl(base_addr+x);
		printk("0x%08X ",reg_val);
	}
	printk("\n");
}
/* config chip select */
s32 aw_spi_set_cs(u32 chipselect, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);

    if (chipselect < 4)
    {
        reg_val &= ~SPI_CTL_SS_MASK;/* SS-chip select, clear two bits */
        reg_val |= chipselect << SPI_SS_BIT_POS;/* set chip select */

        writel(reg_val, base_addr + SPI_CTL_REG);

        //spi_msg("Chip Select set succeed! cs = %d\n", chipselect);
        return AW_SPI_OK;
    }
    else
    {
        spi_wrn("Chip Select set fail! cs = %d\n", chipselect);
        return AW_SPI_FAIL;
    }
}

/* select dma type */
s32 aw_spi_sel_dma_type(u32 dma_type, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);

    //spi_msg("set dma type %s\n", dma_mode ? "ddma" : "ndma");
    reg_val &= ~SPI_CTL_DMAMOD;
    if (dma_type)
    {
        reg_val |= 1 << 5;
    }
    writel(reg_val, base_addr + SPI_CTL_REG);
    return AW_SPI_OK;
}

/* config spi */
void aw_spi_config(u32 master, u32 config, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);

    /*1. POL */
    if (config & SPI_POL_ACTIVE_)
    {
        reg_val |= SPI_CTL_POL;
    }
    else
    {
        reg_val &= ~SPI_CTL_POL;/*default POL = 0 */
    }
    /*2. PHA */
    if (config & SPI_PHA_ACTIVE_)
    {
        reg_val |= SPI_CTL_PHA;
    }
    else
    {
        reg_val &= ~SPI_CTL_PHA;/*default PHA = 0 */
    }
    /*3. SSPOL,chip select signal polarity */
    if (config & SPI_CS_HIGH_ACTIVE_)
    {
        reg_val &= ~SPI_CTL_SSPOL;
    }
    else
    {
        reg_val |= SPI_CTL_SSPOL;/*default SSPOL = 1,Low level effective */
    }
    /*4. LMTF--LSB/MSB transfer first select */
    if (config & SPI_LSB_FIRST_ACTIVE_)
    {
        reg_val |= SPI_CTL_LMTF;
    }
    else
    {
        reg_val &= ~SPI_CTL_LMTF;/*default LMTF =0, MSB first */
    }

    /*master mode: set DDB,DHB,SMC,SSCTL*/
    if(master == 1)
    {
        /*5. dummy burst type */
        if (config & SPI_DUMMY_ONE_ACTIVE_)
        {
            reg_val |= SPI_CTL_DDB;
        }
        else
        {
            reg_val &= ~SPI_CTL_DDB;/*default DDB =0, ZERO */
        }
        /*6.discard hash burst-DHB */
        if (config & SPI_RECEIVE_ALL_ACTIVE_)
        {
            reg_val &= ~SPI_CTL_DHB;
        }
        else
        {
            reg_val |= SPI_CTL_DHB;/*default DHB =1, discard unused burst */
        }

        /*7. set SMC = 1 , SSCTL = 0 ,TPE = 1 */
        reg_val &= ~SPI_CTL_SSCTL;
        reg_val |=  SPI_CTL_T_PAUSE_EN;
    }
    else
    {
        /* tips for slave mode config */
        spi_msg("slave mode configurate control register.\n");
    }
    writel(reg_val, base_addr + SPI_CTL_REG);
    return;
}

/* restore reg data after transfer complete */
void aw_spi_restore_state(u32 master, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);

/*
 * config spi control register
 * | 15 |  14  |  13  |  12  |  11  |  10  |  9  |  8  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * | DHB|  DDB |     SS      | SMC  |  XCH |TFRST|RFRST|SSCTL| MSB | TBW |SSPOL| POL | PHA | MOD | EN  |
 * | 1  |   0  |     00      |  1   |   0  |  0  |  0  |  0  |  0  |  0  |  1  |  1  |  1  |  1  |  1  |
 */
    /* master mode */
    if(master)
    {
        reg_val |= (SPI_CTL_DHB|SPI_CTL_SSPOL|SPI_CTL_POL|SPI_CTL_PHA|SPI_CTL_FUNC_MODE|SPI_CTL_EN);

        /* new bit,transmit pause enable,stop smart dummy when rxfifo full*/
        reg_val |= SPI_CTL_T_PAUSE_EN;

        /* |SPI_CTL_TBW); //deleted SPI_CTL_TBW bit for aw1623, this bit is defined for dma mode select, 2011-5-26 19:55:32 */
        reg_val &= ~(SPI_CTL_DDB|SPI_CTL_SS_MASK|SPI_CTL_XCH|SPI_CTL_SSCTL|SPI_CTL_LMTF);
    }
    else/* slave mode */
    {
        reg_val |= (SPI_CTL_SSPOL|SPI_CTL_POL|SPI_CTL_PHA||SPI_CTL_EN);

        /* |SPI_CTL_TBW);//deleted SPI_CTL_TBW bit for aw1623, this bit is defined for dma mode select, 2011-5-26 19:55:32 */
        reg_val &= ~(SPI_CTL_DHB|SPI_CTL_FUNC_MODE|SPI_CTL_DDB|SPI_CTL_SS_MASK|SPI_CTL_XCH|SPI_CTL_SSCTL|SPI_CTL_LMTF);
    }
    spi_msg("control register set default value: %x \n", reg_val);
    writel(reg_val, base_addr + SPI_CTL_REG);
    return;
}

/* set spi clock */
void aw_spi_set_clk(u32 spi_clk, u32 ahb_clk, void *base_addr)
{
    u32 reg_val = 0;
    u32 N = 0;
    u32 div_clk = (ahb_clk>>1)/spi_clk;

    //spi_msg("set spi clock %d, mclk %d\n", spi_clk, ahb_clk);
    reg_val = readl(base_addr + SPI_CLK_RATE_REG);

    /* CDR2 */
    if(div_clk <= SPI_CLK_SCOPE)
    {
        if (div_clk != 0)
        {
            div_clk--;
        }
        reg_val &= ~SPI_CLKCTL_CDR2;
        reg_val |= (div_clk|SPI_CLKCTL_DRS);
        //spi_msg("CDR2 - n = %d \n", div_clk);
    }
    else /* CDR1 */
    {
        //search 2^N
        while(1)
        {
            if(div_clk == 1)
            {
                break;
            }
            div_clk >>= 1;//divide by 2
            N++;
        };
        reg_val &= ~(SPI_CLKCTL_CDR1|SPI_CLKCTL_DRS);
        reg_val |= (N<<8);
        //spi_msg("CDR1 - n = %d \n", N);
    }
    writel(reg_val, base_addr + SPI_CLK_RATE_REG);
    return;
}

/* start spi transfer */
void aw_spi_start_xfer(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    reg_val |= SPI_CTL_XCH;
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* query tranfer is completed in smc mode */
u32 aw_spi_query_xfer(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    return (reg_val & SPI_CTL_XCH);
}

/* enable spi bus */
void aw_spi_enable_bus(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    reg_val |= SPI_CTL_EN;
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* disbale spi bus */
void aw_spi_disable_bus(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    reg_val &= ~SPI_CTL_EN;
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* set master mode */
void aw_spi_set_master(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    reg_val |= SPI_CTL_FUNC_MODE;
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* set slave mode */
void aw_spi_set_slave(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    reg_val &= ~SPI_CTL_FUNC_MODE;
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* disable irq type */
void aw_spi_disable_irq(u32 bitmap, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_INT_CTL_REG);
    bitmap &= SPI_INTEN_MASK;
    reg_val &= ~bitmap;
    writel(reg_val, base_addr + SPI_INT_CTL_REG);
}

/* enable irq type */
void aw_spi_enable_irq(u32 bitmap, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_INT_CTL_REG);
    bitmap &= SPI_INTEN_MASK;
    reg_val |= bitmap;
    writel(reg_val, (base_addr + SPI_INT_CTL_REG));
}

/* disable dma irq */
void aw_spi_disable_dma_irq(u32 bitmap, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_DMA_CTL_REG);
    bitmap &= SPI_DRQEN_MASK;
    reg_val &= ~bitmap;
    writel(reg_val, base_addr + SPI_DMA_CTL_REG);
}

/* enable dma irq */
void aw_spi_enable_dma_irq(u32 bitmap, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_DMA_CTL_REG);
    bitmap &= SPI_DRQEN_MASK;
    reg_val |= bitmap;
    writel(reg_val, base_addr + SPI_DMA_CTL_REG);
}

/* query irq pending */
u32 aw_spi_qry_irq_pending(void *base_addr)
{
    return ( SPI_STAT_MASK & readl(base_addr + SPI_STATUS_REG) );
}

/* clear irq pending */
void aw_spi_clr_irq_pending(u32 pending_bit, void *base_addr)
{
    pending_bit &= SPI_STAT_MASK;
    writel(pending_bit, base_addr + SPI_STATUS_REG);
}

/* query txfifo bytes */
u32 aw_spi_query_txfifo(void *base_addr)
{
    u32 reg_val = ( SPI_FIFO_TXCNT & readl(base_addr + SPI_FIFO_STA_REG) );
    reg_val >>= SPI_TXCNT_BIT_POS;
    return reg_val;
}

/* query rxfifo bytes */
u32 aw_spi_query_rxfifo(void *base_addr)
{
    u32 reg_val = ( SPI_FIFO_RXCNT & readl(base_addr + SPI_FIFO_STA_REG) );
    reg_val >>= SPI_RXCNT_BIT_POS;
    return reg_val;
}

/* reset fifo */
void aw_spi_reset_fifo(void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    reg_val |= (SPI_CTL_RST_RXFIFO|SPI_CTL_RST_TXFIFO);
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* set transfer total length BC and transfer length WTC */
void aw_spi_set_bc_wtc(u32 bc, u32 tc, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_BC_REG);
    reg_val &= ~SPI_BC_BC_MASK;
    reg_val |= ( SPI_BC_BC_MASK & bc );
    writel(reg_val, base_addr + SPI_BC_REG);
    //spi_msg("\n-- BC = %d --\n", readl(base_addr + SPI_BC_REG));

    reg_val = readl(base_addr + SPI_TC_REG);
    reg_val &= ~SPI_TC_WTC_MASK;
    reg_val |= (SPI_TC_WTC_MASK & tc);
    writel(reg_val, base_addr + SPI_TC_REG);
    //spi_msg("\n-- TC = %d --\n", readl(base_addr + SPI_TC_REG));
}

/* set ss control */
void aw_spi_ss_ctrl(void *base_addr, u32 on_off)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    on_off &= 0x1;
    if(on_off)
    {
        reg_val |= SPI_CTL_SS_CTRL;
    }
    else
    {
        reg_val &= ~SPI_CTL_SS_CTRL;
    }
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* set ss level */
void aw_spi_ss_level(void *base_addr, u32 hi_lo)
{
    u32 reg_val = readl(base_addr + SPI_CTL_REG);
    hi_lo &= 0x1;
    if(hi_lo)
    {
        reg_val |= SPI_CTL_SS_LEVEL;
    }
    else
    {
        reg_val &= ~SPI_CTL_SS_LEVEL;
    }
    writel(reg_val, base_addr + SPI_CTL_REG);
}

/* set wait clock counter */
void aw_spi_set_waitclk_cnt(u32 waitclk_cnt, void *base_addr)
{
    u32 reg_val = readl(base_addr + SPI_WAIT_REG);
    reg_val &= ~SPI_WAIT_CLK_MASK;
    waitclk_cnt &= SPI_WAIT_CLK_MASK;
    reg_val |= waitclk_cnt;
    writel(reg_val, base_addr + SPI_WAIT_REG);
}

/* set sample delay in high speed mode */
void aw_spi_set_sample_delay(u32 on_off, void *base_addr)
{
    u32 reg_val = readl(base_addr+SPI_CTL_REG);
    reg_val &= ~SPI_CTL_MASTER_SDC;
    reg_val |= on_off;
    writel(reg_val, base_addr + SPI_CTL_REG);
}


static int spi_sunxi_get_cfg_csbitmap(int bus_num);

/* flush d-cache */
static void spi_sunxi_cleanflush_dcache_region(void *addr, __u32 len)
{
	__cpuc_flush_dcache_area(addr, len + (1 << 5) * 2 - 2);
}


/* ------------------------------- dma operation start----------------------------- */
#if defined CONFIG_ARCH_SUN4I
static struct sunxi_dma_params spi_dma_params[] = {
	[0] = {
	    .client.name		= "sun4i-spi0",
	},
	[1] = {
	    .client.name		= "sun4i-spi1",
	},
	[2] = {
	    .client.name		= "sun4i-spi2",
	},
    [3] = {
	    .client.name		= "sun4i-spi3",
	}
};
#elif defined CONFIG_ARCH_SUN5I
static struct sunxi_dma_params spi_dma_params[] = {
	[0] = {
	    .client.name		= "sun5i-spi0",
	},
	[1] = {
	    .client.name		= "sun5i-spi1",
	},
	[2] = {
	    .client.name		= "sun5i-spi2",
	}
};
#elif defined CONFIG_ARCH_SUN7I
static struct sunxi_dma_params spi_dma_params[] = {
	[0] = {
	    .client.name		= "sun7i-spi0",
	},
	[1] = {
	    .client.name		= "sun7i-spi1",
	},
	[2] = {
	    .client.name		= "sun7i-spi2",
	},
    [3] = {
	    .client.name		= "sun7i-spi3",
	}
};

#endif

#if defined CONFIG_SUNXI_SPI_NDMA 
/*
 * rx dma callback, disable the 1/4 fifo rx drq.
 * tx dma callback, disable the tx empty drq.
 * it can do this in the isr,too.
 * @ buf: client's id
 */
static void spi_sunxi_dma_cb(struct sunxi_dma_params *dma, void *buf)
{
	struct sunxi_spi *aw_spi = (struct sunxi_spi *)buf;
	unsigned long flags;

    //spi_msg("spi_sunxi_dma_cb\n");
	spin_lock_irqsave(&aw_spi->lock, flags);

	if(aw_spi->dma_dir == SW_DMA_RDEV) {
        //spi_msg("spi dma rx -read data\n");
        aw_spi_disable_dma_irq(SPI_DRQEN_RHF, aw_spi->base_addr);
    }
    else if(aw_spi->dma_dir == SW_DMA_WDEV) {
        //spi_msg("spi dma tx -write data\n");
        aw_spi_disable_dma_irq(SPI_DRQEN_THE, aw_spi->base_addr);
    }
    else {
        spi_wrn("unknow dma direction = %d \n",aw_spi->dma_dir);
    }
    aw_spi_sel_dma_type(0, aw_spi->base_addr);
	spin_unlock_irqrestore(&aw_spi->lock, flags);
}
#else
/*
 * tx dma callback, disable the tx empty drq.
 * it can do this in the isr,too.
 * @ buf: client's id
 */
static void spi_sunxi_dma_tx_cb(struct sunxi_dma_params *dma, void *buf)
{
	struct sunxi_spi *aw_spi = (struct sunxi_spi *)buf;
	unsigned long flags;

    //spi_msg("spi_sunxi_dma_cb\n");
	spin_lock_irqsave(&aw_spi->lock, flags);

    //spi_msg("spi dma tx -write data\n");
    aw_spi_disable_dma_irq(SPI_DRQEN_THE, aw_spi->base_addr);
    aw_spi_sel_dma_type(0, aw_spi->base_addr);
	spin_unlock_irqrestore(&aw_spi->lock, flags);
}


/*
 * rx dma callback, disable the 1/4 fifo rx drq.
 * it can do this in the isr,too.
 * @ buf: client's id
 */
static void spi_sunxi_dma_rx_cb(struct sunxi_dma_params *dma, void *buf)
{
	struct sunxi_spi *aw_spi = (struct sunxi_spi *)buf;
	unsigned long flags;

    //spi_msg("spi_sunxi_dma_cb\n");
	spin_lock_irqsave(&aw_spi->lock, flags);

    //spi_msg("spi dma rx -read data\n");
    aw_spi_disable_dma_irq(SPI_DRQEN_RHF, aw_spi->base_addr);
    aw_spi_sel_dma_type(0, aw_spi->base_addr);
	spin_unlock_irqrestore(&aw_spi->lock, flags);
}
#endif

static int get_busnum(struct sunxi_spi *aw_spi){
	int bus_num;
#ifdef	CONFIG_SPI_SLAVE
	if(aw_spi->master){
#endif
		bus_num=aw_spi->master->bus_num;
#ifdef	CONFIG_SPI_SLAVE
	}else if(aw_spi->slave){
		bus_num=aw_spi->slave->bus_num;
	}else{
		printk("%s: SPI Error: could not determine master/slave.\n",__func__);
	}
#endif
	return bus_num;
}
/*
 * config dma src and dst address,
 * io or linear address,
 * drq type,
 * then enqueue
 * but not trigger dma start
 */
static int spi_sunxi_config_dma(struct sunxi_spi *aw_spi, enum sw_dmadir dma_dir, void *buf, unsigned len)
{
    int ret = 0;
    int bus_num = get_busnum(aw_spi);

#if defined CONFIG_ARCH_SUN4I
    unsigned char spi_drq[] = {DRQ_TYPE_SPI0, DRQ_TYPE_SPI1, DRQ_TYPE_SPI2, DRQ_TYPE_SPI3};
    unsigned long spi_phyaddr[] = {SPI0_BASE_ADDR, SPI1_BASE_ADDR, SPI2_BASE_ADDR, SPI3_BASE_ADDR};/* physical address */
	struct dma_hw_conf spi_hw_conf = {0}; 
#elif defined CONFIG_ARCH_SUN5I
    unsigned char spi_drq[] = {DRQ_TYPE_SPI0, DRQ_TYPE_SPI1, DRQ_TYPE_SPI2};
    unsigned long spi_phyaddr[] = {SPI0_BASE_ADDR, SPI1_BASE_ADDR, SPI2_BASE_ADDR};/* physical address */
	struct dma_hw_conf spi_hw_conf = {0}; 

#elif defined CONFIG_ARCH_SUN7I 
	#if defined CONFIG_SUNXI_SPI_NDMA 
	unsigned char spi_rx_ndrq[] = {N_SRC_SPI0_RX, N_SRC_SPI1_RX, N_SRC_SPI2_RX, N_SRC_SPI3_RX}; 
	unsigned char spi_tx_ndrq[] = {N_DST_SPI0_TX, N_DST_SPI1_TX, N_DST_SPI3_TX, N_DST_SPI3_TX}; 
    #else
	unsigned char spi_rx_ddrq[] = {D_SRC_SPI0_RX, D_SRC_SPI1_RX, D_SRC_SPI2_RX, D_SRC_SPI3_RX}; 
	unsigned char spi_tx_ddrq[] = {D_DST_SPI0_TX, D_DST_SPI1_TX, D_DST_SPI3_TX, D_DST_SPI3_TX}; 
    #endif

	 //NOT USED yet ? unsigned long spi_phyaddr[] = {SPI0_BASE_ADDR, SPI1_BASE_ADDR, SPI2_BASE_ADDR, SPI3_BASE_ADDR};/* physical address */ 
	 dma_config_t spi_hw_conf; 
	 //FIXME: is this needed ? memset(&spi_hw_conf, 0, sizeof(spi_hw_conf)); 
#endif 
	  
	 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
		  #if defined CONFIG_SUNXI_SPI_NDMA 
			 //write 
			 if (dma_dir == SW_DMA_WDEV) { 
					 spi_hw_conf.drqsrc_type  = N_DRQSRC_SDRAM; 			  // must be sdram,or sdram 
					 spi_hw_conf.drqdst_type  = spi_drq[bus_num];			  // spi drq type 
					 spi_hw_conf.dir		  = SW_DMA_WDEV;				  //transmit data to device 
					 spi_hw_conf.address_type = DMAADDRT_D_IO_S_LN; 		  //dest io, src linear 
				 spi_hw_conf.to 		  = spi_phyaddr[bus_num] + SPI_TXDATA_REG; // physical address, destination address 
				 }//read 
				 else if (dma_dir == SW_DMA_RDEV) { 
					 spi_hw_conf.drqsrc_type  = spi_drq[bus_num];	 // spi drq type 
					 spi_hw_conf.drqdst_type  = N_DRQDST_SDRAM; 	 // must be sdram ?? what about sram ? 
					 spi_hw_conf.dir		  = SW_DMA_RDEV;		 // receive data from device 
					 spi_hw_conf.address_type = DMAADDRT_D_LN_S_IO; // dest linear, src io 
					 spi_hw_conf.from		  = spi_phyaddr[bus_num] + SPI_RXDATA_REG; // physical address, source address 
				 } 
				 else { 
					 spi_wrn("[spi-%d]: unknow dma direction = %d \n", bus_num, dma_dir); 
					 return -1; 
				 } 
		 #else 
			 //write 
			 if (dma_dir == SW_DMA_WDEV) { 
					 spi_hw_conf.drqsrc_type  = D_DRQSRC_SDRAM; 			  // must be sdram,or sdram 
					 spi_hw_conf.drqdst_type  = spi_drq[bus_num];			  // spi drq type 
					 spi_hw_conf.dir		  = SW_DMA_WDEV;				  //transmit data to device 
					 spi_hw_conf.address_type = DMAADDRT_D_IO_S_LN; 		  //dest io, src linear 
				 spi_hw_conf.to 		  = spi_phyaddr[bus_num] + SPI_TXDATA_REG; // physical address, destination address 
				 }//read 
				 else if (dma_dir == SW_DMA_RDEV) { 
					 spi_hw_conf.drqsrc_type  = spi_drq[bus_num];	 // spi drq type 
					 spi_hw_conf.drqdst_type  = D_DRQDST_SDRAM; 	 // must be sdram ?? what about sram ? 
					 spi_hw_conf.dir		  = SW_DMA_RDEV;		 // receive data from device 
					 spi_hw_conf.address_type = DMAADDRT_D_LN_S_IO; // dest linear, src io 
					 spi_hw_conf.from		  = spi_phyaddr[bus_num] + SPI_RXDATA_REG; // physical address, source address 
				 } 
				 else { 
					 spi_wrn("[spi-%d]: unknow dma direction = %d \n", bus_num, dma_dir); 
					 return -1; 
				 } 
			 spi_hw_conf.cmbk		 = 0x07070707; 
		 #endif 
	  
		 spi_hw_conf.xfer_type	 = DMAXFER_D_SWORD_S_SWORD; 
		 spi_hw_conf.hf_irq 	 = SW_DMA_IRQ_FULL; 
	  
	 #elif defined CONFIG_ARCH_SUN7I 
		 spi_hw_conf.xfer_type.src_data_width	 = DATA_WIDTH_8BIT; 
		 spi_hw_conf.xfer_type.src_bst_len		 = DATA_BRST_1; 
		 spi_hw_conf.xfer_type.dst_data_width	 = DATA_WIDTH_8BIT; 
		 spi_hw_conf.xfer_type.dst_bst_len		 = DATA_BRST_1; 
		 spi_hw_conf.bconti_mode				 = false; 
		 spi_hw_conf.irq_spt					 = CHAN_IRQ_FD; 
 
		 #if defined CONFIG_SUNXI_SPI_NDMA 
			 //write 
			 if (dma_dir == SW_DMA_WDEV) { 
				 spi_hw_conf.address_type.src_addr_mode  = NDMA_ADDR_INCREMENT; 
				 spi_hw_conf.address_type.dst_addr_mode  = NDMA_ADDR_NOCHANGE; 
				 spi_hw_conf.src_drq_type				 = N_SRC_SDRAM; 
				 spi_hw_conf.dst_drq_type				 = spi_tx_ndrq[bus_num]; 
			 }//read 
			 else if (dma_dir == SW_DMA_RDEV) { 
				 spi_hw_conf.address_type.src_addr_mode  = NDMA_ADDR_NOCHANGE; 
				 spi_hw_conf.address_type.dst_addr_mode  = NDMA_ADDR_INCREMENT; 
				 spi_hw_conf.src_drq_type				 = spi_rx_ndrq[bus_num]; 
				 spi_hw_conf.dst_drq_type				 = N_DST_SDRAM; 
			 } 
			 else { 
				 spi_wrn("[spi-%d]: unknow dma direction = %d \n", bus_num, dma_dir); 
				 return -1; 
			 } 
		 #else 
			 //write 
			 if (dma_dir == SW_DMA_WDEV) { 
				 spi_hw_conf.address_type.src_addr_mode  = DDMA_ADDR_LINEAR; 
				 spi_hw_conf.address_type.dst_addr_mode  = DDMA_ADDR_IO; 
				 spi_hw_conf.src_drq_type				 = D_SRC_SDRAM; 
				 spi_hw_conf.dst_drq_type				 = spi_tx_ddrq[bus_num]; 
			 }//read 
			 else if (dma_dir == SW_DMA_RDEV) { 
				 spi_hw_conf.address_type.src_addr_mode  = DDMA_ADDR_IO; 
				 spi_hw_conf.address_type.dst_addr_mode  = DDMA_ADDR_LINEAR; 
				 spi_hw_conf.src_drq_type				 = spi_rx_ddrq[bus_num]; 
				 spi_hw_conf.dst_drq_type				 = D_DST_SDRAM; 
			 } 
			 else { 
				 spi_wrn("[spi-%d]: unknow dma direction = %d \n", bus_num, dma_dir); 
				 return -1; 
			 } 
		 #endif 
	 #endif 


	 //FIXME: I don't get how spi_dma_patams get the aw_spi infos... missing something, also missing how to distinguish 
	 #if defined CONFIG_SUNXI_SPI_NDMA 
		 /* set src,dst, drq type,configuration */ 
		 //was: ret = sw_dma_config(aw_spi->dma_id, &spi_hw_conf); 
		 ret = sunxi_dma_config(&spi_dma_params[bus_num], &spi_hw_conf, 0); 
		 //was: ret += sw_dma_ctrl(aw_spi->dma_id, SW_DMAOP_FLUSH); 
		 ret += sunxi_dma_flush(&spi_dma_params[bus_num]); 
	#else
		 /* set src,dst, drq type,configuration */ 
		 //was: ret = sw_dma_config((dma_dir == SW_DMA_WDEV) ? aw_spi->dma_tx_id : aw_spi->dma_rx_id, &spi_hw_conf); 
		 ret = sunxi_dma_config(&spi_dma_params[bus_num], &spi_hw_conf, 0); 
		 //was: ret += sw_dma_ctrl((dma_dir == SW_DMA_WDEV) ? aw_spi->dma_tx_id : aw_spi->dma_rx_id, SW_DMAOP_FLUSH); 
		 ret += sunxi_dma_flush(&spi_dma_params[bus_num]); 
	#endif

    /* 1. flush d-cache */
    spi_sunxi_cleanflush_dcache_region((void *)buf, len);
	  #if defined CONFIG_SUNXI_SPI_NDMA 
		 /* 2. enqueue dma transfer, --FIXME--: buf: virtual address, not physical address. */ 
		 //was: ret += sw_dma_enqueue(aw_spi->dma_id, (void *)aw_spi, (dma_addr_t)buf, len); 
		 ret += sunxi_dma_enqueue(&spi_dma_params[bus_num], (dma_addr_t)buf, len, 0); 
	#else
		 /* 2. enqueue dma transfer, --FIXME--: buf: virtual address, not physical address. */ 
		 //was: ret += sw_dma_enqueue((dma_dir == SW_DMA_WDEV) ? aw_spi->dma_tx_id : aw_spi->dma_rx_id, (void *)aw_spi, (dma_addr_t)buf, len); 
		 ret += sunxi_dma_enqueue(&spi_dma_params[bus_num], (dma_addr_t)buf, len, 0); 
	#endif
    return ret;
}

/* set dma start flag, if queue, it will auto restart to transfer next queue */
static int spi_sunxi_start_dma(struct sunxi_spi *aw_spi, unsigned int dma_id)
{
    int ret = 0;
    int bus_num = get_busnum(aw_spi);

    /* change the state of the dma channel, dma start */
	//was: ret = sw_dma_ctrl(dma_id, SW_DMAOP_START); 
	//TODO:start here next time 
	ret = sunxi_dma_start(&spi_dma_params[bus_num]); 
    /* set the channel's flags to a given state */
	#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
		ret += sw_dma_setflags(dma_id, SW_DMAF_AUTOSTART); 
	#endif 
    return ret;
}

/* request dma channel and set callback function */
static int spi_sunxi_prepare_dma(struct sunxi_spi *aw_spi, enum sw_dmadir dma_dir)
{
    int ret = 0;
    int bus_num   = get_busnum(aw_spi);
	 #if defined CONFIG_SUNXI_SPI_NDMA 
		 //was: aw_spi->dma_hdle = sw_dma_request(aw_spi->dma_id, &spi_dma_params[bus_num], NULL); 
		 ret = sunxi_dma_request(&spi_dma_params[bus_num], 0); 
		 if(ret < 0) { 
			 spi_wrn("[spi-%d]: request dma failed!\n", bus_num); 
			 return ret; 
		 } 
			 ret = sunxi_dma_set_callback(aw_spi->dma_hdle, spi_sunxi_dma_cb, aw_spi); 
		 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
		 /* make sure the queue safe */ 
			 ret += sw_dma_setflags(aw_spi->dma_id, 0); // set flag ??? dma run status, mainly usd 
		 #endif 
	#else
			 if (dma_dir == SW_DMA_WDEV) { 
				 ret = sunxi_dma_request(&spi_dma_params[bus_num], 0); 
					 if(ret < 0) { 
						 spi_wrn("[spi-%d]: request dma tx failed!\n", bus_num); 
						 return ret; 
					 } 
					 ret = sunxi_dma_set_callback(aw_spi->dma_tx_hdle, spi_sunxi_dma_tx_cb, aw_spi); 
				 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
			 /* make sure the queue safe */ 
					 ret += sw_dma_setflags(aw_spi->dma_tx_id, 0); // set flag ??? dma run status, mainly usd 
			 #endif 
		 } 
			 else if (dma_dir == SW_DMA_RDEV) { 
				 ret = sunxi_dma_request(&spi_dma_params[bus_num], 0); 
					 if(ret < 0) { 
						 spi_wrn("[spi-%d]: request dma rx failed!\n", bus_num); 
						 return ret; 
					 } 
					 ret = sunxi_dma_set_callback(aw_spi->dma_rx_hdle, spi_sunxi_dma_rx_cb, aw_spi); 
				 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
			 /* make sure the queue safe */ 
					 ret += sw_dma_setflags(aw_spi->dma_rx_id, 0); // set flag ??? dma run status, mainly usd 
			 #endif 
			 } 
			 else { 
					 spi_wrn("[spi-%d]: unknow dma direction = %d\n", bus_num, dma_dir); 
				 return -1; 
			 } 

	#endif
    //spi_msg("[spi-%d] request dma handle = %d, dmaid %d\n", bus_num, aw_spi->dma_hdle, aw_spi->dma_id);
    return ret;
}

/* release dma channel, and set queue status to idle. */
static int spi_sunxi_release_dma(struct sunxi_spi *aw_spi)
{
	int ret = 0;
	 int bus_num = get_busnum(aw_spi); 
	 #if defined CONFIG_SUNXI_SPI_NDMA 
	 ret  = sunxi_dma_stop(&spi_dma_params[bus_num]); /* first stop */ 
	 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
    ret  = sw_dma_ctrl(aw_spi->dma_id, SW_DMAOP_STOP); /* first stop */
    ret += sw_dma_setflags(aw_spi->dma_id, 0);
	 #endif 
	 //FIXME: test error on dma_release and add to ret 
	 sunxi_dma_release(&spi_dma_params[bus_num]); 
    //spi_msg("[spi-%d] release dma,ret = %d \n", aw_spi->master->bus_num, ret);
	 //FIXME: -1 bellow is wrong, we need a NULL on hw_parama ? 
	 //aw_spi->dma_hdle = -1; 
	#else
	 ret  = sunxi_dma_stop(&spi_dma_params[bus_num]); /* first stop */ 
	 //ret += sw_dma_ctrl(aw_spi->dma_rx_id, SW_DMAOP_STOP); /* first stop */ 
	 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
    ret += sw_dma_setflags(aw_spi->dma_tx_id, 0);
    ret += sw_dma_setflags(aw_spi->dma_rx_id, 0);
	 #endif 
	 //FIXME: test error on dma_release and add to ret 
	 sunxi_dma_release(&spi_dma_params[bus_num]); 
	 //ret += sw_dma_free(aw_spi->dma_rx_id, &spi_dma_params[aw_spi->master->bus_num]); 
    //spi_msg("[spi-%d] release dma,ret = %d \n", aw_spi->master->bus_num, ret);
	 //FIXME: -1 bellow is wrong, we need a NULL on hw_parama ? 
	 //aw_spi->dma_tx_hdle = -1; 
	 //aw_spi->dma_rx_hdle = -1; 
	#endif
    return ret;
}
/* ------------------------------dma operation end----------------------------- */


/* check the valid of cs id */
static int spi_sunxi_check_cs(int cs_id, struct sunxi_spi *aw_spi)
{
    int ret = AW_SPI_FAIL;
    switch(cs_id)
    {
        case 0:
            ret = (aw_spi->cs_bitmap & SPI_CHIP_SELECT_CS0) ? AW_SPI_OK : AW_SPI_FAIL;
            break;
        case 1:
            ret = (aw_spi->cs_bitmap & SPI_CHIP_SELECT_CS1) ? AW_SPI_OK : AW_SPI_FAIL;
            break;
        default:
            spi_wrn("chip select not support! cs = %d \n", cs_id);
            break;
    }
    return ret;
}

/* spi device on or off control */
static void spi_sunxi_cs_control(struct spi_device *spi, bool on)
{
	struct sunxi_spi *aw_spi;
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master){
#endif
		aw_spi = spi_master_get_devdata(spi->master);
#ifdef	CONFIG_SPI_SLAVE
	}else if(spi->slave){
		aw_spi = spi_slave_get_devdata(spi->slave);
	}
#endif
	unsigned int cs = 0;
	if (aw_spi->cs_control) {
		if(on) {
			/* set active */
			cs = (spi->mode & SPI_CS_HIGH) ? 1 : 0;
		}
		else {
			/* set inactive */
			cs = (spi->mode & SPI_CS_HIGH) ? 0 : 1;
		}
		aw_spi_ss_level(aw_spi->base_addr, cs);
	}
}

/*
 * change the properties of spi device with spi transfer.
 * every spi transfer must call this interface to update the master to the excute transfer
 * set clock frequecy, bits per word, mode etc...
 * return:  >= 0 : succeed;    < 0: failed.
 */
static int spi_sunxi_xfer_setup(struct spi_device *spi, struct spi_transfer *t)
{
	int bus_num;
	int num_chipselect;
	/* get at the setup function, the properties of spi device */
	struct sunxi_spi *aw_spi;
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master){
#endif
		aw_spi = spi_master_get_devdata(spi->master);
		bus_num = spi->master->bus_num;
		num_chipselect = spi->master->num_chipselect;
#ifdef	CONFIG_SPI_SLAVE
	}else if(spi->slave!=NULL){
		aw_spi = spi_slave_get_devdata(spi->slave);
		bus_num = spi->slave->bus_num;
		num_chipselect = spi->slave->num_chipselect;
	}
#endif
	struct sunxi_spi_config *config = spi->controller_data; //allocate in the setup,and free in the cleanup
    void *__iomem base_addr = aw_spi->base_addr;
    //spi_msg("spi_sunxi_xfer_setup\n");
	config->max_speed_hz  = (t && t->speed_hz) ? t->speed_hz : spi->max_speed_hz;
	config->bits_per_word = (t && t->bits_per_word) ? t->bits_per_word : spi->bits_per_word;
	config->bits_per_word = ((config->bits_per_word + 7) / 8) * 8;

	if(config->bits_per_word != 8) {
	    spi_wrn("[spi-%d]: just support 8bits per word... \n", bus_num);
	    return -EINVAL;
	}
	if(spi->chip_select >= num_chipselect) {
	    spi_wrn("[spi-%d]: spi device's chip select = %d exceeds the master supported cs_num[%d] \n",
	                    bus_num, spi->chip_select, num_chipselect);
	    return -EINVAL;
	}
	/* check again board info */
	if( AW_SPI_OK != spi_sunxi_check_cs(spi->chip_select, aw_spi) ) {
	    spi_wrn("spi_sunxi_check_cs failed! spi_device cs =%d ...\n", num_chipselect);
	    return -EINVAL;
	}
	/* set cs */
	aw_spi_set_cs(spi->chip_select, base_addr);
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master!=NULL){
#endif
    	/*
     	*  master: set spi module clock;
     	*  set the default frequency	10MHz
     	*/
		aw_spi_set_master(base_addr);
		if(config->max_speed_hz > SPI_MAX_FREQUENCY) {
			return -EINVAL;
		}
		aw_spi_set_clk(config->max_speed_hz, clk_get_rate(aw_spi->mclk), base_addr);
		/*
		 *  master : set POL,PHA,SSOPL,LMTF,DDB,DHB; default: SSCTL=0,SMC=1,TBW=0.
		 *  set bit width-default: 8 bits
		 */
		aw_spi_config(1, spi->mode, base_addr);
#ifdef	CONFIG_SPI_SLAVE
	}else{
		aw_spi_set_slave(base_addr);
		aw_spi_config(0, spi->mode, base_addr);
	}
#endif
	return 0;
}


/*
 * > 64 : cpu ;  =< 64 : dma
 * wait for done completion in this function, wakup in the irq hanlder
 */
static int spi_sunxi_xfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct sunxi_spi *aw_spi;
	int bus_num;
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master){
#endif
		aw_spi = spi_master_get_devdata(spi->master);
		bus_num = aw_spi->master->bus_num;
#ifdef	CONFIG_SPI_SLAVE
	}else if(spi->slave){
		aw_spi = spi_slave_get_devdata(spi->slave);
		bus_num = aw_spi->slave->bus_num;
	}else
		printk("%s: SPI Error: could not determine master/slave.\n",__func__);
#endif
	void __iomem* base_addr = aw_spi->base_addr;
	unsigned long flags = 0;
	unsigned tx_len = 0;	/* number of bytes receieved */
	unsigned rx_len = 0;	/* number of bytes sent */
	unsigned char *rx_buf = (unsigned char *)t->rx_buf;
	unsigned char *tx_buf = (unsigned char *)t->tx_buf;
	int ret = 0;

	spi_msg("%d Begin transfer, txbuf %p, rxbuf %p, len %d\n", bus_num, t->tx_buf, t->rx_buf, t->len);
	if (!t->tx_buf && !t->rx_buf && t->len)
		return -EINVAL;
	if (t->tx_buf)
		tx_len = t->len;
	if (t->rx_buf)
		rx_len = t->len;
    //hex_dump("spi_regs:", base_addr, 0x60, 2);

    /* write 1 to clear 0 */
    aw_spi_clr_irq_pending(SPI_STAT_MASK, base_addr);
    /* disable all DRQ */
    aw_spi_disable_dma_irq(SPI_DRQEN_MASK, base_addr);
    aw_spi_sel_dma_type(0, base_addr);
    /* reset tx/rx fifo */
    aw_spi_reset_fifo(base_addr);

    aw_spi_set_bc_wtc(t->len, tx_len, base_addr);

    /*
     * 1. Tx/Rx error irq,process in IRQ;
     * 2. Transfer Complete Interrupt Enable
     */
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master)
#endif
    aw_spi_enable_irq(SPI_INTEN_TC|SPI_INTEN_ERR, base_addr);
#ifdef	CONFIG_SPI_SLAVE
	else
		aw_spi_enable_irq(SPI_INTEN_SSI|SPI_INTEN_TC|SPI_INTEN_ERR, base_addr);
#endif
	if (t->len > BULK_DATA_BOUNDARY) { /* dma */
        #if defined CONFIG_SUNXI_SPI_NDMA 
		if (tx_len && rx_len) /* dma full duplex not possible in normal dma mode */
		{
		    aw_spi_disable_irq(SPI_INTEN_TC|SPI_INTEN_ERR, base_addr);
		    spi_wrn("Full duplex not supported for normal dma!\n");
			return EINVAL;
		}
        aw_spi_sel_dma_type(0, base_addr);
        #else
        aw_spi_sel_dma_type(1, base_addr);
        #endif

		if(tx_len) {
			/* txFIFO 1/4 empty dma request enable,when 16 or less than 16 bytes. */
            aw_spi_enable_dma_irq(SPI_DRQEN_THE, base_addr);
		}

		if(rx_len) {
    		/* rxFIFO 1/4 full dma request enable,when 16 or more than 16 bytes */
            aw_spi_enable_dma_irq(SPI_DRQEN_RHF, base_addr);
		}

		if (rx_len) {
			ret = spi_sunxi_prepare_dma(aw_spi, SW_DMA_RDEV);
    		if(ret < 0) {
    			aw_spi_disable_irq(SPI_INTEN_TC|SPI_INTEN_ERR, base_addr);
				aw_spi_disable_dma_irq(SPI_DRQEN_RHF, base_addr);
				if (tx_len)
					aw_spi_disable_dma_irq(SPI_DRQEN_THE, base_addr);
				spi_wrn("");
    			return -EINVAL;
    		}
			spi_sunxi_config_dma(aw_spi, SW_DMA_RDEV, (void *)rx_buf, rx_len);
	        #if defined CONFIG_SUNXI_SPI_NDMA
			spi_sunxi_start_dma(aw_spi, aw_spi->dma_id);
			#else
			spi_sunxi_start_dma(aw_spi, aw_spi->dma_rx_id);
			#endif
		}

		if (tx_len) {
			ret = spi_sunxi_prepare_dma(aw_spi, SW_DMA_WDEV);
			if(ret < 0) {
			    aw_spi_disable_irq(SPI_INTEN_TC|SPI_INTEN_ERR, base_addr);
			    aw_spi_disable_dma_irq(SPI_DRQEN_THE, base_addr);
				if (rx_len)
					aw_spi_disable_dma_irq(SPI_DRQEN_RHF, base_addr);
				spi_wrn("");
				return -EINVAL;
			}
			spi_sunxi_config_dma(aw_spi, SW_DMA_WDEV, (void *)tx_buf, tx_len);
	        #if defined CONFIG_SUNXI_SPI_NDMA
			spi_sunxi_start_dma(aw_spi, aw_spi->dma_id);
			#else
			spi_sunxi_start_dma(aw_spi, aw_spi->dma_tx_id);
			#endif
		}

	    aw_spi_start_xfer(base_addr);
	} else { /* no dma */
		unsigned int poll_time = 0xfffff;
		//spi_msg(" tx -> by ahb\n");
		spin_lock_irqsave(&aw_spi->lock, flags);
		for(; tx_len > 0; --tx_len) {
		    writeb(*tx_buf++, base_addr + SPI_TXDATA_REG);
		}
		spin_unlock_irqrestore(&aw_spi->lock, flags);
	    aw_spi_start_xfer(base_addr);
		if (rx_len) {
			while(rx_len && (--poll_time >0)) {
				/* rxFIFO counter */
			    if(aw_spi_query_rxfifo(base_addr)){
			        *rx_buf++ =  readb(base_addr + SPI_RXDATA_REG);//fetch data
			        --rx_len;
			        poll_time = 0xffff;
			    }
			}
		} else {
            while(aw_spi_query_txfifo(base_addr)&&(--poll_time > 0) );/* txFIFO counter */
		}
		if(poll_time <= 0) {
		    spi_wrn("cpu tx data time out!\n");
		}
	}

	/* wait for xfer complete in the isr. */
	wait_for_completion(&aw_spi->done);
    /* get the isr return code */
    if(aw_spi->result != 0) {
        spi_wrn("[spi-%d]: xfer failed... \n", bus_num);
        ret = -1;
    }
    /* release dma resource */
	if (t->len > BULK_DATA_BOUNDARY)
		spi_sunxi_release_dma(aw_spi);
	return ret;
}

/* spi core xfer process */
static void spi_sunxi_work(struct work_struct *work)
{
	struct sunxi_spi *aw_spi = container_of(work, struct sunxi_spi, work);
	
	printk("%s %d\n", __FUNCTION__, __LINE__);
	spin_lock_irq(&aw_spi->lock);
	aw_spi->busy = SPI_BUSY;
	/*
     * get from messages queue, and then do with them,
	 * if message queue is empty ,then return and set status to free,
	 * otherwise process them.
	 */
	while (!list_empty(&aw_spi->queue)) {
		struct spi_message *msg = NULL;
		struct spi_device  *spi = NULL;
		struct spi_transfer *t  = NULL;
		unsigned int cs_change = 0;
#ifdef CONFIG_HARDWARE_LIB_SPI
		unsigned int cs_laststop = 0;
#endif		
		int status;
		/* get message from message queue in sunxi_spi. */
		msg = container_of(aw_spi->queue.next, struct spi_message, queue);
		/* then delete from the message queue,now it is alone.*/
		list_del_init(&msg->queue);
		spin_unlock_irq(&aw_spi->lock);
		/* get spi device from this message */
		spi = msg->spi;
		/* set default value,no need to change cs,keep select until spi transfer require to change cs. */
		cs_change = 1;
		/* set to force xfer_setup on first transfer. */
		status = -1;
		/* search the spi transfer in this message, deal with it alone. */
		list_for_each_entry (t, &msg->transfers, transfer_list) {
			if ((status == -1) || t->bits_per_word || t->speed_hz ) { /* xfer_setup if first transfer or overrides provided. */
				status = spi_sunxi_xfer_setup(spi, t);/* set the value every spi transfer */
				spi_msg(" xfer setup \n");
				if (status < 0)
					break;/* fail, quit */
			}
			/* first active the cs */
			if (cs_change) {
				aw_spi->cs_control(spi, 1);
			}
			/* update the new cs value */
			cs_change = t->cs_change;
#ifdef CONFIG_HARDWARE_LIB_SPI
			if (t->delay_usecs == 0xFFFF) 
				aw_spi_set_sample_delay(SPI_CTL_MASTER_SDC, aw_spi->base_addr);
			else
				aw_spi_set_sample_delay(0, aw_spi->base_addr);

			cs_laststop = (t->delay_usecs == 0xAA55);
			/* full duplex mode */
			if(t->rx_buf && t->tx_buf)
				aw_spi_clear_dhb(aw_spi->base_addr);            
#endif			
			/*
             * do transfer
			 * > 64 : cpu ;  =< 64 : dma
			 * wait for done completion in this function, wakup in the irq hanlder
			 */
			status = spi_sunxi_xfer(spi, t);
			if (status)
				break;/* fail quit, zero means succeed */
			/* accmulate the value in the message */
			msg->actual_length += t->len;
			/* may be need to delay */
#ifdef CONFIG_HARDWARE_LIB_SPI
			if ((t->delay_usecs) && (t->delay_usecs!=0xAA55) && (t->delay_usecs != 0xFFFF))
#else
			if (t->delay_usecs)
#endif
				udelay(t->delay_usecs);
			/* if zero ,keep active,otherwise deactived. */
			if (cs_change) {
				aw_spi->cs_control(spi, 0);
			}
		}
		/*
		 * spi message complete,succeed or failed
		 * return value
		 */
		msg->status = status;
		/* wakup the uplayer caller,complete one message */
		msg->complete(msg->context);
		/* fail or need to change cs */
		if (status || !cs_change) {
#ifdef CONFIG_HARDWARE_LIB_SPI
			if (cs_laststop)
#endif			
			aw_spi->cs_control(spi, 0);
		}
		spin_lock_irq(&aw_spi->lock);
	}
	/* set spi to free */
	aw_spi->busy = SPI_FREE;
	spin_unlock_irq(&aw_spi->lock);
	
	printk("%s %d\n", __FUNCTION__, __LINE__);
	return;
}

/* wake up the sleep thread, and give the result code */
static irqreturn_t spi_sunxi_isr(int irq, void *dev_id)
{
	struct sunxi_spi *aw_spi = (struct sunxi_spi *)dev_id;
	void *base_addr = aw_spi->base_addr;
    unsigned int status = aw_spi_qry_irq_pending(base_addr);
    aw_spi_clr_irq_pending(status, base_addr);//write 1 to clear 0.
    //spi_msg("status = %x \n", status);

    aw_spi->result = 0; /* assume succeed */
    /* master mode, Transfer Complete Interrupt */
    if( status & SPI_STAT_TC ) {
        aw_spi_disable_irq(SPI_STAT_TC|SPI_STAT_ERR, base_addr);
        //spi_msg("SPI TC comes\n");
        /*wakup uplayer, by the sem */
        complete(&aw_spi->done);
        return IRQ_HANDLED;
    }/* master mode:err */
    else if( status & SPI_STAT_ERR ) {
        /* error process, release dma in the workqueue,should not be here */
        aw_spi_disable_irq(SPI_STAT_TC|SPI_STAT_ERR, base_addr);
        aw_spi_restore_state(1, base_addr);
        aw_spi->result = -1;
        complete(&aw_spi->done);
        //spi_msg("SPI ERR comes\n");
        spi_wrn("master mode error: txFIFO overflow/rxFIFO underrun or overflow\n");
        return IRQ_HANDLED;
    }
    //spi_msg("SPI NONE comes\n");
	return IRQ_NONE;
}

/* interface 1 */
static int spi_sunxi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct sunxi_spi *aw_spi;
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master)
#endif
		aw_spi = spi_master_get_devdata(spi->master);
#ifdef	CONFIG_SPI_SLAVE
	else if(spi->slave!=NULL){
		aw_spi = spi_slave_get_devdata(spi->slave);
	}else{
		printk("%s:  SPI Error: could not determine master/slave.\n",__func__);
	}
#endif
	unsigned long flags;
	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	printk("%s %d\n", __FUNCTION__, __LINE__);
	spin_lock_irqsave(&aw_spi->lock, flags);
    /* add msg to the sunxi_spi queue */
	list_add_tail(&msg->queue, &aw_spi->queue);
    /* add work to the workqueue,schedule the cpu. */
	queue_work(aw_spi->workqueue, &aw_spi->work);
	spin_unlock_irqrestore(&aw_spi->lock, flags);
	printk("%s %d\n", __FUNCTION__, __LINE__);

    /* return immediately and wait for completion in the uplayer caller. */
	return 0;
}

/* interface 2, setup the frequency and default status */
static int spi_sunxi_setup(struct spi_device *spi)
{
	struct sunxi_spi *aw_spi;
	int bus_num;
#ifdef	CONFIG_SPI_SLAVE
	if(spi->master){
#endif
		aw_spi = spi_master_get_devdata(spi->master);
#ifdef	CONFIG_SPI_SLAVE
	}else if(spi->slave!=NULL){
		aw_spi = spi_slave_get_devdata(spi->slave);
	}else{
		printk("%s:  SPI Error: could not determine master/slave.\n",__func__);
	}
#endif
	struct sunxi_spi_config *config = spi->controller_data;/* general is null. */
	unsigned long flags;
	printk("%s %d\n", __FUNCTION__, __LINE__);

    /* just support 8 bits per word */
	if (spi->bits_per_word != 8)
		return -EINVAL;
    /* first check its valid,then set it as default select,finally set its */
    if(AW_SPI_FAIL == spi_sunxi_check_cs(spi->chip_select, aw_spi)) {
#ifdef	CONFIG_SPI_SLAVE
		if(aw_spi->master){
#endif
			bus_num=aw_spi->master->bus_num;
#ifdef	CONFIG_SPI_SLAVE
		}else if(aw_spi->slave){
			bus_num=aw_spi->slave->bus_num;
		}else{
			printk("%s: SPI Error: could not determine master/slave.\n",__func__);
		}
#endif
        spi_wrn("[spi-%d]: not support cs-%d \n", bus_num, spi->chip_select);

        return -EINVAL;
    }
   	if(spi->max_speed_hz > SPI_MAX_FREQUENCY) {
	    return -EINVAL;
	}
	if (!config) {
		config = kzalloc(sizeof *config, GFP_KERNEL);
		if (!config)
			return -ENOMEM;
		spi->controller_data = config;
	}
    /*
     * set the default vaule with spi device
     * can change by every spi transfer
     */
	config->bits_per_word = spi->bits_per_word;
	config->max_speed_hz  = spi->max_speed_hz;
	config->mode		  = spi->mode;

	spin_lock_irqsave(&aw_spi->lock, flags);
	/* if aw16xx spi is free, then deactived the spi device */
	if (aw_spi->busy & SPI_FREE) {
		/* set chip select number */
	    aw_spi_set_cs(spi->chip_select, aw_spi->base_addr);
	    /* deactivate chip select */
		aw_spi->cs_control(spi, 0);
	}
	spin_unlock_irqrestore(&aw_spi->lock, flags);
	
	printk("%s %d\n", __FUNCTION__, __LINE__);
	return 0;
}

/* interface 3 */
static void spi_sunxi_cleanup(struct spi_device *spi)
{
    if(spi->controller_data) {
        kfree(spi->controller_data);
        spi->controller_data = NULL;
    }
}

static int spi_sunxi_set_gpio(struct sunxi_spi *aw_spi, bool on)
{
	int bus_num = get_busnum(aw_spi);
    if(on) {
        if( bus_num == 0 ) {
            aw_spi->gpio_hdle = gpio_request_ex("spi0_para", NULL);
            if(!aw_spi->gpio_hdle) {
                spi_wrn("spi0 request gpio fail!\n");
                return -1;
            }
            //hex_dump("gpio regs:", (void __iomem*)SW_VA_PORTC_IO_BASE, 0x200, 2);
        }
        else if( bus_num == 1 ) {
			/**
			 * PI8		SPI1_CS0
			 * PI9		SPI1_CS1
			 * PI10    	SPI1_CLK
			 * PI11    	SPI1_MOSI
			 * PI12	    SPI1_MISO
			 */
            #ifndef SYS_SPI_PIN
		    unsigned int  reg_val = readl(_Pn_CFG1(8));
			/* set spi function */
		    reg_val &= ~0x77777;
		    reg_val |=  0x22222;
		    writel(reg_val, _Pn_CFG1(8));
            /* set pull up */
    		reg_val = readl(_Pn_PUL1(8));
    		reg_val &= ~(0x3ff<<16);
    		reg_val |=  (0x155<<16);
    		writel(reg_val, _Pn_PUL1(8));
    		/* no need to set driver,default is driver 1. */
    		#else
            aw_spi->gpio_hdle = gpio_request_ex("spi1_para", NULL);
            if(!aw_spi->gpio_hdle) {
                spi_wrn("spi1 request gpio fail!\n");
                return -1;
            }
    		#endif
        }
        else if(bus_num == 2) {
            aw_spi->gpio_hdle = gpio_request_ex("spi2_para", NULL);
            if(!aw_spi->gpio_hdle) {
                spi_wrn("spi2 request gpio fail!\n");
                return -1;
            }
        }

        #ifdef AW1623_FPGA
        {
            #include <mach/platform.h>
            void __iomem* pi_cfg0 = (void __iomem*)(SW_VA_PORTC_IO_BASE+0x48);
            u32 rval = readl(pi_cfg0) & (~(0x70777));
            writel(rval|(0x30333), pi_cfg0);
        }
        #endif
    }
    else {
        if(bus_num == 0) {
            gpio_release(aw_spi->gpio_hdle, 0);
        }
        else if(bus_num == 1) {
        	#ifndef SYS_SPI_PIN
		    unsigned int  reg_val = readl(_Pn_CFG1(8));
			/* set default */
		    reg_val &= ~0x77777;
		    writel(reg_val, _Pn_CFG1(8));
            #else
		    gpio_release(aw_spi->gpio_hdle, 0);
		    #endif
        }
        else if(bus_num == 2) {
            gpio_release(aw_spi->gpio_hdle, 0);
        }
    }
	return 0;
}

static int spi_sunxi_set_mclk(struct sunxi_spi *aw_spi, u32 mod_clk)
{
    struct clk *source_clock = NULL;
    char* name = NULL;
    u32 source = 1;
    int ret = 0;
    switch (source)
    {
        case 0:
            source_clock = clk_get(NULL, "hosc");
            name = "hosc";
            break;
        case 1:
            source_clock = clk_get(NULL, "sdram_pll_p");
            name = "sdram_pll_p";
            break;
        case 2:
            source_clock = clk_get(NULL, "sata_pll");
            name = "sata_pll";
            break;
        default:
            return -1;
    }

    if (IS_ERR(source_clock))
	{
		ret = PTR_ERR(source_clock);
		spi_wrn("Unable to get spi source clock resource\n");
		return -1;
	}

    if (clk_set_parent(aw_spi->mclk, source_clock))
    {
        spi_wrn("clk_set_parent failed\n");
        ret = -1;
        goto out;
    }

    if (clk_set_rate(aw_spi->mclk, mod_clk))
    {
        spi_wrn("clk_set_rate failed\n");
        ret = -1;
        goto out;
    }

    spi_msg("source = %s, src_clk = %u, mclk %u\n", name, (unsigned)clk_get_rate(source_clock), (unsigned)clk_get_rate(aw_spi->mclk));

	if (clk_enable(aw_spi->mclk)) {
		spi_wrn("Couldn't enable module clock 'spi'\n");
		ret = -EBUSY;
		goto out;
	}
	ret = 0;
out:
    clk_put(source_clock);
    return ret;
}

static int spi_sunxi_hw_init(struct sunxi_spi *aw_spi)
{
	void *base_addr = aw_spi->base_addr;
	unsigned long sclk_freq = 0;
#if defined CONFIG_ARCH_SUN4I
	char* mclk_name[] = {"spi0","spi1","spi2","spi3"};
#elif defined CONFIG_ARCH_SUN5I
    char* mclk_name[] = {"spi0","spi1","spi2"};
#elif defined CONFIG_ARCH_SUN7I 
	char* mclk_name[] = {"spi0","spi1","spi2","spi3"}; 
#endif
    aw_spi->mclk = clk_get(&aw_spi->pdev->dev, mclk_name[aw_spi->pdev->id]);
	if (IS_ERR(aw_spi->mclk)) {
		spi_wrn("Unable to acquire module clock 'spi'\n");
		return -1;
	}
    if (spi_sunxi_set_mclk(aw_spi, 100000000))
    {
        spi_wrn("spi_sunxi_set_mclk 'spi'\n");
		clk_put(aw_spi->mclk);
		return -1;
    }
    //hex_dump("ccmu regs:", (void __iomem*)SW_VA_CCM_IO_BASE, 0x200, 2);

	/* 1. enable the spi module */
	aw_spi_enable_bus(base_addr);
	/* 2. set the default chip select */
	if(AW_SPI_OK == spi_sunxi_check_cs(0, aw_spi)) {
	    aw_spi_set_cs(0, base_addr);
	}
	else{
        aw_spi_set_cs(1, base_addr);
	}
#ifdef	CONFIG_SPI_SLAVE
	if(aw_spi->master) {
#endif
		/* 3. master */
    	aw_spi_set_master(base_addr);
    	sclk_freq  = clk_get_rate(aw_spi->mclk);
#ifdef	CONFIG_SPI_SLAVE
	}else{
		printk("%s: spi slave mode!\n",__func__);
		aw_spi_set_slave(base_addr);
	}
#endif
	/* 4. manual control the chip select */
	aw_spi_ss_ctrl(base_addr, 1);
	return 0;
}

static int spi_sunxi_hw_exit(struct sunxi_spi *aw_spi)
{
	/* disable the spi controller */
    aw_spi_disable_bus(aw_spi->base_addr);
	/* disable module clock */
    clk_disable(aw_spi->mclk);
    clk_put(aw_spi->mclk);
	return 0;
}

static int __devinit spi_sunxi_probe(struct platform_device *pdev)
{
	#if defined CONFIG_SUNXI_SPI_NDMA
	struct resource	*dma_res;
	#else
	struct resource	*dma_tx_res, *dma_rx_res;
	#endif
	struct resource	*mem_res;
	struct sunxi_spi *aw_spi;
	struct sunxi_spi_platform_data *pdata;
	struct spi_master *master;
#ifdef	CONFIG_SPI_SLAVE
	struct spi_slave *slave;
#endif
	int ret = 0, err = 0, irq;
	int cs_bitmap = 0;

	if (pdev->id < 0) {
		spi_wrn("Invalid platform device id-%d\n", pdev->id);
		return -ENODEV;
	}

	if (pdev->dev.platform_data == NULL) {
		spi_wrn("platform_data missing!\n");
		return -ENODEV;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata->clk_name) {
		spi_wrn("platform data must initial! \n");
		return -EINVAL;
	}

	/* Check for availability of necessary resource */
    #if defined CONFIG_SUNXI_SPI_NDMA
    dma_res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
    if (dma_res == NULL) {
        spi_wrn("Unable to get spi DMA resource\n");
        return -ENXIO;
    }
    #else
    dma_tx_res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
    if (dma_tx_res == NULL) {
        spi_wrn("Unable to get spi DMA TX resource\n");
        return -ENXIO;
    }
    dma_rx_res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
    if (dma_rx_res == NULL) {
        spi_wrn("Unable to get spi DMA RX resource\n");
        return -ENXIO;
    }
    #endif

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (mem_res == NULL) {
        	spi_wrn("Unable to get spi MEM resource\n");
        	return -ENXIO;
        }

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		spi_wrn("No spi IRQ specified\n");
		return -ENXIO;
	}
#ifdef	CONFIG_SPI_SLAVE
		if(pdev->id!=SLAVE_BUS){
#endif
    /* create spi master */
		master = spi_alloc_master(&pdev->dev, sizeof(struct sunxi_spi));
		if (master == NULL) {
			spi_wrn("Unable to allocate SPI Master\n");
			return -ENOMEM;
		}

		platform_set_drvdata(pdev, master);
		aw_spi = spi_master_get_devdata(master);
		aw_spi->master = master;
#ifdef	CONFIG_SPI_SLAVE
		aw_spi->slave = NULL;
		}else{
			/* create spi slave */
			printk("%s: spi slave mode!\n",__func__);
			master = NULL;
			slave = spi_alloc_slave(&pdev->dev, sizeof(struct sunxi_spi));
			if (slave == NULL) {
				spi_wrn("Unable to allocate SPI Slave\n");
				return -ENOMEM;
			}

			platform_set_drvdata(pdev, slave);
			aw_spi = spi_slave_get_devdata(slave);
			aw_spi->slave = slave;
			aw_spi->master = NULL;
		}
#endif
    memset(aw_spi, 0, sizeof(struct sunxi_spi));

	aw_spi->master     = master;
	aw_spi->irq        = irq;
    #if defined CONFIG_SUNXI_SPI_NDMA
	aw_spi->dma_id     = dma_res->start;
    aw_spi->dma_hdle = -1;
	#else
	aw_spi->dma_tx_id     = dma_tx_res->start;
	aw_spi->dma_rx_id     = dma_rx_res->start;
    aw_spi->dma_tx_hdle = (dma_hdl_t)-1;
    aw_spi->dma_rx_hdle = (dma_hdl_t)-1;
	#endif
	aw_spi->cs_control = spi_sunxi_cs_control;
	aw_spi->cs_bitmap  = pdata->cs_bitmap; /* cs0-0x1; cs1-0x2; cs0&cs1-0x3. */
	aw_spi->busy       = SPI_FREE;

#ifdef	CONFIG_SPI_SLAVE
	if(pdev->id!=SLAVE_BUS) {
#endif
	master->bus_num         = pdev->id;
	master->setup           = spi_sunxi_setup;
	master->cleanup         = spi_sunxi_cleanup;
	master->transfer        = spi_sunxi_transfer;
	master->num_chipselect  = pdata->num_cs;
    //master->dma_alignment   = 8; //  should be set to 32  ??
    //master->flags           = SPI_MASTER_HALF_DUPLEX; // temporay not support duplex

	/* the spi->mode bits understood by this driver: */
	master->mode_bits       = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH| SPI_LSB_FIRST | SPI_LOOP;
#ifdef	CONFIG_SPI_SLAVE
	}else{
		printk("%s: spi slave mode!\n",__func__);
		slave->bus_num         = pdev->id;
		slave->setup           = spi_sunxi_setup;
		slave->cleanup         = spi_sunxi_cleanup;
		slave->transfer        = spi_sunxi_transfer;
		slave->num_chipselect  = pdata->num_cs;
		//slave->dma_alignment   = 8; //  should be set to 32  ??
		//slave->flags           = SPI_SLAVE_HALF_DUPLEX; // temporay not support duplex

		/* the spi->mode bits understood by this driver: */
		//slave->mode_bits       = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH| SPI_LSB_FIRST | SPI_LOOP;
	}
#endif
    /* update the cs bitmap */
    cs_bitmap = spi_sunxi_get_cfg_csbitmap(pdev->id);
	if(cs_bitmap & 0x3){
	    aw_spi->cs_bitmap  = cs_bitmap&0x3;
    //spi_msg("cs_bitmap = 0x%x \n", cs_bitmap);
	}

	err = request_irq(aw_spi->irq, spi_sunxi_isr, IRQF_DISABLED, pdev->name, aw_spi);
	if (err) {
		spi_wrn("Cannot claim IRQ\n");
		goto err0;
	}

	if (request_mem_region(mem_res->start,
			resource_size(mem_res), pdev->name) == NULL) {
		spi_wrn("Req mem region failed\n");
		ret = -ENXIO;
		goto err1;
	}

	aw_spi->base_addr = ioremap(mem_res->start, resource_size(mem_res));
	if (aw_spi->base_addr == NULL) {
		spi_wrn("Unable to remap IO\n");
		ret = -ENXIO;
		goto err2;
	}

	/* Setup clocks */
	aw_spi->hclk = clk_get(&pdev->dev, pdata->clk_name);
	if (IS_ERR(aw_spi->hclk)) {
		spi_wrn("Unable to acquire clock 'spi'\n");
		ret = PTR_ERR(aw_spi->hclk);
		goto err3;
	}

	if (clk_enable(aw_spi->hclk)) {
		spi_wrn("Couldn't enable clock 'spi'\n");
		ret = -EBUSY;
		goto err4;
	}
#ifdef	CONFIG_SPI_SLAVE
	if(pdev->id!=SLAVE_BUS) {
#endif
		aw_spi->workqueue = create_singlethread_workqueue(dev_name(master->dev.parent));
#ifdef	CONFIG_SPI_SLAVE
	}else{
		aw_spi->workqueue = create_singlethread_workqueue(dev_name(slave->dev.parent));
	}
#endif

	if (aw_spi->workqueue == NULL) {
		spi_wrn("Unable to create workqueue\n");
		ret = -ENOMEM;
		goto err5;
	}

    aw_spi->pdev = pdev;

	/* Setup Deufult Mode */
	spi_sunxi_hw_init(aw_spi);
#ifndef SYS_SPI_PIN
	/* set gpio */
	gpio_addr = ioremap(_PIO_BASE_ADDRESS, 0x1000);
#endif
	spi_sunxi_set_gpio(aw_spi, 1);

	spin_lock_init(&aw_spi->lock);
	init_completion(&aw_spi->done);
	INIT_WORK(&aw_spi->work, spi_sunxi_work);/* banding the process handler */
	INIT_LIST_HEAD(&aw_spi->queue);

#ifdef	CONFIG_SPI_SLAVE
	if(pdev->id!=SLAVE_BUS) {
#endif
		if (spi_register_master(master)) {
			spi_wrn("cannot register SPI master\n");
			ret = -EBUSY;
			goto err6;
		}

		spi_msg("allwinners SoC SPI Driver loaded for Bus SPI-%d with %d Slaves attached\n", pdev->id, master->num_chipselect);
		//spi_msg("\tIOmem=[0x%x-0x%x]\tDMA=[%d]\n", mem_res->end, mem_res->start, aw_spi->dma_id);
		#if defined CONFIG_SUNXI_SPI_NDMA
		spi_msg("[spi-%d]: driver probe succeed, base %p, irq %d, dma_id %d!\n", master->bus_num, aw_spi->base_addr, aw_spi->irq, aw_spi->dma_id);
		#else
		spi_msg("[spi-%d]: driver probe succeed, base %p, irq %d, dma_tx_id %d, dma_rx_id %d!\n", master->bus_num, aw_spi->base_addr, aw_spi->irq, aw_spi->dma_tx_id, aw_spi->dma_rx_id);
		#endif
#ifdef	CONFIG_SPI_SLAVE
	}else{
		printk("%s: spi slave mode! spi_register_slave(slave)\n",__func__);
		if (spi_register_slave(slave)) {
			spi_wrn("cannot register SPI Slave\n");
			ret = -EBUSY;
			goto err6;
		}
		spi_msg("allwinners SoC SPI Driver loaded for Bus SPI-%d as slave\n", pdev->id);
	}
#endif

	return 0;
err6:
	destroy_workqueue(aw_spi->workqueue);
err5:
	clk_disable(aw_spi->hclk);
err4:
	clk_put(aw_spi->hclk);
err3:
	iounmap((void *)aw_spi->base_addr);
err2:
	release_mem_region(mem_res->start, resource_size(mem_res));
err1:
	free_irq(aw_spi->irq, aw_spi);
err0:
	platform_set_drvdata(pdev, NULL);
#ifdef	CONFIG_SPI_SLAVE
	if(aw_spi->master) {
#endif
	spi_master_put(master);
#ifdef	CONFIG_SPI_SLAVE
	}else{
		spi_slave_put(slave);
	}
#endif
	return ret;
}

static int spi_sunxi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
#ifdef	CONFIG_SPI_SLAVE
	struct spi_slave *slave = spi_slave_get(platform_get_drvdata(pdev));
#endif
	struct sunxi_spi *aw_spi = spi_master_get_devdata(master);
	struct resource	*mem_res;
	unsigned long flags;

	spin_lock_irqsave(&aw_spi->lock, flags);
	aw_spi->busy |= SPI_FREE;
	spin_unlock_irqrestore(&aw_spi->lock, flags);

	while (aw_spi->busy & SPI_BUSY)
		msleep(10);

	spi_sunxi_hw_exit(aw_spi);
#ifdef	CONFIG_SPI_SLAVE
	if(master) {
#endif
	spi_unregister_master(master);
#ifdef	CONFIG_SPI_SLAVE
	}else{
		spi_unregister_slave(slave);
	}
#endif
	destroy_workqueue(aw_spi->workqueue);

	clk_disable(aw_spi->hclk);
	clk_put(aw_spi->hclk);

	iounmap((void *) aw_spi->base_addr);
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem_res != NULL)
		release_mem_region(mem_res->start, resource_size(mem_res));

	platform_set_drvdata(pdev, NULL);
#ifdef	CONFIG_SPI_SLAVE
	if(master) {
#endif
		spi_master_put(master);
#ifdef	CONFIG_SPI_SLAVE
	}else{
		spi_slave_put(slave);
	}
#endif
	return 0;
}

#ifdef CONFIG_PM
static int spi_sunxi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
#ifdef	CONFIG_SPI_SLAVE
	struct spi_slave *slave = spi_slave_get(platform_get_drvdata(pdev));
#endif
	struct sunxi_spi *aw_spi;
	unsigned long flags;
#ifdef	CONFIG_SPI_SLAVE
	if(master)
#endif
		aw_spi = spi_master_get_devdata(master);
#ifdef	CONFIG_SPI_SLAVE
	else if(slave)
		aw_spi = spi_slave_get_devdata(slave);
	else
		printk("%s: SPI Error: could not determine master/slave.\n",__func__);
#endif
	printk("[spi-%d]: suspend okay.. \n", master->bus_num);

	spin_lock_irqsave(&aw_spi->lock, flags);
	aw_spi->busy |= SPI_SUSPND;
	spin_unlock_irqrestore(&aw_spi->lock, flags);

	while (aw_spi->busy & SPI_BUSY)
		msleep(10);

	/* Disable the clock */
	clk_disable(aw_spi->hclk);
	return 0;
}

static int spi_sunxi_resume(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
#ifdef	CONFIG_SPI_SLAVE
	struct spi_slave *slave = spi_slave_get(platform_get_drvdata(pdev));
#endif
	struct sunxi_spi  *aw_spi;
	unsigned long flags;
	int bus_num;
#ifdef	CONFIG_SPI_SLAVE
	if(master) {
#endif
		aw_spi = spi_master_get_devdata(master);
		bus_num = master->bus_num;
#ifdef	CONFIG_SPI_SLAVE
	}else if(slave){
		aw_spi = spi_slave_get_devdata(slave);
		bus_num = slave->bus_num;
	}else
		printk("%s: SPI Error: could not determine master/slave.\n",__func__);
#endif
    printk("[spi-%d]: resume okay.. \n", bus_num);

	/* Enable the clock */
	clk_enable(aw_spi->hclk);
	spi_sunxi_hw_init(aw_spi);

	spin_lock_irqsave(&aw_spi->lock, flags);
	aw_spi->busy = SPI_FREE;
	spin_unlock_irqrestore(&aw_spi->lock, flags);
	return 0;
}
#else
#define spi_sunxi_suspend	NULL
#define spi_sunxi_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver __refdata spi_sunxi_driver = {
	.driver = {
#if defined CONFIG_ARCH_SUN4I
		.name	= "sun4i-spi",
#elif defined CONFIG_ARCH_SUN5I
        .name	= "sun5i-spi",
#elif defined CONFIG_ARCH_SUN7I 
		.name	 = "sun7i-spi",
#endif
		.owner = THIS_MODULE,
	},
	.probe   = spi_sunxi_probe,
	.remove  = spi_sunxi_remove,
	.suspend = spi_sunxi_suspend,
	.resume  = spi_sunxi_resume,
};

/* ---------------- spi resouce and platform data start ---------------------- */
struct sunxi_spi_platform_data sunxi_spi0_pdata = {
#if defined CONFIG_ARCH_SUN4I
	.cs_bitmap	= 0x3,
	.num_cs		= 2,
#elif defined CONFIG_ARCH_SUN5I
	.cs_bitmap  = 0x1,
	.num_cs		= 1,
#elif defined CONFIG_ARCH_SUN7I 
	.cs_bitmap  = 0x3, 
	.num_cs	 = 2, 
#endif
	.clk_name = "ahb_spi0",
};
static struct resource sunxi_spi0_resources[] = {
	 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
			 [0] = { 
					 .start 	   = SPI0_BASE_ADDR, 
					 .end		 = SPI0_BASE_ADDR + 1024, 
					 .flags 	   = IORESOURCE_MEM, 
			 }, 
			 [1] = { 
					 .start 	   = SW_INT_IRQNO_SPI00, 
					 .end		 = SW_INT_IRQNO_SPI00, 
					 .flags 	   = IORESOURCE_IRQ, 
			 }, 
			 #if defined CONFIG_SUNXI_SPI_NDMA 
			 [2] = { 
					 .start 	   = DMACH_NSPI0, 
					 .end		 = DMACH_NSPI0, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #else 
			 [2] = { 
					 .start 	   = DMACH_DSPI0_TX, 
					 .end		 = DMACH_DSPI0_TX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 [3] = { 
					 .start 	   = DMACH_DSPI0_RX, 
					 .end		 = DMACH_DSPI0_RX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
         #endif 
	 #else 
		 [0] = { 
		 .start  = SPI0_BASE_ADDR, 
		 .end	 = SPI0_BASE_ADDR + 1024, 
		 .flags  = IORESOURCE_MEM, 
	 }, 
		 #ifdef CONFIG_SUNXI_SPI_NDMA 
			 [1] = { 
				 .start  = N_SRC_SPI0_RX, 
				 .end	 = N_SRC_SPI0_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI0_TX, 
				 .end	 = N_DST_SPI0_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #else 
			 [1] = { 
				 .start  = D_SRC_SPI0_RX, 
				 .end	 = D_SRC_SPI0_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = D_DST_SPI0_TX, 
				 .end	 = D_DST_SPI0_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #endif 
			 [3] = { 
				 .start  = SW_INT_IRQNO_SPI00, 
				 .end	 = SW_INT_IRQNO_SPI00, 
				 .flags  = IORESOURCE_IRQ, 
			 } 
	 #endif 

};
static struct platform_device sunxi_spi0_device = {
#if defined CONFIG_ARCH_SUN4I
	.name		= "sun4i-spi",
#elif defined CONFIG_ARCH_SUN5I
    .name		= "sun5i-spi",
#elif defined CONFIG_ARCH_SUN7I 
	.name       = "sun7i-spi",   
#endif
	.id			= 0,
	.num_resources	= ARRAY_SIZE(sunxi_spi0_resources),
	.resource	= sunxi_spi0_resources,
	.dev		= {
		.platform_data = &sunxi_spi0_pdata,
	},
};

struct sunxi_spi_platform_data sunxi_spi1_pdata = {
#if defined CONFIG_ARCH_SUN4I
	.cs_bitmap	= 0x3,
	.num_cs		= 2,
#elif defined CONFIG_ARCH_SUN5I
	.cs_bitmap	= 0x1,
	.num_cs		= 1,
#elif defined CONFIG_ARCH_SUN7I 
	.cs_bitmap  = 0x3, 
	.num_cs     = 2, 	
#endif
	.clk_name = "ahb_spi1",
};
static struct resource sunxi_spi1_resources[] = {
	 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
			 [0] = { 
					 .start 	   = SPI1_BASE_ADDR, 
					 .end		 = SPI1_BASE_ADDR + 1024, 
					 .flags 	   = IORESOURCE_MEM, 
			 }, 
			 [1] = { 
					 .start 	   = SW_INT_IRQNO_SPI01, 
					 .end		 = SW_INT_IRQNO_SPI01, 
					 .flags 	   = IORESOURCE_IRQ, 
			 }, 
			 #if defined CONFIG_SUNXI_SPI_NDMA 
			 [2] = { 
					 .start 	   = DMACH_NSPI1, 
					 .end		 = DMACH_NSPI1, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #else 
			 [2] = { 
					 .start 	   = DMACH_DSPI1_TX, 
					 .end		 = DMACH_DSPI1_TX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 [3] = { 
					 .start 	   = DMACH_DSPI1_RX, 
					 .end		 = DMACH_DSPI1_RX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #endif 
	 #else 
			 [0] = { 
				 .start  = SPI1_BASE_ADDR, 
				 .end	 = SPI1_BASE_ADDR + 1024, 
				 .flags  = IORESOURCE_MEM, 
			 }, 
		 #ifdef CONFIG_SUNXI_SPI_NDMA 
			 [1] = { 
				 .start  = N_SRC_SPI1_RX, 
				 .end	 = N_SRC_SPI1_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI1_TX, 
				 .end	 = N_DST_SPI1_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #else 
			 [1] = { 
				 .start  = D_SRC_SPI1_RX, 
				 .end	 = D_SRC_SPI1_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI1_TX, 
				 .end	 = N_DST_SPI1_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #endif 
			 [3] = { 
				 .start  = SW_INT_IRQNO_SPI01, 
				 .end	 = SW_INT_IRQNO_SPI01, 
				 .flags  = IORESOURCE_IRQ, 
			 } 
	 #endif

};
static struct platform_device sunxi_spi1_device = {
#if defined CONFIG_ARCH_SUN4I
	.name		= "sun4i-spi",
#elif defined CONFIG_ARCH_SUN5I
    .name		= "sun5i-spi",
#elif defined CONFIG_ARCH_SUN7I 
	.name		 = "sun7i-spi",
#endif
	.id			= 1,
	.num_resources	= ARRAY_SIZE(sunxi_spi1_resources),
	.resource	= sunxi_spi1_resources,
	.dev		= {
		.platform_data = &sunxi_spi1_pdata,
	},
};

struct sunxi_spi_platform_data sunxi_spi2_pdata = {
	.cs_bitmap	= 0x3,
	.num_cs		= 2,
	.clk_name	= "ahb_spi2",
};
static struct resource sunxi_spi2_resources[] = {
	 #if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I 
			 [0] = { 
					 .start 	   = SPI2_BASE_ADDR, 
					 .end		 = SPI2_BASE_ADDR + 1024, 
					 .flags 	   = IORESOURCE_MEM, 
			 }, 
			 [1] = { 
					 .start 	   = SW_INT_IRQNO_SPI02, 
					 .end		 = SW_INT_IRQNO_SPI02, 
					 .flags 	   = IORESOURCE_IRQ, 
			 }, 
			 #if defined CONFIG_SUNXI_SPI_NDMA 
			 [2] = { 
					 .start 	   = DMACH_NSPI2, 
					 .end		 = DMACH_NSPI2, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #else 
			 [2] = { 
					 .start 	   = DMACH_DSPI2_TX, 
					 .end		 = DMACH_DSPI2_TX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 [3] = { 
					 .start 	   = DMACH_DSPI2_RX, 
					 .end		 = DMACH_DSPI2_RX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #endif 
	 #else 
		 [0] = { 
				 .start  = SPI2_BASE_ADDR, 
				 .end	 = SPI2_BASE_ADDR + 1024, 
				 .flags  = IORESOURCE_MEM, 
			 }, 
		 #ifdef CONFIG_SUNXI_SPI_NDMA 
			 [1] = { 
				 .start  = N_SRC_SPI2_RX, 
				 .end	 = N_SRC_SPI2_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI2_TX, 
				 .end	 = N_DST_SPI2_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #else 
			 [1] = { 
				 .start  = D_SRC_SPI2_RX, 
				 .end	 = D_SRC_SPI2_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI2_TX, 
				 .end	 = N_DST_SPI2_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #endif 
			 [3] = { 
				 .start  = SW_INT_IRQNO_SPI02, 
				 .end	 = SW_INT_IRQNO_SPI02, 
				 .flags  = IORESOURCE_IRQ, 
			 } 
	 #endif 

};
static struct platform_device sunxi_spi2_device = {
#if defined CONFIG_ARCH_SUN4I
	.name		= "sun4i-spi",
#elif defined CONFIG_ARCH_SUN5I
    .name		= "sun5i-spi",
#elif defined CONFIG_ARCH_SUN7I
	.name		= "sun7i-spi",
#endif
	.id			= 2,
	.num_resources	= ARRAY_SIZE(sunxi_spi2_resources),
	.resource	= sunxi_spi2_resources,
	.dev		= {
		.platform_data = &sunxi_spi2_pdata,
	},
};

#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN7I
struct sunxi_spi_platform_data sunxi_spi3_pdata = {
	.cs_bitmap	= 0x3,
	.num_cs		= 2,
	.clk_name	= "ahb_spi3",
};
static struct resource sunxi_spi3_resources[] = {
	 #if defined CONFIG_ARCH_SUN4I 
			 [0] = { 
					 .start 	   = SPI3_BASE_ADDR, 
					 .end		 = SPI3_BASE_ADDR + 1024, 
					 .flags 	   = IORESOURCE_MEM, 
			 }, 
			 [1] = { 
					 .start 	   = SW_INT_IRQNO_SPI3, 
					 .end		 = SW_INT_IRQNO_SPI3, 
					 .flags 	   = IORESOURCE_IRQ, 
			 }, 
			 #if defined CONFIG_SUNXI_SPI_NDMA 
			 [2] = { 
					 .start 	   = DMACH_NSPI3, 
					 .end		 = DMACH_NSPI3, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #else 
			 [2] = { 
					 .start 	   = DMACH_DSPI3_TX, 
					 .end		 = DMACH_DSPI3_TX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 [3] = { 
					 .start 	   = DMACH_DSPI3_RX, 
					 .end		 = DMACH_DSPI3_RX, 
					 .flags 	   = IORESOURCE_DMA, 
			 }, 
			 #endif 
	 #else 
			 [0] = { 
				 .start  = SPI3_BASE_ADDR, 
				 .end	 = SPI3_BASE_ADDR + 1024, 
				 .flags  = IORESOURCE_MEM, 
			 }, 
		 #ifdef CONFIG_SUNXI_SPI_NDMA 
			 [1] = { 
				 .start  = N_SRC_SPI3_RX, 
				 .end	 = N_SRC_SPI3_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI3_TX, 
				 .end	 = N_DST_SPI3_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #else 
			 [1] = { 
				 .start  = D_SRC_SPI3_RX, 
				 .end	 = D_SRC_SPI3_RX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
			 [2] = { 
				 .start  = N_DST_SPI3_TX, 
				 .end	 = N_DST_SPI3_TX, 
				 .flags  = IORESOURCE_DMA, 
			 }, 
		 #endif 
			 [3] = { 
				 .start  = SW_INT_IRQNO_SPI3, 
				 .end	 = SW_INT_IRQNO_SPI3, 
				 .flags  = IORESOURCE_IRQ, 
			 } 
	 #endif 

};
static struct platform_device sunxi_spi3_device = {
#if defined CONFIG_ARCH_SUN4I 
	 .name		 = "sun4i-spi", 
#elif defined CONFIG_ARCH_SUN7I 
	 .name		 = "sun7i-spi", 
#endif

	.id			= 3,
	.num_resources	= ARRAY_SIZE(sunxi_spi3_resources),
	.resource	= sunxi_spi3_resources,
	.dev		= {
		.platform_data = &sunxi_spi3_pdata,
	},
};
#endif

/* ---------------- spi resource and platform data end ----------------------- */
static struct spi_board_info *spi_boards = NULL;
int __devinit spi_sunxi_register_spidev(void)
{
    int spi_dev_num = 0;
    int ret = 0;
    int i = 0;
    unsigned int irq_gpio = 0;
    char spi_board_name[32] = {0};
    struct spi_board_info* board;

    ret = script_parser_fetch("spi_devices", "spi_dev_num", &spi_dev_num, sizeof(int));
    if(ret != SCRIPT_PARSER_OK){
        spi_msg("Get spi devices number failed\n");
        return -1;
    }
    spi_msg("Found %d spi devices in config files\n", spi_dev_num);

    /* alloc spidev board information structure */
    spi_boards = (struct spi_board_info*)kzalloc(sizeof(struct spi_board_info) * spi_dev_num, GFP_KERNEL);
    if (spi_boards == NULL)
    {
        spi_msg("Alloc spi board information failed \n");
        return -1;
    }

    spi_msg("%-10s %-16s %-16s %-8s %-4s %-4s\n", "boards num", "modalias", "max_spd_hz", "bus_num", "cs", "mode");
    for (i=0; i<spi_dev_num; i++)
    {
        board = &spi_boards[i];
        sprintf(spi_board_name, "spi_board%d", i);
        ret = script_parser_fetch(spi_board_name, "modalias", (void*)board->modalias, sizeof(char*));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices modalias failed\n");
            goto fail;
        }
        ret = script_parser_fetch(spi_board_name, "max_speed_hz", (void*)&board->max_speed_hz, sizeof(int));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices max_speed_hz failed\n");
            goto fail;
        }
        ret = script_parser_fetch(spi_board_name, "bus_num", (void*)&board->bus_num, sizeof(u16));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices bus_num failed\n");
            goto fail;
        }
        ret = script_parser_fetch(spi_board_name, "chip_select", (void*)&board->chip_select, sizeof(u16));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices chip_select failed\n");
            goto fail;
        }
        ret = script_parser_fetch(spi_board_name, "mode", (void*)&board->mode, sizeof(u8));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices mode failed\n");
            goto fail;
        }
	ret = script_parser_fetch(spi_board_name, "irq_gpio", (void*)&irq_gpio, sizeof(unsigned int));
	if (ret != SCRIPT_PARSER_OK) {
		spi_msg("%s irq gpio not used\n", spi_board_name);
		board->irq = -1;
	} else if(gpio_request(irq_gpio, spi_board_name) == 0)
		board->irq = gpio_to_irq(irq_gpio);
        /*
        ret = script_parser_fetch(spi_board_name, "full_duplex", &board->full_duplex, sizeof(int));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices full_duplex failed\n");
            goto fail;
        }
        ret = script_parser_fetch(spi_board_name, "manual_cs", &board->manual_cs, sizeof(int));
        if(ret != SCRIPT_PARSER_OK) {
            spi_msg("Get spi devices manual_cs failed\n");
            goto fail;
        }
        */
        spi_msg("%-10d %-16s %-16d %-8d %-4d 0x%-4d\n", i, board->modalias, board->max_speed_hz, board->bus_num, board->chip_select, board->mode);
    }

    /* register boards */
    ret = spi_register_board_info(spi_boards, spi_dev_num);
    if (ret)
    {
        spi_msg("Register board information failed\n");
        goto fail;
    }
    return 0;
fail:
    if (spi_boards)
    {
        kfree(spi_boards);
        spi_boards = NULL;
    }
    return -1;
}

static int spi_sunxi_get_cfg_csbitmap(int bus_num)
{
    int value = 0;
    int ret   = 0;
#if defined CONFIG_ARCH_SUN4I
    char *main_name[] = {"spi0_para", "spi1_para", "spi2_para", "spi3_para"};
#elif defined CONFIG_ARCH_SUN5I
    char *main_name[] = {"spi0_para", "spi1_para", "spi2_para"};
#elif defined CONFIG_ARCH_SUN7I 
    char *main_name[] = {"spi0_para", "spi1_para", "spi2_para", "spi3_para"}; 
#endif
    char *sub_name = "spi_cs_bitmap";
    ret = script_parser_fetch(main_name[bus_num], sub_name, &value, sizeof(int));
    if(ret != SCRIPT_PARSER_OK){
        spi_wrn("get spi %d para failed, err code = %d \n", bus_num, ret);
        return 0;
    }
    spi_msg("bus num = %d, spi used = %d \n", bus_num, value);
    return value;
}

/* get configuration in the script */
#define SPI0_USED_MASK 0x1
#define SPI1_USED_MASK 0x2
#define SPI2_USED_MASK 0x4
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN7I
    #define SPI3_USED_MASK 0x8
#endif
static int spi_used = 0;

static int __init spi_sunxi_init(void)
{
    int used = 0;
    int i = 0;
    int ret = 0;
    char spi_para[16] = {0};

    spi_msg("sw spi init !!\n");
    spi_used = 0;
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN7I 
    for (i=0; i<4; i++)
#elif defined  CONFIG_ARCH_SUN5I
    for (i=0; i<3; i++)
#endif
    {
        used = 0;
        sprintf(spi_para, "spi%d_para", i);
        ret = script_parser_fetch(spi_para, "spi_used", &used, sizeof(int));
        if (ret)
        {
            spi_msg("sw spi init fetch spi%d uning configuration failed\n", i);
            continue;
        }
        if (used)
            spi_used |= 1 << i;
    }

    ret = spi_sunxi_register_spidev();
    if (ret)
    {
        spi_msg("register spi devices board info failed \n");
    }

    if (spi_used & SPI0_USED_MASK)
        platform_device_register(&sunxi_spi0_device);
    if (spi_used & SPI1_USED_MASK)
        platform_device_register(&sunxi_spi1_device);
    if (spi_used & SPI2_USED_MASK)
        platform_device_register(&sunxi_spi2_device);		
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN7I
    if (spi_used & SPI3_USED_MASK)
        platform_device_register(&sunxi_spi3_device);
#endif

    if (spi_used)
    {
        return platform_driver_register(&spi_sunxi_driver);
    }
    else
    {
        pr_warning("spi: cannot find any using configuration for \
                    all 4 spi controllers, return directly!\n");
        return 0;
    }
}
module_init(spi_sunxi_init);

static void __exit spi_sunxi_exit(void)
{
    if (spi_used)
	    platform_driver_unregister(&spi_sunxi_driver);
}
module_exit(spi_sunxi_exit);

MODULE_AUTHOR("Victor.Wei @allwinner");
MODULE_DESCRIPTION("SUNXI SPI BUS Driver");
MODULE_ALIAS("platform:sunxi-spi");
MODULE_LICENSE("GPL");

