/***************************************************************************
 *
 *  config-alce68k.c
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive
 *  for more details.
 *
 *  2012.04.17, ljalvs@gmail.com, Created based on 68328/coldfire sources.
 *
 ***************************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
//#include <asm/system.h>
#include <asm/machdep.h>
#include <asm/alce68k.h>
//#include <linux/rtc.h>

#include <linux/console.h>

#include <linux/m48t86.h>

#include <linux/major.h>

#include <linux/root_dev.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>


#ifdef CONFIG_HEARTBEAT
void (*mach_heartbeat) (int);
EXPORT_SYMBOL(mach_heartbeat);
static void alce68k_heartbeat(int on);
#endif


static struct mtd_partition alce68k_flash_parts[] = {
	{
		.name	= "kernel",
		.offset	= 0x00000000,
		.size	= 0x00020000,
	},
	{
		.name	= "flash",
		.offset	= 0x00020000,
		.size	= 0x000E0000,
	},
	{
		.name	= "romfs",
		.offset	= 0x00100000,
		.size	= 0x00100000,
	},
};

/*static struct mtd_partition alce68k_flash2_parts[] = {
	{
		.name	= "kernel",
		.offset	= 0x00000000,
		.size	= 0x00100000,
	},
};*/

/*static struct mtd_partition alce68k_flash1_parts[] = {
	{
		.name	= "Bootloader",
		.offset	= 0x00000000,
		.size	= 0x00008000,
	},
	{
		.name	= "FPGA",
		.offset	= 0x00008000,
		.size	= 0x00002000,
	},
	{
		.name	= "kernel",
		.offset	= 0x00010000,
		.size	= 0x000F0000,
	},
};*/


static struct resource alce68k_flash_resource = {
	.start		= 0x00A00000,
	.end		= 0x00C00000,
	.flags		= IORESOURCE_MEM,
};



static struct physmap_flash_data alce68k_flash_data = {
	.width		= 2,
	.nr_parts	= ARRAY_SIZE(alce68k_flash_parts),
	.parts		= alce68k_flash_parts,
};



static struct platform_device alce68k_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data = &alce68k_flash_data,
	},
	.num_resources	= 1,
	.resource	= &alce68k_flash_resource,
};


static int __init init_alce68k_flash(void)
{
	platform_device_register(&alce68k_flash);
	return 0;
}

arch_initcall(init_alce68k_flash);





/* enc28j60 */

static struct resource enc28j60_resources[] = {
	[0] = {
		.start	= ETHSPI_DT_ADDR,
		.end	= ETHSPI_CT_ADDR,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_ENC28J60,
		.end	= INT_ENC28J60,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device enc28j60_device = {
	.name		= "enc28j60",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(enc28j60_resources),
	.resource	= enc28j60_resources,
};

/* end of enc28j60 */





/* mmc */

static struct resource alcemmc_resources[] = {
	[0] = {
		.start	= MMCSPI_DT_ADDR,
		.end	= MMCSPI_CT_ADDR,
		.flags	= IORESOURCE_MEM,
	},
};

static struct alcemmc_platform_data alcemmc_data[] = {
	{
	    .irq		= INT_MMCCD,
	},    
};

static struct platform_device alcemmc_device = {
	.name	= "alce_mmc",
	.id	= -1,
	
	
	.dev	= {
		.platform_data = &alcemmc_data,
	},
		
	.num_resources	= ARRAY_SIZE(alcemmc_resources),
	.resource	= alcemmc_resources,
};

/* end of mmc */




/*


static struct resource alce68kspi_resources[] = {
	{
		.start		= SPI0_REGBASE,
		.end		= SPI0_REGBASE + 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct resource alce68kspi2_resources[] = {
	{
		.start		= SPI1_REGBASE,
		.end		= SPI1_REGBASE + 1,
		.flags		= IORESOURCE_MEM,
	},
};



static struct alce68kspi_platform_data alce68kspi_data = {
	.bus_num		= 0,
	.num_chipselect		= 1,
};

static struct alce68kspi_platform_data alce68kspi2_data = {
	.bus_num		= 1,
	.num_chipselect		= 1,
};

*/


/*
   INTERFACE between board init code and SPI infrastructure.
*/   
/*
static struct spi_board_info alce68k_spi_board_info[] __initdata = {

	{
		.modalias = "enc28j60",
		.max_speed_hz = 20000000,
		.bus_num = 0,
		.chip_select = 0,
		.irq = 18,
	},

	{
		.modalias = "mmc_spi",
		.max_speed_hz = 20000000,
		.bus_num = 1,
		.chip_select = 0,
	},


};
*/



static struct resource ft245uart_res[] = {
	{
		.start = 0xE00000,
		.end = 0xE00002,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 5,
		.end = 5,
		.flags = IORESOURCE_IRQ,
	},
}; 






/***************************************************************************/

static struct alce_platform_adc alce_mcp3208_platform_data[] = {
	{
		.mapbase = 0xF00018,
	},
};


static struct alce_platform_uart alce_ft245uart_platform_data[] = {
	{
		.mapbase = 0xE00000,
		.irq     = 5,
	},
};


static struct alce_platform_uart alce_fpgauart_platform_data[] = {
	{
		.mapbase = 0xF00020 ,
		.irq     = 19,
	},
};


static unsigned char alce68k_rtc_readbyte(unsigned long addr)
{
	//writeb(addr, TS_RTC_CTRL);
	//return readb(TS_RTC_DATA);
	//printk
	return BYTE_REF(RTC_BASE_ADDR + addr); //(*((volatile unsigned char*)(addr + 0xF00100)));
}

static void alce68k_rtc_writebyte(unsigned char value, unsigned long addr)
{
	//writeb(addr, TS_RTC_CTRL);
	//writeb(value, TS_RTC_DATA);
	BYTE_REF(RTC_BASE_ADDR + addr) = value;
	//(*((volatile unsigned char*)(addr + 0xF00100))) = value;
}

static struct m48t86_ops alce68k_rtc_ops = {
	.readbyte	= alce68k_rtc_readbyte,
	.writebyte	= alce68k_rtc_writebyte,
};


#ifdef CONFIG_MAX31855_ALCE
static struct alce_platform_adc alce_max31855_platform_data[] = {
	{
		.mapbase = 0xF00018,
	},
};

static struct platform_device temp_max31855 = {
	.name			= "max31855",
	.id			= -1,
	.dev		= {
		.platform_data	= &alce_max31855_platform_data,
	},
	.num_resources		= 0,
};
#endif



/***************************************************************************/
/* plataform devices */

static struct platform_device ft245_uart = {
	.name			= "ft245uart",
	.id			= -1,
	.num_resources		= 1,
	.resource		= ft245uart_res,
	.dev.platform_data	= alce_ft245uart_platform_data,
};

static struct platform_device fpga_uart = {
	.name			= "fpgauart",
	.id			= -1,
	.dev.platform_data	= alce_fpgauart_platform_data,
};

static struct platform_device rtc_ds12887 = {
	.name			= "rtc-m48t86",
	.id			= -1,
	.dev		= {
		.platform_data	= &alce68k_rtc_ops,
	},
	.num_resources		= 0,
};

static struct platform_device adc_mcp3208 = {
	.name			= "mcp3208-adc",
	.id			= -1,
	.dev		= {
		.platform_data	= &alce_mcp3208_platform_data,
	},
	.num_resources		= 0,
};



/*
static struct platform_device alce68k_spi = {
	.name			= "alce68k-spi",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(alce68kspi_resources),
	.resource		= alce68kspi_resources,
	.dev.platform_data	= &alce68kspi_data,
};


static struct platform_device alce68k_spi2 = {
	.name			= "alce68k-spi",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(alce68kspi2_resources),
	.resource		= alce68kspi2_resources,
	.dev.platform_data	= &alce68kspi2_data,
};
*/

/***************************************************************************/


static struct platform_device *alce68k_devices[] __initdata = {
	&ft245_uart,
	&enc28j60_device,
	//&alce68k_spi,
	//&alce68k_spi2,
	&alcemmc_device,
	&fpga_uart,
	&rtc_ds12887,
	&adc_mcp3208,
#ifdef CONFIG_MAX31855_ALCE
	&temp_max31855,
#endif
};



/***************************************************************************/

/*void alce68k_timer_gettod(int *year, int *mon, int *day, int *hour, int *min, int *sec);*/

/***************************************************************************/

void alce68k_reset (void)
{
  local_irq_disable();
  asm volatile ("moveal #0x00800000, %a0;\n\t"
		"moveal 0(%a0), %sp;\n\t"
		"moveal 4(%a0), %a0;\n\t"
		"jmp (%a0);");
}


static struct platform_device *fpga_early_devs[] __initdata = {
#if defined(CONFIG_SERIAL_ALCE_CONSOLE)
	&ft245_uart,
#endif
};


void __init alce68k_platform_early_init(void)
{
	printk("[plat-alce68k]: registering early dev resources\n");

	//setup_bvci_lat_unit();

#if defined(CONFIG_SERIAL_ALCE_CONSOLE)
	early_platform_add_devices(fpga_early_devs, ARRAY_SIZE(fpga_early_devs));
	early_platform_driver_register_all("earlyprintk");
	early_platform_driver_probe("earlyprintk", 1, 0);

	add_preferred_console("ttyS", 0, "115200");
#endif
}



#ifdef CONFIG_HEARTBEAT
static void alce68k_heartbeat(int on)
{
	if (on)
		PCR1 |= PCR_REDLED;
	else
		PCR1 &= ~PCR_REDLED;
}
#endif






/***************************************************************************/

void __init config_BSP(char *command, int len)
{
  printk(KERN_INFO "Alcetronics M68K support by Luis Alves <ljalvs@gmail.com>\n");
  
  mach_sched_init = hw_timer_init;
  //mach_hwclk = alce68k_hwclk;
  mach_reset = alce68k_reset;
#ifdef CONFIG_HEARTBEAT
  mach_heartbeat = alce68k_heartbeat;
#endif  
  alce68k_platform_early_init();
  
  
}

/***************************************************************************/


static int __init init_BSP(void)
{
	//int retval = 0;

	platform_add_devices(alce68k_devices, ARRAY_SIZE(alce68k_devices));
	/*if (ARRAY_SIZE(alce68k_spi_board_info))
		retval = spi_register_board_info(alce68k_spi_board_info, ARRAY_SIZE(alce68k_spi_board_info));*/


	//ROOT_DEV = MKDEV(MMC_BLOCK_MAJOR, 1);
	//ROOT_DEV = MKDEV(MTD_BLOCK_MAJOR, 2);
	ROOT_DEV = MKDEV(31, 0);
	return 0;
}

arch_initcall(init_BSP);

/***************************************************************************/
