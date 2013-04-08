
/* include/asm/alce68k.h
 * Alcetronics M68K board
 *
 * 2012.04.17, ljalvs@gmail.com, Created.
 *
 */

#ifndef _ALCE68K_H_
#define _ALCE68K_H_

#include <linux/platform_device.h>


#define BYTE_REF(addr) (*((volatile unsigned char*)addr))
#define WORD_REF(addr) (*((volatile unsigned short*)addr))
#define LONG_REF(addr) (*((volatile unsigned long*)addr))

#define PUT_FIELD(field, val) (((val) << field##_SHIFT) & field##_MASK)
#define GET_FIELD(reg, field) (((reg) & field##_MASK) >> field##_SHIFT)


/* Peripheral control register */

#define PCR0_ADDR	0xF00000
#define PCR0		BYTE_REF(PCR0_ADDR)
#define PCR1_ADDR	0xF00001
#define PCR1		BYTE_REF(PCR1_ADDR)

#define PCR_REDLED	0x80

/* Timer modules */

#define TMR0_ADDR	0xF00002
#define	TMR0		WORD_REF(TMR0_ADDR)

/*#define TMR0_CTRL_ADDR	0xF00000
#define TMR0_CTRL	BYTE_REF(TMR0_CTRL_ADDR)
#define TMR0_REG	LONG_REF(TMR0_CTRL_ADDR)

#define TMR0_VAL16_ADDR	0xF00006
#define TMR0_VAL16	WORD_REF(TMR0_VAL16_ADDR)

#define TMR0_VAL_ADDR	0xF00004
#define TMR0_VAL	LONG_REF(TMR0_VAL_ADDR)



#define TMR1_CTRL_ADDR	0xF00008
#define TMR1_CTRL	BYTE_REF(TMR1_CTRL_ADDR)
#define TMR1_REG	LONG_REF(TMR1_CTRL_ADDR)

#define TMR1_VAL16_ADDR	0xF0000C
#define TMR1_VAL16	WORD_REF(TMR1_VAL16_ADDR)

#define TMR1_VAL_ADDR	0xF0000E
#define TMR1_VAL	LONG_REF(TMR1_VAL_ADDR)
*/


#define TMR0_EN		0x01

#define TMR0_RINT	0x01


/* FTDI245 */


struct alce_platform_uart {
	unsigned long	mapbase;	/* Physical address base */
	void __iomem	*membase;	/* Virtual address if mapped */
	unsigned int	irq;		/* Interrupt vector */
	//unsigned int	uartclk;	/* UART clock rate */
};


#define FTDI_DATA_ADDR	0xE00000
#define FTDI_STAT_ADDR	0xE00002

#define FTDI_DATA	BYTE_REF(FTDI_DATA_ADDR)
#define FTDI_STAT	BYTE_REF(FTDI_STAT_ADDR)

#define FTDI_RXF	0x01
#define FTDI_TXE	0x02




#define FTDI_TXFIFOSIZE	385

/* FPGA UART */


//struct alce_platform_fpgauart {
//	unsigned long	mapbase;	/* Physical address base */
//	void __iomem	*membase;	/* Virtual address if mapped */
//	unsigned int	irq;		/* Interrupt vector */
//	unsigned int	uartclk;	/* UART clock rate */
//};

#define UART_DATA_ADDR	0xF00020
#define UART_STAT_ADDR	0xF00021

#define UART_DATA	BYTE_REF(UART_DATA_ADDR)
#define UART_STAT	BYTE_REF(UART_STAT_ADDR)

#define UART_RXF	0x01
#define UART_RXE	0x02
#define UART_TXB 0x04




/* SPI CONTROL REG MASK */

#define SPICTRL_CS		0x10
#define SPICTRL_EN		0x20
#define SPICTRL_BUSY		0x80



struct alce68kspi_platform_data {
	s16     bus_num;
	u16     num_chipselect;
};


#define SPI0_REGBASE	0xF00014
#define SPI1_REGBASE	0xF00016




/* ETHERNET SPI */


struct alcenet_platform_data {
	unsigned char mac[6];
};


#define ETHSPI_DT_ADDR	0xF00014
#define ETHSPI_CT_ADDR	0xF00015
#define ETHSPI_DATA	BYTE_REF(ETHSPI_DT_ADDR)
#define ETHSPI_CTRL	BYTE_REF(ETHSPI_CT_ADDR)



/* MMC SPI */


struct alcemmc_platform_data {
    unsigned char irq;
};


#define MMCSPI_DT_ADDR	0xF00016
#define MMCSPI_CT_ADDR	0xF00017
#define MMCSPI_DATA	BYTE_REF(MMCSPI_DT_ADDR)
#define MMCSPI_CTRL	BYTE_REF(MMCSPI_CT_ADDR)


#define MMC_SDCD		0x40
#define MMC_RINT		0x02


/* RTC */

#define RTC_BASE_ADDR	0xF00080

#define RTC_SEC_ADDR	0xF00080
#define RTC_SEC		BYTE_REF(RTC_SEC_ADDR)
#define RTC_MIN_ADDR	0xF00082
#define RTC_MIN		BYTE_REF(RTC_MIN_ADDR)
#define RTC_HOU_ADDR	0xF00084
#define RTC_HOU		BYTE_REF(RTC_HOU_ADDR)

#define RTC_DWE_ADDR	0xF00086
#define RTC_DWE		BYTE_REF(RTC_DWE_ADDR)
#define RTC_DAY_ADDR	0xF00087
#define RTC_DAY		BYTE_REF(RTC_DAY_ADDR)
#define RTC_MON_ADDR	0xF00088
#define RTC_MON		BYTE_REF(RTC_MON_ADDR)
#define RTC_YEA_ADDR	0xF00089
#define RTC_YEA		BYTE_REF(RTC_YEA_ADDR)


/* MCP3208 ADC */

#define ADC_DATA_ADDR	0xF00018
#define ADC_CTRL_ADDR	0xF00019
#define ADC_DATA		BYTE_REF(ADC_DATA_ADDR)
#define ADC_CTRL		BYTE_REF(ADC_CTRL_ADDR)

struct alce_platform_adc {
	unsigned long	mapbase;	/* Physical address base */
/*	void __iomem	*membase;*/	/* Virtual address if mapped */
};



/* Interrupt Vectors */


#define INTVEC_TIMER0		64
#define INTVEC_TIMER1		65
#define INTVEC_MMCCD		66
#define INTVEC_FTDI		68
#define INTVEC_RTC		80
#define INTVEC_ENC28J60		81
#define INTVEC_FPGAUART		82
#define INTVEC_MAS		83

/* Interrupt numbers
 * At the moment different from vector numbers */

#define INT_TIMER0		1
#define INT_TIMER1		2
#define INT_MMCCD		3

#define INT_ENC28J60		18

#define INT_UART		19
#define INT_MAS		20


/* Interrupt control */

#define INT_CTRL_ADDR	0xF00011
#define INT_CTRL	BYTE_REF(INT_CTRL_ADDR)

#define FTDI_IEN	0x01
#define FTDI_RXIE	0x02
#define FTDI_TXIE	0x04
#define ETH_IEN		0x08
#define UART_IEN	0x10

#define MAS_IEN	0x20
#define MMCCD_IEN	0x40


#endif /* _ALCE68K_H_ */
