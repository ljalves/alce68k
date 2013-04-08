/*
 * mmc_spi.c - Access SD/MMC cards through SPI master controllers
 *
 * (C) Copyright 2005, Intec Automation,
 *		Mike Lavender (mike@steroidmicros)
 * (C) Copyright 2006-2007, David Brownell
 * (C) Copyright 2007, Axis Communications,
 *		Hans-Peter Nilsson (hp@axis.com)
 * (C) Copyright 2007, ATRON electronic GmbH,
 *		Jan Nikitenko <jan.nikitenko@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bio.h>
#include <linux/dma-mapping.h>
#include <linux/crc7.h>
#include <linux/crc-itu-t.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>		/* for R1_SPI_* bit values */

#include <linux/platform_device.h>

#include <linux/io.h>

#include <asm/unaligned.h>

#include "asm/alce68k.h"


/* NOTES:
 *
 * - For now, we won't try to interoperate with a real mmc/sd/sdio
 *   controller, although some of them do have hardware support for
 *   SPI protocol.  The main reason for such configs would be mmc-ish
 *   cards like DataFlash, which don't support that "native" protocol.
 *
 *   We don't have a "DataFlash/MMC/SD/SDIO card slot" abstraction to
 *   switch between driver stacks, and in any case if "native" mode
 *   is available, it will be faster and hence preferable.
 *
 * - MMC depends on a different chipselect management policy than the
 *   SPI interface currently supports for shared bus segments:  it needs
 *   to issue multiple spi_message requests with the chipselect active,
 *   using the results of one message to decide the next one to issue.
 *
 *   Pending updates to the programming interface, this driver expects
 *   that it not share the bus with other drivers (precluding conflicts).
 *
 * - We tell the controller to keep the chipselect active from the
 *   beginning of an mmc_host_ops.request until the end.  So beware
 *   of SPI controller drivers that mis-handle the cs_change flag!
 *
 *   However, many cards seem OK with chipselect flapping up/down
 *   during that time ... at least on unshared bus segments.
 */


/*
 * Local protocol constants, internal to data block protocols.
 */

/* Response tokens used to ack each block written: */
#define SPI_MMC_RESPONSE_CODE(x)	((x) & 0x1f)
#define SPI_RESPONSE_ACCEPTED		((2 << 1)|1)
#define SPI_RESPONSE_CRC_ERR		((5 << 1)|1)
#define SPI_RESPONSE_WRITE_ERR		((6 << 1)|1)

/* Read and write blocks start with these tokens and end with crc;
 * on error, read tokens act like a subset of R2_SPI_* values.
 */
#define SPI_TOKEN_SINGLE	0xfe	/* single block r/w, multiblock read */
#define SPI_TOKEN_MULTI_WRITE	0xfc	/* multiblock write */
#define SPI_TOKEN_STOP_TRAN	0xfd	/* terminate multiblock write */

#define MMC_SPI_BLOCKSIZE	512


/* These fixed timeouts come from the latest SD specs, which say to ignore
 * the CSD values.  The R1B value is for card erase (e.g. the "I forgot the
 * card's password" scenario); it's mostly applied to STOP_TRANSMISSION after
 * reads which takes nowhere near that long.  Older cards may be able to use
 * shorter timeouts ... but why bother?
 */
#define r1b_timeout		(HZ * 3)

/* One of the critical speed parameters is the amount of data which may
 * be transferred in one command. If this value is too low, the SD card
 * controller has to do multiple partial block writes (argggh!). With
 * today (2008) SD cards there is little speed gain if we transfer more
 * than 64 KBytes at a time. So use this value until there is any indication
 * that we should do more here.
 */
#define MMC_SPI_BLOCKSATONCE	128

#define SPI_SPEED		0

/****************************************************************************/

/*
 * Local Data Structures
 */

/* "scratch" is per-{command,block} data exchanged with the card */
struct scratch {
	u8			status[29];
// 	u8			data_token;
	__be16			crc_val;
	u8			*end;
};

struct mmc_spi_host {
	struct mmc_host		*mmc;
	struct mmc_command *cmd;
	struct mmc_request *request;
	
	
	void __iomem	*membase;
	
	//struct alcemmc_platform_data *pdata;

	unsigned char		power_mode;
	//u16			powerup_msecs;

	/* buffer used for commands and for message "overhead" */
	struct scratch		*data;
};


/****************************************************************************/

/*
 * MMC-over-SPI protocol glue, used by the MMC stack interface
 */

static int
mmc_spi_readbytes(struct mmc_spi_host *host, unsigned len)
{
	u8	*cp = host->data->status;
	u16	i;

	if (len > sizeof(*host->data)) {
		WARN_ON(1);
		return -EIO;
	}

	for (i = 0; i<len; i++) {
		MMCSPI_DATA = 0xFF;
		while (MMCSPI_CTRL & SPICTRL_BUSY);
		*cp++ = MMCSPI_DATA;
	}	
	return 0;
}

static int mmc_spi_skip(struct mmc_spi_host *host, unsigned long timeout,
			unsigned n, u8 byte)
{
	u8		*cp = host->data->status;
	unsigned long start = jiffies;

	while (1) {
		int		status;
		unsigned	i;

		status = mmc_spi_readbytes(host, n);
		if (status < 0)
			return status;

		for (i = 0; i < n; i++) {
			if (cp[i] != byte)
				return cp[i];
		}

		if (time_is_before_jiffies(start + timeout))
			break;

		/* If we need long timeouts, we may release the CPU.
		 * We use jiffies here because we want to have a relation
		 * between elapsed time and the blocking of the scheduler.
		 */
		if (time_is_before_jiffies(start+1))
			schedule();
	}
	return -ETIMEDOUT;
}

static inline int
mmc_spi_wait_unbusy(struct mmc_spi_host *host, unsigned long timeout)
{
	return mmc_spi_skip(host, timeout, sizeof(host->data->status), 0);
}

/*static int mmc_spi_readtoken(struct mmc_spi_host *host, unsigned long timeout)
{
	return mmc_spi_skip(host, timeout, 1, 0xff);
}*/


/*
 * Note that for SPI, cmd->resp[0] is not the same data as "native" protocol
 * hosts return!  The low byte holds R1_SPI bits.  The next byte may hold
 * R2_SPI bits ... for SEND_STATUS, or after data read errors.
 *
 * cmd->resp[1] holds any four-byte response, for R3 (READ_OCR) and on
 * newer cards R7 (IF_COND).
 */

static char *maptype(struct mmc_command *cmd)
{
	switch (mmc_spi_resp_type(cmd)) {
	case MMC_RSP_SPI_R1:	return "R1";
	case MMC_RSP_SPI_R1B:	return "R1B";
	case MMC_RSP_SPI_R2:	return "R2/R5";
	case MMC_RSP_SPI_R3:	return "R3/R4/R7";
	default:		return "?";
	}
}

/* return zero, else negative errno after setting cmd->error */
static int mmc_spi_response_get(struct mmc_spi_host *host,
		struct mmc_command *cmd, int cs_on)
{
	u8	*cp = host->data->status;
	u8	*end = host->data->end;
	int	value = 0;
	int	bitshift;
	u8 	leftover = 0;
	unsigned short rotator;
	u8 	i;
	char	tag[32];
	//u8	d;

	snprintf(tag, sizeof(tag), "  ... CMD%d response SPI_%s",
		cmd->opcode, maptype(cmd));

	/* Except for data block reads, the whole response will already
	 * be stored in the scratch buffer.  It's somewhere after the
	 * command and the first byte we read after it.  We ignore that
	 * first byte.  After STOP_TRANSMISSION command it may include
	 * two data bits, but otherwise it's all ones.
	 */
	cp += 8;
	while (cp < end && *cp == 0xff)
		cp++;

	/* Data block reads (R1 response types) may need more data... */
	if (cp == end) {
		cp = host->data->status;
		end = cp+1;

		/* Card sends N(CR) (== 1..8) bytes of all-ones then one
		 * status byte ... and we already scanned 2 bytes.
		 *
		 * REVISIT block read paths use nasty byte-at-a-time I/O
		 * so it can always DMA directly into the target buffer.
		 * It'd probably be better to memcpy() the first chunk and
		 * avoid extra i/o calls...
		 *
		 * Note we check for more than 8 bytes, because in practice,
		 * some SD cards are slow...
		 */
		for (i = 2; i < 16; i++) {
			/*value = mmc_spi_readbytes(host, 1);
			if (value < 0)
				goto done;*/
			/*if (*cp != 0xff)
				goto checkstatus;*/
			MMCSPI_DATA = 0xFF;
			while (MMCSPI_CTRL & SPICTRL_BUSY);
			*cp = MMCSPI_DATA;
			if (*cp != 0xFF)
				goto checkstatus;
		}
		value = -ETIMEDOUT;
		goto done;
	}/* else {
	   d = *cp;
	}*/

checkstatus:
	bitshift = 0;
	if (*cp & 0x80)	{
	//if (d & 0x80)	{
		/* Houston, we have an ugly card with a bit-shifted response */
		rotator = *cp++ << 8;
		/* read the next byte */
		if (cp == end) {
		/*	value = mmc_spi_readbytes(host, 1);
			if (value < 0)
				goto done;*/
			cp = host->data->status;
			end = cp+1;
			MMCSPI_DATA = 0xFF;
			while (MMCSPI_CTRL & SPICTRL_BUSY);
			*cp = MMCSPI_DATA;
		}
		rotator |= *cp++;
		//rotator |= d;
		while (rotator & 0x8000) {
			bitshift++;
			rotator <<= 1;
		}
		cmd->resp[0] = rotator >> 8;
		leftover = rotator;
	} else {
		cmd->resp[0] = *cp++;
		//cmd->resp[0] = d;
	}
	cmd->error = 0;

	/* Status byte: the entire seven-bit R1 response.  */
	if (cmd->resp[0] != 0) {
		if ((R1_SPI_PARAMETER | R1_SPI_ADDRESS)
				& cmd->resp[0])
			value = -EFAULT; /* Bad address */
		else if (R1_SPI_ILLEGAL_COMMAND & cmd->resp[0])
			value = -ENOSYS; /* Function not implemented */
		else if (R1_SPI_COM_CRC & cmd->resp[0])
			value = -EILSEQ; /* Illegal byte sequence */
		else if ((R1_SPI_ERASE_SEQ | R1_SPI_ERASE_RESET)
				& cmd->resp[0])
			value = -EIO;    /* I/O error */
		/* else R1_SPI_IDLE, "it's resetting" */
	}

	switch (mmc_spi_resp_type(cmd)) {

	/* SPI R1B == R1 + busy; STOP_TRANSMISSION (for multiblock reads)
	 * and less-common stuff like various erase operations.
	 */
	case MMC_RSP_SPI_R1B:
		/* maybe we read all the busy tokens already */
		while (cp < end && *cp == 0)
			cp++;
		/*if (cp == end)
			mmc_spi_wait_unbusy(host, r1b_timeout);
		break;*/
		if (cp == end) {
		  cp = host->data->status;
		  end = cp+1;
		  MMCSPI_DATA = 0xFF;
		  while (MMCSPI_CTRL & SPICTRL_BUSY);
		  *cp = MMCSPI_DATA;
		  while (*cp == 0) {
		    MMCSPI_DATA = 0xFF;
		    while (MMCSPI_CTRL & SPICTRL_BUSY);
		    *cp = MMCSPI_DATA;
		  }
		}
		break;

	/* SPI R2 == R1 + second status byte; SEND_STATUS
	 * SPI R5 == R1 + data byte; IO_RW_DIRECT
	 */
	case MMC_RSP_SPI_R2:
		/* read the next byte */
		
		if (cp == end) {
			/*value = mmc_spi_readbytes(host, 1);
			if (value < 0)
				goto done;*/
			cp = host->data->status;
			end = cp+1;
			MMCSPI_DATA = 0xFF;
			while (MMCSPI_CTRL & SPICTRL_BUSY);
			*cp = MMCSPI_DATA;
		}
		if (bitshift) {
			rotator = leftover << 8;
			rotator |= *cp << bitshift;
			cmd->resp[0] |= (rotator & 0xFF00);
		} else {
			cmd->resp[0] |= *cp << 8;
		}
		break;

	/* SPI R3, R4, or R7 == R1 + 4 bytes */
	case MMC_RSP_SPI_R3:
		rotator = leftover << 8;
		cmd->resp[1] = 0;
		for (i = 0; i < 4; i++) {
			cmd->resp[1] <<= 8;
			/* read the next byte */
			if (cp == end) {
				/*value = mmc_spi_readbytes(host, 1);
				if (value < 0)
					goto done;*/
				cp = host->data->status;
				end = cp+1;
				MMCSPI_DATA = 0xFF;
				while (MMCSPI_CTRL & SPICTRL_BUSY);
				*cp = MMCSPI_DATA;
			}
			if (bitshift) {
				rotator |= *cp++ << bitshift;
				cmd->resp[1] |= (rotator >> 8);
				rotator <<= 8;
			} else {
				cmd->resp[1] |= *cp++;
			}
		}
		break;

	/* SPI R1 == just one status byte */
	case MMC_RSP_SPI_R1:
		break;

	default:
		printk("mmc_spi: bad response type %04x\n",
				mmc_spi_resp_type(cmd));
		if (value >= 0)
			value = -EINVAL;
		goto done;
	}

	if (value < 0)
		printk("%s: resp %04x %08x\n",
			tag, cmd->resp[0], cmd->resp[1]);

	/* disable chipselect on errors and some success cases */
	if (value >= 0 && cs_on)
		return value;
done:
	if (value < 0)
		cmd->error = value;
	
	// cs off
	//mmc_cs_off(host);
	MMCSPI_CTRL = SPICTRL_EN | SPI_SPEED;


	//printk("OK: %s: resp %04x %08x\n",
	//		tag, cmd->resp[0], cmd->resp[1]);

	return value;
}

/* Issue command and read its response.
 * Returns zero on success, negative for error.
 *
 * On error, caller must cope with mmc core retry mechanism.  That
 * means immediate low-level resubmit, which affects the bus lock...
 */
static int
mmc_spi_command_send(struct mmc_spi_host *host,
		struct mmc_request *mrq,
		struct mmc_command *cmd, int cs_on)
{
	struct scratch		*data = host->data;
	u8			*cp = data->status;
	u32			arg = cmd->arg;

	int i;
	u8	s;
	
	//printk("  mmc_send_cmd: cmd=%d, arg=%d\n", cmd->opcode, arg);

	// prepare data
	
	memset(cp++, 0xff, sizeof(data->status));
	
	*cp++ = 0x40 | cmd->opcode;
	*cp++ = (u8)(arg >> 24);
	*cp++ = (u8)(arg >> 16);
	*cp++ = (u8)(arg >> 8);
	*cp++ = (u8)arg;
	*cp++ = (crc7(0, &data->status[1], 5) << 1) | 0x01;
	

	if (cs_on && (mrq->data->flags & MMC_DATA_READ)) {
		cp += 2;	/* min(N(CR)) + status */
		/* R1 */
	} else {
		cp += 10;	/* max(N(CR)) + status + min(N(RC),N(WR)) */
		if (cmd->flags & MMC_RSP_SPI_S2)	/* R2/R5 */
			cp++;
		else if (cmd->flags & MMC_RSP_SPI_B4)	/* R3/R4/R7 */
			cp += 4;
		else if (cmd->flags & MMC_RSP_BUSY)	/* R1B */
			cp = data->status + sizeof(data->status);
		/* else:  R1 (most commands) */
	}

	//printk("  mmc_spi: CMD%d, resp %s\n",
	//	cmd->opcode, maptype(cmd));
	
	
	// send command and leave CS active

	s = cp - data->status;
	
	data->end = cp;
	
	cp = data->status;
	
	
	// cs low
	MMCSPI_CTRL = SPICTRL_EN | SPICTRL_CS | SPI_SPEED;

	for (i = 0; i < s; i++) {
		MMCSPI_DATA = *cp;
		while (MMCSPI_CTRL & SPICTRL_BUSY);
		*cp++ = MMCSPI_DATA;
	}
	
	return mmc_spi_response_get(host, cmd, cs_on);
}



/*
 * Write one block:
 *  - caller handled preceding N(WR) [1+] all-ones bytes
 *  - data block
 *	+ token
 *	+ data bytes
 *	+ crc16
 *  - an all-ones byte ... card writes a data-response byte
 *  - followed by N(EC) [0+] all-ones bytes, card writes zero/'busy'
 *
 * Return negative errno, else success.
 */
static int
mmc_spi_writeblock(struct mmc_spi_host *host, u8 *t, unsigned long timeout, unsigned len)
{
	int			status;
	u16 i;
	struct scratch		*scratch = host->data;
	u32			pattern, p2;
	u8	*cp = t;
	u8	c;

	if (host->mmc->use_spi_crc)
		scratch->crc_val = cpu_to_be16(
				crc_itu_t(0, t, len));

	// write 512 bytes
	for (i = 0; i<len; i++) {
		MMCSPI_DATA = *cp++;
		while (MMCSPI_CTRL & SPICTRL_BUSY);
	}

	// send CRC

	c = (u8) ((scratch->crc_val) >> 8);
	MMCSPI_DATA = c;
	while (MMCSPI_CTRL & SPICTRL_BUSY);

	c = (u8) (scratch->crc_val);
	MMCSPI_DATA = c;
	while (MMCSPI_CTRL & SPICTRL_BUSY);

	/* get write status */
	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	pattern = MMCSPI_DATA << 24;

	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	pattern |= MMCSPI_DATA << 16;

	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	pattern |= MMCSPI_DATA << 8;

	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	pattern |= MMCSPI_DATA;

	
	p2 = pattern;
	
	
	pattern |= 0xE0000000;
	
	/* left-adjust to leading 0 bit */
	while (pattern & 0x80000000)
		pattern <<= 1;
	/* right-adjust for pattern matching. Code is in bit 4..0 now. */
	pattern >>= 27;

	
	
	switch (pattern) {
	case SPI_RESPONSE_ACCEPTED:
		//printk("ok\n");
		status = 0;
		break;
	case SPI_RESPONSE_CRC_ERR:
		printk("crc_err\n");
		/* host shall then issue MMC_STOP_TRANSMISSION */
		status = -EILSEQ;
		break;
	case SPI_RESPONSE_WRITE_ERR:
		printk("write_err\n");
		/* host shall then issue MMC_STOP_TRANSMISSION,
		 * and should MMC_SEND_STATUS to sort it out
		 */
		status = -EIO;
		break;
	default:
		printk("def_err\n");
		status = -EPROTO;
		break;
	}
	if (status != 0) {
	    printk("pat1=%x\n", p2);
	    printk("pat2=%x\n", pattern);
		printk("write error %x (%d)\n",
			p2, status);
		return status;
	}
	
	/* Return when not busy.  If we didn't collect that status yet,
	 * we'll need some more I/O.
	 */

	return mmc_spi_wait_unbusy(host, timeout);
}

/*
 * Read one block:
 *  - skip leading all-ones bytes ... either
 *      + N(AC) [1..f(clock,CSD)] usually, else
 *      + N(CX) [0..8] when reading CSD or CID
 *  - data block
 *	+ token ... if error token, no data or crc
 *	+ data bytes
 *	+ crc16
 *
 * After single block reads, we're done; N(EC) [0+] all-ones bytes follow
 * before dropping chipselect.
 *
 * For multiblock reads, caller either reads the next block or issues a
 * STOP_TRANSMISSION command.
 */
static int
//int
mmc_spi_readblock(struct mmc_spi_host *host, u8 *t, unsigned long timeout, u16 len)
{

	int			status;
	struct scratch		*scratch = host->data;
	unsigned int 		bitshift;
	u8			leftover;

	u8	*cp;
	u8	c;
	u16	i;
	
	//unsigned char *spi_d;
	//int i, j;
	
	unsigned long start = jiffies;

	/*status = mmc_spi_readbytes(host, 1);
	if (status < 0)
		return status;
	status = scratch->status[0];*/
	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	status = MMCSPI_DATA;
	
	if (status == 0xff || status == 0) {
		//status = mmc_spi_readtoken(host, timeout);
		
		while (1) {
			MMCSPI_DATA = 0xFF;
			while (MMCSPI_CTRL & SPICTRL_BUSY);
			status = MMCSPI_DATA;

			if (status != 0xFF)
				break;

			if (time_is_before_jiffies(start + timeout)) {
				status = -ETIMEDOUT;
				break;
			}

			if (time_is_before_jiffies(start+1))
				schedule();
		}
	}

	if (status < 0) {
		printk("    read error %02x (%d)\n", status, status);
		return status;
	}
	
	/* The token may be bit-shifted...
	 * the first 0-bit precedes the data stream.
	 */
	bitshift = 7;
	while (status & 0x80) {
		status <<= 1;
		bitshift--;
	}
	leftover = status << 1;
	
	//spi_d = (unsigned char*) MMCSPI_DT_ADDR;
	//printk("len: %d\n", len);
	//i = len;
	//while (i--) {
	cp = t;
	for (i = 0; i<len; i++) {
		//MMCSPI_DATA = 0xFF;
		//*spi_d = 0xFF;
		writeb(0xff, host->membase);
		//while (MMCSPI_CTRL & SPICTRL_BUSY);
		//*cp++ = MMCSPI_DATA; //*spi_d;
		*cp++ = readb(host->membase);
	}
	//cp = t;
	
	asm volatile("nop\t\n"
		    "nop\t\n"
		    "nop\t\n");
	/*asm volatile("lea 0xf00016,%%a2\t\n"
		     "movel %1,%%a3\t\n"
		     "moveb %2,%%d1\t\n"
		     "1:\n\t"
		     "moveb  #0xff,(%%a2)\n\t"
		     "nop\t\n"
		     "nop\t\n"
		     "nop\t\n"
		     "nop\t\n"
		     "nop\t\n"
		     "nop\t\n"
		     "moveb  (%%a2),(%%a3)+\n\t"
		     "subiw  #1,%%d1\n\t"
		     "bnes   1b\n\t"
		     :
		     : "a" (spi_d), "a" (t), "d" (len)
		     : "d1", "a2", "a3" );*/

	//len = i;
	
	// read crc
	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	c = MMCSPI_DATA;
	
	scratch->crc_val = (u16) (c << 8);
	
	MMCSPI_DATA = 0xFF;
	while (MMCSPI_CTRL & SPICTRL_BUSY);
	c = MMCSPI_DATA;

	scratch->crc_val |= (u16) c;


	if (bitshift) {
		printk("blk_read: bitshift!\n");
		/* Walk through the data and the crc and do
		 * all the magic to get byte-aligned data.
		 */
		cp = t;
		unsigned int len_;
		unsigned int bitright = 8 - bitshift;
		u8 temp;
		for (len_ = len; len_; len_--) {
			temp = *cp;
			*cp++ = leftover | (temp >> bitshift);
			leftover = temp << bitright;
		}
		cp = (u8 *) &scratch->crc_val;
		temp = *cp;
		*cp++ = leftover | (temp >> bitshift);
		leftover = temp << bitright;
		temp = *cp;
		*cp = leftover | (temp >> bitshift);
	}
	
	if (host->mmc->use_spi_crc) {
		u16 crc = crc_itu_t(0, t, len);
		be16_to_cpus(&scratch->crc_val);
		if (scratch->crc_val != crc) {
			printk("read - crc error: crc_val=0x%04x, "
					"computed=0x%04x len=%d\n",
					scratch->crc_val, crc, len);
			return -EILSEQ;
		}
	}
	
	
	return 0;
}

/*
 * An MMC/SD data stage includes one or more blocks, optional CRCs,
 * and inline handshaking.  That handhaking makes it unlike most
 * other SPI protocol stacks.
 */
static void
mmc_spi_data_do(struct mmc_spi_host *host, struct mmc_command *cmd,
		struct mmc_data *data, u32 blk_size)
{
	int			multiple = (data->blocks > 1);
	u32			clock_rate;
	unsigned long		timeout;
	enum dma_data_direction	direction;
	struct scatterlist	*sg;
	unsigned		n_sg;
	unsigned		tlen;
	
	

	if (data->flags & MMC_DATA_READ) {
		direction = DMA_FROM_DEVICE;
	} else {
		direction = DMA_TO_DEVICE;
	}

	/*timeout = data->timeout_ns + data->timeout_clks / 20;
	timeout = usecs_to_jiffies((unsigned int)(timeout / 1000)) + 1;*/
	timeout = 11;

	for (sg = data->sg, n_sg = data->sg_len; n_sg; n_sg--, sg++) {
		int			status = 0;
		void			*kmap_addr;
		unsigned		length = sg->length;
		
		u8	*cp;

		kmap_addr = kmap(sg_page(sg));
		cp = kmap_addr + sg->offset;

		/* transfer each block, and update request status */
		while (length) {
			tlen = min(length, blk_size);

			//printk("    mmc_spi: %s block, %d bytes\n",
			//	(direction == DMA_TO_DEVICE)
			//	? "write"
			//	: "read",
			//	tlen);

			if (direction == DMA_TO_DEVICE) {
			
				if (multiple) {
					MMCSPI_DATA = SPI_TOKEN_MULTI_WRITE;
				} else {
					MMCSPI_DATA = SPI_TOKEN_SINGLE;
				}
				while (MMCSPI_CTRL & SPICTRL_BUSY);
				
				status = mmc_spi_writeblock(host, cp, timeout, tlen);
				cp += tlen;
			}
			else {
				//printk("pointer before = %08x\n", cp);
				status = mmc_spi_readblock(host, cp, timeout, tlen);
				cp += tlen;
				//printk("pointer after  = %08x\n", cp);
			}
			if (status < 0)
				break;

			data->bytes_xfered += tlen;
			length -= tlen;

			if (!multiple)
				break;
		}

		/* discard mappings */
		if (direction == DMA_FROM_DEVICE)
			flush_kernel_dcache_page(sg_page(sg));
		kunmap(sg_page(sg));
		
		if (status < 0) {
			data->error = status;
			printk("%s status %d\n",
				(direction == DMA_TO_DEVICE)
					? "write" : "read",
				status);
			break;
		}
	}
//				status = mmc_spi_readblock(host, cp, timeout, tlen);

	

	if (direction == DMA_TO_DEVICE && multiple) {
	
		int		tmp;
		u8 c;
		
		MMCSPI_DATA = SPI_TOKEN_STOP_TRAN;
		while (MMCSPI_CTRL & SPICTRL_BUSY);

		MMCSPI_DATA = SPI_TOKEN_STOP_TRAN;
		while (MMCSPI_CTRL & SPICTRL_BUSY);

		MMCSPI_DATA = SPI_TOKEN_STOP_TRAN;
		while (MMCSPI_CTRL & SPICTRL_BUSY);
		
		c = MMCSPI_DATA;
		if (c != 0)
		    return;

		tmp = mmc_spi_wait_unbusy(host, timeout);
		if (tmp < 0 && !data->error)
			data->error = tmp;
	
	}
}

/****************************************************************************/

/*
 * MMC driver implementation -- the interface to the MMC stack
 */

static void mmc_spi_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_spi_host	*host = mmc_priv(mmc);
	int			status = -EINVAL;
	int			crc_retry = 5;
	struct mmc_command	stop;


crc_recover:

	status = mmc_spi_command_send(host, mrq, mrq->cmd, mrq->data != NULL);
	
	if (status == 0 && mrq->data) {
		mmc_spi_data_do(host, mrq->cmd, mrq->data, mrq->data->blksz);
	
		if (mrq->data->error == -EILSEQ && crc_retry) {
			stop.opcode = MMC_STOP_TRANSMISSION;
			stop.arg = 0;
			stop.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
			status = mmc_spi_command_send(host, mrq, &stop, 0);
			crc_retry--;
			mrq->data->error = 0;
			goto crc_recover;
		}

		if (mrq->stop)
			status = mmc_spi_command_send(host, mrq, mrq->stop, 0);
		else
		  MMCSPI_CTRL = SPICTRL_EN | SPI_SPEED;
		  //mmc_cs_off(host);
	
	}
	
	mmc_request_done(host->mmc, mrq);
}

/* See Section 6.4.1, in SD "Simplified Physical Layer Specification 2.0"
 *
 * NOTE that here we can't know that the card has just been powered up;
 * not all MMC/SD sockets support power switching.
 *
 * FIXME when the card is still in SPI mode, e.g. from a previous kernel,
 * this doesn't seem to do the right thing at all...
 */
static void mmc_spi_initsequence(struct mmc_spi_host *host)
{
	u8 i;
	
	/* Try to be very sure any previous command has completed;
	 * wait till not-busy, skip debris from any old commands.
	 */
	mmc_spi_wait_unbusy(host, r1b_timeout);

	// chip select high 
	MMCSPI_CTRL = SPICTRL_EN | SPI_SPEED;
		
	for (i = 0; i < 10; i++) {
		MMCSPI_DATA = 0xFF;
		while (MMCSPI_CTRL & SPICTRL_BUSY);
	}
}

static char *mmc_powerstring(u8 power_mode)
{
	switch (power_mode) {
	case MMC_POWER_OFF: return "off";
	case MMC_POWER_UP:  return "up";
	case MMC_POWER_ON:  return "on";
	}
	return "?";
}

static void mmc_spi_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmc_spi_host *host = mmc_priv(mmc);
	

	if (ios->clock == 0) {
		printk("mmc_spi: clk = 0\n");
	} else {
		printk("mmc_spi: clock to %d Hz\n", ios->clock);
	}
	
	
	if (host->power_mode != ios->power_mode) {
		printk("mmc_spi: mmc_ios_pm\n");
	
		if (ios->power_mode == MMC_POWER_ON)
			mmc_spi_initsequence(host);

		host->power_mode = ios->power_mode;
	}


	
}

static int mmc_spi_get_ro(struct mmc_host *mmc)
{
	//printk("mmc_ro\n");
	return -ENOSYS;
}

static int mmc_spi_get_cd(struct mmc_host *mmc)
{
	struct mmc_spi_host *host = mmc_priv(mmc);
	u8 cd;
	
	//cd = MMCSPI_CTRL & MMC_SDCD;
	cd = readb(host->membase+1) & MMC_SDCD;
	printk("mmc_cd = %d\n", cd);
	if (cd != 0)
		return 0;
	else
		return 1;
	
	/*printk("mmc_cd\n");
	return -ENOSYS;*/
}

static const struct mmc_host_ops mmc_spi_ops = {
	.request	= mmc_spi_request,
	.set_ios	= mmc_spi_set_ios,
	.get_ro		= mmc_spi_get_ro,
	.get_cd		= mmc_spi_get_cd,
};


/****************************************************************************/

/*
 * SPI driver implementation
 */

static irqreturn_t
mmc_spi_detect_irq(int irq, void *mmc)
{
	struct mmc_spi_host *host = mmc_priv(mmc);

	PCR0 = MMC_RINT;
	mmc_detect_change(mmc, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}


static int mmc_spi_probe(struct platform_device *pdev)
{
	struct alcemmc_platform_data *pdata = pdev->dev.platform_data;
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct mmc_host		*mmc;
	struct mmc_spi_host	*host;
	int			status;


	mmc = mmc_alloc_host(sizeof(struct mmc_spi_host), &pdev->dev);
	if (!mmc)
		goto nomem;

	mmc->ops = &mmc_spi_ops;
	mmc->f_min = 400000;
	mmc->f_max = 20000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_SPI;

	mmc->max_blk_size = MMC_SPI_BLOCKSIZE;
	mmc->max_blk_count = MMC_SPI_BLOCKSATONCE;
	mmc->max_req_size = MMC_SPI_BLOCKSATONCE * MMC_SPI_BLOCKSIZE;
	mmc->max_segs = MMC_SPI_BLOCKSATONCE;
	mmc->max_seg_size  = MMC_SPI_BLOCKSATONCE * MMC_SPI_BLOCKSIZE;




	host = mmc_priv(mmc);
	host->mmc = mmc;
	
	/* Platform data is used to hook up things like card sensing
	 * and power switching gpios.
	 */
	 
	host->membase = (u8 __iomem *) res->start;
	
	
	/*if (host->pdata && host->pdata->setpower) {
		host->powerup_msecs = host->pdata->powerup_msecs;
		if (!host->powerup_msecs || host->powerup_msecs > 250)
			host->powerup_msecs = 250;
	}*/

	dev_set_drvdata(&pdev->dev, mmc);

	/* preallocate dma buffers */
	/*host->data = kmalloc(sizeof(*host->data), GFP_KERNEL);
	if (!host->data)
		goto fail_nobuf1;

	if (spi->master->dev.parent->dma_mask) {
		struct device	*dev = spi->master->dev.parent;

		host->dma_dev = dev;
		host->ones_dma = dma_map_single(dev, ones,
				MMC_SPI_BLOCKSIZE, DMA_TO_DEVICE);
		host->data_dma = dma_map_single(dev, host->data,
				sizeof(*host->data), DMA_BIDIRECTIONAL);*/

		/* REVISIT in theory those map operations can fail... */

		/*dma_sync_single_for_cpu(host->dma_dev,
				host->data_dma, sizeof(*host->data),
				DMA_BIDIRECTIONAL);
	}*/

	/* setup message for status/busy readback */
	/*spi_message_init(&host->readback);
	host->readback.is_dma_mapped = (host->dma_dev != NULL);*/

	/*spi_message_add_tail(&host->status, &host->readback);
	host->status.tx_buf = host->ones;
	host->status.tx_dma = host->ones_dma;
	host->status.rx_buf = &host->data->status;
	host->status.rx_dma = host->data_dma + offsetof(struct scratch, status);
	host->status.cs_change = 1;*/

	/* register card detect irq */
	PCR0 = MMC_RINT;
	if (request_irq(pdata->irq, mmc_spi_detect_irq, 0, "MMCCD", mmc))
		printk(KERN_ERR "SD CD: unable to attach SD CD "
			"interrupt vector=%d\n", pdata->irq);

	/* Enable interrupt */
	INT_CTRL |= MMCCD_IEN;
	
	
	
	/*if (host->pdata && host->pdata->init) {
		status = host->pdata->init(&spi->dev, mmc_spi_detect_irq, mmc);
		if (status != 0)
			goto fail_glue_init;
	}*/

	/* pass platform capabilities, if any */
	/*if (host->pdata)
		mmc->caps |= host->pdata->caps;*/

	status = mmc_add_host(mmc);
	if (status != 0)
		goto fail_add_host;

	/*dev_info(&pdev->dev, "SD/MMC host %s%s%s%s%s\n",
			dev_name(&mmc->class_dev),
			host->dma_dev ? "" : ", no DMA",
			(host->pdata && host->pdata->get_ro)
				? "" : ", no WP",
			(host->pdata && host->pdata->setpower)
				? "" : ", no poweroff",
			(mmc->caps & MMC_CAP_NEEDS_POLL)
				? ", cd polling" : "");*/
	return 0;

fail_add_host:
	mmc_remove_host (mmc);
fail_glue_init:
	/*if (host->dma_dev)
		dma_unmap_single(host->dma_dev, host->data_dma,
				sizeof(*host->data), DMA_BIDIRECTIONAL);
	kfree(host->data);
*/
fail_nobuf1:
	mmc_free_host(mmc);
	//mmc_spi_put_pdata(spi);
	dev_set_drvdata(&pdev->dev, NULL);

nomem:
	//kfree(ones);
	return status;
}


static int mmc_spi_remove(struct platform_device *pdev)
{
	struct alcemmc_platform_data *pdata = pdev->dev.platform_data;
	struct mmc_host 	*mmc = platform_get_drvdata(pdev);
	struct mmc_spi_host	*host;

	if (!mmc)
		return -1;

	if (mmc) {
		host = mmc_priv(mmc);
		
		/* prevent new mmc_detect_change() calls */
		INT_CTRL &= ~MMCCD_IEN;
		free_irq(pdata->irq, mmc);

		
		/*if (host->pdata && host->pdata->exit)
			host->pdata->exit(&pdev->dev, mmc);*/

		mmc_remove_host(mmc);

		/*if (host->dma_dev) {
			dma_unmap_single(host->dma_dev, host->ones_dma,
				MMC_SPI_BLOCKSIZE, DMA_TO_DEVICE);
			dma_unmap_single(host->dma_dev, host->data_dma,
				sizeof(*host->data), DMA_BIDIRECTIONAL);
		}

		kfree(host->data);
		kfree(host->ones);*/

		//spi->max_speed_hz = mmc->f_max;
		mmc_free_host(mmc);
		//mmc_spi_put_pdata(spi);
		dev_set_drvdata(&pdev->dev, NULL);
	}
	return 0;
}

static struct platform_driver mmc_spi_driver = {
	.probe		= mmc_spi_probe,
	.remove		= mmc_spi_remove,

	.driver		= {
		.name	= "alce_mmc",
		.owner	= THIS_MODULE,
	},
};


static int __init mmc_spi_init(void)
{
	return platform_driver_register(&mmc_spi_driver);
}


static void __exit mmc_spi_exit(void)
{
	platform_driver_unregister(&mmc_spi_driver);
}

module_init(mmc_spi_init);
module_exit(mmc_spi_exit);


MODULE_AUTHOR("Luis Alves");
MODULE_DESCRIPTION("Alcetronics SPI SD/MMC host driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("alce:mmc_spi");
