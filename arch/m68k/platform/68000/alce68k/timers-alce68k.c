/***************************************************************************
 *
 *  linux/arch/m68k/platform/68000/timers-alce68k.c
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file COPYING in the main directory of this archive
 *  for more details.
 *
 *  2012.04.18, ljalvs@gmail.com, Created based on 68328/coldfire sources.
 *
 ***************************************************************************/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/profile.h>
#include <linux/clocksource.h>
#include <linux/rtc.h>
#include <asm/io.h>
#include <asm/traps.h>
#include <asm/machdep.h>

#include <asm/alce68k.h>

#define	FREQ	1000000

/***************************************************************************/

static u32 alce68k_tick_cnt;
static irq_handler_t timer_interrupt;

typedef volatile struct {
    volatile u16 cnt;
} tmr0_s;

tmr0_s *tmr0 = (tmr0_s *)TMR0_ADDR;


/***************************************************************************/

static irqreturn_t tmr0_tick(int irq, void *dummy)
{
	PCR0 = TMR0_RINT;
	alce68k_tick_cnt += 10000;

	return timer_interrupt(irq, dummy);
}

/***************************************************************************/

static struct irqaction alce68k_timer_irq = {
	.name	 = "timer",
	.flags	 = IRQF_DISABLED | IRQF_TIMER,
	.handler = tmr0_tick,
};

/***************************************************************************/

static cycle_t alce68k_read_clk(struct clocksource *cs)
{
	unsigned long flags;
	u32 cycles;
	u16 current_value;

	local_irq_save(flags);
	cycles = alce68k_tick_cnt;
	current_value = tmr0->cnt;
	local_irq_restore(flags);

	return cycles + current_value;
}

/***************************************************************************/

static struct clocksource alce68k_clk = {
	.name	= "tmr",
	.rating	= 250,
	.read	= alce68k_read_clk,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

/***************************************************************************/


void hw_timer_init(irq_handler_t handler)
{
	/* disable timer 0 */
	PCR1 &= ~TMR0_EN;
	
	/* set preset value */
	/* overflow at 10000 ticks = 10ms @ 1MHz */
	//TMR0 = 0xFFFF - 9999;
	//TMR0 = 0xFFFF - 9999;
	TMR0 = 9999;

	/* register clock */
	clocksource_register_hz(&alce68k_clk, FREQ);
	timer_interrupt = handler;

	/* set ISR */
	setup_irq(INT_TIMER0, &alce68k_timer_irq);

	/* enable timer0 */
	PCR1 |= TMR0_EN;
}



/***************************************************************************/

/*void alce68k_hwclk(int set, struct rtc_time *t)
{
	if (!set) {
		//long now = RTCTIME;
		t->tm_year = RTC_YEA;
		t->tm_mon = RTC_MON;
		t->tm_mday = RTC_DAY;
		t->tm_hour = RTC_HOU; //(now >> 24) % 24;
		t->tm_min = RTC_MIN; //(now >> 16) % 60;
		t->tm_sec = RTC_SEC; //now % 60;
	}

	return 0;
}*/

/***************************************************************************/
