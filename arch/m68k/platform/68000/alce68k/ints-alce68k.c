/***************************************************************************
 *
 *  linux/arch/m68k/platform/68000/ints-alce68k.c
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
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/traps.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/siginfo.h>
#include <linux/kallsyms.h>
#include <linux/signal.h>
#include <linux/ptrace.h>

#include <asm/alce68k.h>

/* assembler routines */
asmlinkage void system_call(void);
asmlinkage void buserr(void);
asmlinkage void trap(void);
asmlinkage void trap3(void);
asmlinkage void trap4(void);
asmlinkage void trap5(void);
asmlinkage void trap6(void);
asmlinkage void trap7(void);
asmlinkage void trap8(void);
asmlinkage void trap9(void);
asmlinkage void trap10(void);
asmlinkage void trap11(void);
asmlinkage void trap12(void);
asmlinkage void trap13(void);
asmlinkage void trap14(void);
asmlinkage void trap15(void);
asmlinkage void trap33(void);
asmlinkage void trap34(void);
asmlinkage void trap35(void);
asmlinkage void trap36(void);
asmlinkage void trap37(void);
asmlinkage void trap38(void);
asmlinkage void trap39(void);
asmlinkage void trap40(void);
asmlinkage void trap41(void);
asmlinkage void trap42(void);
asmlinkage void trap43(void);
asmlinkage void trap44(void);
asmlinkage void trap45(void);
asmlinkage void trap46(void);
asmlinkage void trap47(void);
asmlinkage irqreturn_t bad_interrupt(int, void *);
asmlinkage irqreturn_t inthandler1(void);
asmlinkage irqreturn_t inthandler2(void);
asmlinkage irqreturn_t inthandler3(void);
asmlinkage irqreturn_t inthandler4(void);
asmlinkage irqreturn_t inthandler5(void);
asmlinkage irqreturn_t inthandler6(void);
asmlinkage irqreturn_t inthandler7(void);

asmlinkage irqreturn_t inthandler_vec1(void);
asmlinkage irqreturn_t inthandler_vec2(void);
asmlinkage irqreturn_t inthandler_vec3(void);
asmlinkage irqreturn_t inthandler_vec4(void);
asmlinkage irqreturn_t inthandler_vec5(void);
asmlinkage irqreturn_t inthandler_vec6(void);
asmlinkage irqreturn_t inthandler_vec7(void);
asmlinkage irqreturn_t inthandler_vec8(void);
asmlinkage irqreturn_t inthandler_vec9(void);
asmlinkage irqreturn_t inthandler_vec10(void);
asmlinkage irqreturn_t inthandler_vec11(void);
asmlinkage irqreturn_t inthandler_vec12(void);
asmlinkage irqreturn_t inthandler_vec13(void);
asmlinkage irqreturn_t inthandler_vec14(void);
asmlinkage irqreturn_t inthandler_vec15(void);
asmlinkage irqreturn_t inthandler_vec16(void);
asmlinkage irqreturn_t inthandler_vec17(void);
asmlinkage irqreturn_t inthandler_vec18(void);
asmlinkage irqreturn_t inthandler_vec19(void);
asmlinkage irqreturn_t inthandler_vec20(void);


/*
 * The 68000 desn't provide the trap vector
 * on the stack. Each trap need its own callback.
 */

asmlinkage void trap_addrerr(void);
asmlinkage void trap_c_addrerr(struct frame *fp);
asmlinkage void trap_zerodiv(void);
asmlinkage void trap_c_zerodiv(struct frame *fp);


//asmlinkage void trap_line111(void);



asmlinkage irqreturn_t debug_btn(void);




extern e_vector *_ramvec;


static void intc_irq_enable(struct irq_data *d)
{
}

static void intc_irq_disable(struct irq_data *d)
{
}

static struct irq_chip intc_irq_chip = {
	.name		= "ALCE68K-INTC",
	.irq_enable	= intc_irq_enable,
	.irq_disable	= intc_irq_disable,
};


void __init trap_init(void)
{
	int i;

	for (i = 3; (i <= 23); i++)
		_ramvec[i] = (e_vector) trap;
	

	//_ramvec[4] = (e_vector) trap_addrerr;
	//_ramvec[10] = (e_vector) trap_addrerr;
	//_ramvec[11] = (e_vector) trap_zerodiv;
		
		

	/* set spurious and autovector interrupts to bad_interrupt */
	for (i = VEC_SPUR; (i <= VEC_INT7); i++)
		_ramvec[i] = (e_vector) bad_interrupt;

	/* unused traps */
	for (i = 33; (i <= 63); i++)
		_ramvec[i] = (e_vector) trap;
		
	/* set all user vectors to bad_interrupt */
	for (i = 64; i < 256; ++i)
		_ramvec[i] = (e_vector) bad_interrupt;



	_ramvec[VEC_BUSERR] 		= (e_vector) buserr;
	_ramvec[VEC_SYS] 		= (e_vector) system_call;
	_ramvec[VEC_ADDRERR]		= (e_vector) trap_addrerr;
	_ramvec[VEC_ZERODIV]		= (e_vector) trap_zerodiv;



	//_ramvec[VEC_INT7] 		= (e_vector) debug_btn; // int7 - nmi


	_ramvec[INTVEC_TIMER0] 		= (e_vector) inthandler_vec1; // timer0
	_ramvec[INTVEC_TIMER1] 		= (e_vector) inthandler_vec2; // timer1
	_ramvec[INTVEC_MMCCD] 		= (e_vector) inthandler_vec3;
	//_ramvec[67] = (e_vector) inthandler_vec4;
	_ramvec[INTVEC_FTDI] 		= (e_vector) inthandler_vec5; // ftdi
	//_ramvec[69] = (e_vector) inthandler_vec6;
	//_ramvec[70] = (e_vector) inthandler_vec7;
	//_ramvec[71] = (e_vector) inthandler_vec8;
	//_ramvec[72] = (e_vector) inthandler_vec9;
	//_ramvec[73] = (e_vector) inthandler_vec10;
	//_ramvec[74] = (e_vector) inthandler_vec11;
	//_ramvec[75] = (e_vector) inthandler_vec12;
	//_ramvec[76] = (e_vector) inthandler_vec13;
	//_ramvec[77] = (e_vector) inthandler_vec14;
	//_ramvec[78] = (e_vector) inthandler_vec15;
	//_ramvec[79] = (e_vector) inthandler_vec16;
	_ramvec[INTVEC_RTC] 		= (e_vector) inthandler_vec17; // RTC
	_ramvec[INTVEC_ENC28J60] 	= (e_vector) inthandler_vec18; // enc28j60
	_ramvec[INTVEC_FPGAUART] 	= (e_vector) inthandler_vec19; // fpga uart

	_ramvec[INTVEC_MAS] 	= (e_vector) inthandler_vec20; // mas3507d
	
}

void __init init_IRQ(void)
{
	int irq;
	
	for (irq = 0; (irq < NR_IRQS); irq++) {
		irq_set_chip_and_handler(irq, &intc_irq_chip, handle_simple_irq);
	}
}





/* TODO: fix the trap callback code */



asmlinkage void trap_c_addrerr(struct frame *fp)
{
	siginfo_t info;

	printk("address error!\n");

	/* send the appropriate signal to the user program */
	info.si_code = BUS_ADRALN;
	info.si_signo = SIGBUS;
	info.si_errno = 0;
	switch (fp->ptregs.format) {
	    default:
		info.si_addr = (void *) fp->ptregs.pc;
		break;
	    case 2:
		info.si_addr = (void *) fp->un.fmt2.iaddr;
		break;
	    case 7:
		info.si_addr = (void *) fp->un.fmt7.effaddr;
		break;
	    case 9:
		info.si_addr = (void *) fp->un.fmt9.iaddr;
		break;
	    case 10:
		info.si_addr = (void *) fp->un.fmta.daddr;
		break;
	    case 11:
		info.si_addr = (void *) fp->un.fmtb.daddr;
		break;
	}
	force_sig_info (SIGBUS, &info, current);
}


asmlinkage void trap_c_zerodiv(struct frame *fp)
{
	siginfo_t info;

	printk("div0!\n");

	/* send the appropriate signal to the user program */

	info.si_code = FPE_INTDIV;
	info.si_code = BUS_ADRALN;
	info.si_signo = SIGFPE;
	info.si_errno = 0;
	switch (fp->ptregs.format) {
	    default:
		info.si_addr = (void *) fp->ptregs.pc;
		break;
	    case 2:
		info.si_addr = (void *) fp->un.fmt2.iaddr;
		break;
	    case 7:
		info.si_addr = (void *) fp->un.fmt7.effaddr;
		break;
	    case 9:
		info.si_addr = (void *) fp->un.fmt9.iaddr;
		break;
	    case 10:
		info.si_addr = (void *) fp->un.fmta.daddr;
		break;
	    case 11:
		info.si_addr = (void *) fp->un.fmtb.daddr;
		break;
	}
	force_sig_info (SIGFPE, &info, current);
}

/*

asmlinkage void trap_c_(struct frame *fp)
{
	int sig;
	int vector = (fp->ptregs.vector >> 2) & 0xff;
	siginfo_t info;

	printk("address error!\n");

	if (fp->ptregs.sr & PS_S) {
		if (vector == VEC_TRACE) {
			/
		} else if (!handle_kernel_fault(&fp->ptregs))
			bad_super_trap(fp);
		return;
	}

	
	switch (vector) {
	    case VEC_ADDRERR:
		info.si_code = BUS_ADRALN;
		sig = SIGBUS;
		break;
	    case VEC_ILLEGAL:
	    case VEC_LINE10:
	    case VEC_LINE11:
		info.si_code = ILL_ILLOPC;
		sig = SIGILL;
		break;
	    case VEC_PRIV:
		info.si_code = ILL_PRVOPC;
		sig = SIGILL;
		break;
	    case VEC_COPROC:
		info.si_code = ILL_COPROC;
		sig = SIGILL;
		break;
	    case VEC_TRAP1:
	    case VEC_TRAP2:
	    case VEC_TRAP3:
	    case VEC_TRAP4:
	    case VEC_TRAP5:
	    case VEC_TRAP6:
	    case VEC_TRAP7:
	    case VEC_TRAP8:
	    case VEC_TRAP9:
	    case VEC_TRAP10:
	    case VEC_TRAP11:
	    case VEC_TRAP12:
	    case VEC_TRAP13:
	    case VEC_TRAP14:
		info.si_code = ILL_ILLTRP;
		sig = SIGILL;
		break;
	    case VEC_FPBRUC:
	    case VEC_FPOE:
	    case VEC_FPNAN:
		info.si_code = FPE_FLTINV;
		sig = SIGFPE;
		break;
	    case VEC_FPIR:
		info.si_code = FPE_FLTRES;
		sig = SIGFPE;
		break;
	    case VEC_FPDIVZ:
		info.si_code = FPE_FLTDIV;
		sig = SIGFPE;
		break;
	    case VEC_FPUNDER:
		info.si_code = FPE_FLTUND;
		sig = SIGFPE;
		break;
	    case VEC_FPOVER:
		info.si_code = FPE_FLTOVF;
		sig = SIGFPE;
		break;
	    case VEC_ZERODIV:
		info.si_code = FPE_INTDIV;
		sig = SIGFPE;
		break;
	    case VEC_CHK:
	    case VEC_TRAP:
		info.si_code = FPE_INTOVF;
		sig = SIGFPE;
		break;
	    case VEC_TRACE:		
		info.si_code = TRAP_TRACE;
		sig = SIGTRAP;
		break;
	    case VEC_TRAP15:		
		info.si_code = TRAP_BRKPT;
		sig = SIGTRAP;
		break;
	    default:
		info.si_code = ILL_ILLOPC;
		sig = SIGILL;
		break;
	}
	info.si_signo = sig;
	info.si_errno = 0;
	switch (fp->ptregs.format) {
	    default:
		info.si_addr = (void *) fp->ptregs.pc;
		break;
	    case 2:
		info.si_addr = (void *) fp->un.fmt2.iaddr;
		break;
	    case 7:
		info.si_addr = (void *) fp->un.fmt7.effaddr;
		break;
	    case 9:
		info.si_addr = (void *) fp->un.fmt9.iaddr;
		break;
	    case 10:
		info.si_addr = (void *) fp->un.fmta.daddr;
		break;
	    case 11:
		info.si_addr = (void *) fp->un.fmtb.daddr;
		break;
	}
	force_sig_info (sig, &info, current);
}
*/
