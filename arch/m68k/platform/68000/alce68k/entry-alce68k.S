/*
 *  linux/arch/m68k/platform/68000/entry.S
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License.  See the file README.legal in the main directory of this archive
 *  for more details.
 *
 *  2012.04.17, ljalvs@gmail.com, Initial version based on coldfire sources.
 */

#include <linux/linkage.h>
#include <asm/unistd.h>
#include <asm/thread_info.h>
#include <asm/errno.h>
#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/traps.h>
#include <asm/asm-offsets.h>
#include <asm/entry.h>

.text

.globl system_call
.globl resume
.globl ret_from_exception
.globl ret_from_signal
.globl sys_call_table
//.globl ret_from_interrupt
.globl bad_interrupt

.globl inthandler_vec1
.globl inthandler_vec2
.globl inthandler_vec3
.globl inthandler_vec4
.globl inthandler_vec5
.globl inthandler_vec6
.globl inthandler_vec7
.globl inthandler_vec8
.globl inthandler_vec9
.globl inthandler_vec10
.globl inthandler_vec11
.globl inthandler_vec12
.globl inthandler_vec13
.globl inthandler_vec14
.globl inthandler_vec15
.globl inthandler_vec16
.globl inthandler_vec17
.globl inthandler_vec18
.globl inthandler_vec19
.globl inthandler_vec20

.globl inthandler1
.globl inthandler2
.globl inthandler3
.globl inthandler4
.globl inthandler5
.globl inthandler6
.globl inthandler7

.globl trap_addrerr
.globl trap_zerodiv

.globl debug_btn


#if 0

badsys:
	movel	#-ENOSYS,%sp@(PT_OFF_D0)
	jra	ret_from_exception

do_trace:
	movel	#-ENOSYS,%sp@(PT_OFF_D0) /* needed for strace*/
	subql	#4,%sp
	SAVE_SWITCH_STACK
	jbsr	syscall_trace_enter
	RESTORE_SWITCH_STACK
	addql	#4,%sp
	movel	%sp@(PT_OFF_ORIG_D0),%d1
	movel	#-ENOSYS,%d0
	cmpl	#NR_syscalls,%d1
	jcc	1f
	lsl	#2,%d1
	lea	sys_call_table, %a0
	jbsr	%a0@(%d1)

1:	movel	%d0,%sp@(PT_OFF_D0)	/* save the return value */
	subql	#4,%sp			/* dummy return address */
	SAVE_SWITCH_STACK
	jbsr	syscall_trace_leave

ret_from_signal:
	RESTORE_SWITCH_STACK
	addql	#4,%sp
	jra	ret_from_exception

ENTRY(system_call)
	SAVE_ALL_SYS

	/* save top of frame*/
	pea	%sp@
	jbsr	set_esp0
	addql	#4,%sp

	movel	%sp@(PT_OFF_ORIG_D0),%d0

	movel	%sp,%d1			/* get thread_info pointer */
	andl	#-THREAD_SIZE,%d1
	movel	%d1,%a2
	btst	#(TIF_SYSCALL_TRACE%8),%a2@(TINFO_FLAGS+(31-TIF_SYSCALL_TRACE)/8)
	jne	do_trace
	cmpl	#NR_syscalls,%d0
	jcc	badsys
	lsl	#2,%d0
	lea	sys_call_table,%a0
	movel	%a0@(%d0), %a0
	jbsr	%a0@
	movel	%d0,%sp@(PT_OFF_D0)	/* save the return value*/

ret_from_exception:
	btst	#5,%sp@(PT_OFF_SR)	/* check if returning to kernel*/
	jeq	Luser_return		/* if so, skip resched, signals*/

Lkernel_return:
	RESTORE_ALL

Luser_return:
	/* only allow interrupts when we are really the last one on the*/
	/* kernel stack, otherwise stack overflow can occur during*/
	/* heavy interrupt load*/
	andw	#ALLOWINT,%sr

	movel	%sp,%d1			/* get thread_info pointer */
	andl	#-THREAD_SIZE,%d1
	movel	%d1,%a2
1:
	move	%a2@(TINFO_FLAGS),%d1	/* thread_info->flags */
	jne	Lwork_to_do
	RESTORE_ALL

Lwork_to_do:
	movel	%a2@(TINFO_FLAGS),%d1	/* thread_info->flags */
	btst	#TIF_NEED_RESCHED,%d1
	jne	reschedule

Lsignal_return:
	subql	#4,%sp			/* dummy return address*/
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	bsrw	do_notify_resume
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	addql	#4,%sp
	jra	1b

#endif

#if 1

ENTRY(system_call)
	SAVE_ALL_SYS
	//andw	#ALLOWINT,%sr		/* enable intrs again */

	cmpl	#NR_syscalls,%d0
	bcc	enosys
	lea	sys_call_table,%a0
	lsll	#2,%d0			/* movel %a0@(%d0:l:4),%d3 */
	movel	%a0@(%d0),%d3
	bne	1f
enosys:
	mov.l	#sys_ni_syscall,%d3
1:
	movel	%sp,%d2			/* get thread_info pointer */
	andl	#-THREAD_SIZE,%d2	/* at start of kernel stack */
	movel	%d2,%a0
	movel	%a0@,%a1		/* save top of frame */
	movel	%sp,%a1@(TASK_THREAD+THREAD_ESP0)
	btst	#(TIF_SYSCALL_TRACE%8),%a0@(TINFO_FLAGS+(31-TIF_SYSCALL_TRACE)/8)
	bnes	1f

	movel	%d3,%a0
	jsr	%a0@
	movel	%d0,%sp@(PT_OFF_D0)	/* save the return value */
	bra	ret_from_exception
1:
	movel	#-ENOSYS,%sp@(PT_OFF_D0) /* strace needs -ENOSYS in PT_OFF_D0 */
	subql	#4,%sp
	SAVE_SWITCH_STACK
	jsr	syscall_trace_enter
	RESTORE_SWITCH_STACK
	addql	#4,%sp
	movel	%d3,%a0
	jsr	%a0@
	movel	%d0,%sp@(PT_OFF_D0)	/* save the return value */
	subql	#4,%sp			/* dummy return address */
	SAVE_SWITCH_STACK
	jsr	syscall_trace_leave

ret_from_signal:
	RESTORE_SWITCH_STACK
	addql	#4,%sp

ret_from_exception:
	//oriw	#0x0700,%sr		/* disable intrs */
	btst	#5,%sp@(PT_OFF_SR)	/* check if returning to kernel */
	beq	Luser_return		/* if so, skip resched, signals */
	
Lkernel_return:
	RESTORE_ALL

Luser_return:
	movel	%sp,%d1			/* get thread_info pointer */
	andl	#-THREAD_SIZE,%d1	/* at base of kernel stack */
	movel	%d1,%a0
	moveb	%a0@(TINFO_FLAGS+3),%d1	/* thread_info->flags (low 8 bits) */
	bne	Lwork_to_do		/* still work to do */

Lreturn:
	RESTORE_ALL

Lwork_to_do:
	movel	%a0@(TINFO_FLAGS),%d1	/* get thread_info->flags */
	andw	#ALLOWINT,%sr		/* enable intrs again */
	btst	#TIF_NEED_RESCHED,%d1
	jne	reschedule

Lsignal_return:
	subql	#4,%sp			/* dummy return address */
	SAVE_SWITCH_STACK
	pea	%sp@(SWITCH_STACK_SIZE)
	jsr	do_notify_resume
	addql	#4,%sp
	RESTORE_SWITCH_STACK
	addql	#4,%sp
	bra	Luser_return

#endif
/*
 * Hardware interrupt handlers, responsible for calling do_IRQ()
 * The 68000 doesn't provide the vector number on the stack so individual
 * handlers need to provide different vectors to do_IRQ.
 */

/* TIMER 0 INTERRUPT */
inthandler_vec1:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#1,%sp@- 		/*  put vector # on stack */
	jsr	do_IRQ			/*  process the IRQ */
	addql	#8,%sp			/*  pop parameters off stack */
	bra	ret_from_exception

/* TIMER 1 INTERRUPT */
inthandler_vec2:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#2,%sp@- 		/*  put vector # on stack */
	jsr	do_IRQ			/*  process the IRQ */
	addql	#8,%sp			/*  pop parameters off stack */
	bra	ret_from_exception

/* SD Card insertion / removal detection INTERRUPT */
inthandler_vec3:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#3,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ */
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception


inthandler_vec4:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec5:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#5,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec6:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#6,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec7:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec8:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec9:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec10:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec11:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec12:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec13:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec14:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec15:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec16:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler_vec17:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#0x51,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

/* ENC28J60 INTERRUPT */
inthandler_vec18:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#18,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

/* FPGA UART INTERRUPT */
inthandler_vec19:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#19,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

/* MAS3507D INTERRUPT */
inthandler_vec20:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#20,%sp@- 		/*  put vector # on stack*/
	jsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception








/*
 * Autovector interrupt handlers
 */

inthandler1:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#65,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler2:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#66,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler3:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#67,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler4:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#68,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler5:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#69,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler6:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#70,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

inthandler7:
	SAVE_ALL_INT
	movel	%sp,%sp@-
	movel	#71,%sp@- 		/*  put vector # on stack*/
	jbsr	do_IRQ			/*  process the IRQ*/
	addql	#8,%sp			/*  pop parameters off stack*/
	bra	ret_from_exception

#if 0
ret_from_interrupt:

	bra	ret_from_exception
	beq	1f
2:
	RESTORE_ALL
1:
	#moveb	'1',0xE00000
	moveb	%sp@(PT_OFF_SR), %d0
	and	#7, %d0
	bhi	2b

	/* check if we need to do software interrupts */
	beq	ret_from_exception

	pea	ret_from_exception
	jmp	do_softirq
#endif

/*
 * Handler for uninitialized and spurious interrupts.
 */
ENTRY(bad_interrupt)
	addql	#1,irq_err_count
	rte




ENTRY(trap_addrerr)
	SAVE_ALL_INT
	GET_CURRENT(%d0)
	movel	%sp,%sp@- 		/* stack frame pointer argument */
	jsr	trap_c_addrerr
	addql	#4,%sp
	jra	ret_from_exception


ENTRY(trap_zerodiv)
	SAVE_ALL_INT
	GET_CURRENT(%d0)
	movel	%sp,%sp@- 		/* stack frame pointer argument */
	jsr	trap_c_zerodiv
	addql	#4,%sp
	jra	ret_from_exception












/*
 * Beware - when entering resume, prev (the current task) is
 * in a0, next (the new task) is in a1, so don't change these
 * registers until their contents are no longer needed.
 */
ENTRY(resume)
	movel	%a0,%d1				/* save prev thread in d1 */
	movew	%sr,%a0@(TASK_THREAD+THREAD_SR)	/* save sr */
	SAVE_SWITCH_STACK
	movel	%sp,%a0@(TASK_THREAD+THREAD_KSP) /* save kernel stack */
	movel	%usp,%a3			/* save usp */
	movel	%a3,%a0@(TASK_THREAD+THREAD_USP)

	movel	%a1@(TASK_THREAD+THREAD_USP),%a3 /* restore user stack */
	movel	%a3,%usp
	movel	%a1@(TASK_THREAD+THREAD_KSP),%sp /* restore new thread stack */
	movew	%a1@(TASK_THREAD+THREAD_SR),%sr	/* restore thread status reg */
	RESTORE_SWITCH_STACK
	rts

