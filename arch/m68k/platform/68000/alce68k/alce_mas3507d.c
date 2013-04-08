/*
 * MAS3507d simple device driver
 */

#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <asm/current.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/alce68k.h>


#define I2C_D	BYTE_REF(0xF00030)
#define I2C_C	BYTE_REF(0xF00031)

#define SPI_D	BYTE_REF(0xF00032)
#define SPI_C	BYTE_REF(0xF00033)

int my_open(struct inode *inode,struct file *filep);
int my_release(struct inode *inode,struct file *filep);
ssize_t my_read(struct file *filep,char *buff,size_t count,loff_t *offp );
ssize_t my_write(struct file *filep,const char *buff,size_t count,loff_t *offp );

void send_i2c(unsigned char *buf, unsigned char len);

struct file_operations my_fops={
	open: my_open,
	read: my_read,
	write: my_write,
	release:my_release,
};



struct mas_fifo {
  unsigned char *buf;
  u32 head;
  u32 tail;
  u32 size;
  u16 underrun;
  u32 max_usage;
};


struct mas_fifo fifo;



static irqreturn_t mas_interrupt(int irq, void *data)
{
  unsigned long flags;
  
  u32 len, tot;
  
  unsigned char *cp, *mas;
  
  if (fifo.tail == fifo.head) {
    fifo.underrun++;
    //printk("MAS3507d: Buffer underrun!\n");
    INT_CTRL &= ~MAS_IEN;
  } else {

    if (fifo.head > fifo.tail) {
      // head is ahead
      len = fifo.head - fifo.tail;
    } else {
      len = fifo.size - fifo.tail;
    }

    //printk("len: %d\n", len);
    
    cp = &fifo.buf[fifo.tail];
    tot = len;
    local_irq_save(flags);
    // send as fast as possible
    /*asm volatile ("1:\t"
		  "moveb (%1)+,0xf00032\n\t"
		  "subql #1,%0\n\t"
		  "beq   2f\n\t"
		  "btst  #6,0xf00033\n\t"
		  "bne   1b\n\t"
		  "2:"
		  : "+d" (len)
		  : "a" (cp), "d" (len)
		  :  );*/
    mas = (unsigned char*) 0xf00032;
    asm volatile ("1:\t"
		  "moveb (%1)+,(%2)\n\t"
		  "subql #1,%0\n\t"
		  "beq   2f\n\t"
		  "btst  #6,1(%2)\n\t"
		  "bne   1b\n\t"
		  "2:"
		  : "+d" (len)
		  : "a" (cp), "a" (mas), "d" (len)
		  :  );
    
    /*while ((SPI_C & 0x40) && len > 0) {

      //SPI_D = *cp++;//fifo.buf[p++];
      asm volatile ("moveb (%0)+,0xf00032\n\t"
		   "\n\t"
      
		    :: "a" (cp));
      len--;
    }*/

    
    local_irq_restore(flags);
    //printk("len: %d\n", len);
    fifo.tail += tot-len;
    
    //fifo.tail &= fifo.size-1;
    if (fifo.tail >= fifo.size)
	  fifo.tail = 0;
  }
  return IRQ_HANDLED;
}


void send_i2c(unsigned char *buf, unsigned char len)
{
  unsigned char i;
  
  while(I2C_C & 0x02);
  
  // send start
  I2C_C = 0x01;
  while(I2C_C & 0x02);
  for(i=0; i<len; i++) {
    I2C_D = *buf++;
    I2C_C = 0x18;
    while(I2C_C & 0x02);
  }

  // send stop
  I2C_C = 0x02;
  while(I2C_C & 0x02);
}

int my_open(struct inode *inode,struct file *filep)
{
	/*unsigned char i2c_init1[] = {0x3a,0x68,0x93,0xb0,0x00,0x02};
	unsigned char i2c_init2[] = {0x3a,0x68,0x00,0x01};
	unsigned long flags;*/

	// turn off irq
	/*INT_CTRL &= ~MAS_IEN;
	SPI_C = 0x00;*/

	printk("Port opened, spi enabled, filling buffer.\n");
	/*MOD_INC_USE_COUNT;*/ /* increments usage count of module */
	
	
	SPI_C = 0x22;
	
	
	/* allocate 256K for buffer */
	fifo.size = 0x40000;
	fifo.buf = kmalloc(fifo.size, GFP_KERNEL);
	if (!fifo.buf) {
	    printk("Error alocation memory for MAS3507d");
	    return -ENOMEM;
	}
	
	fifo.underrun = 0;
	fifo.max_usage = 0;
	fifo.head = 0;
	fifo.tail = 0;
	
	return 0;
}

int my_release(struct inode *inode,struct file *filep)
{
	/*MOD_DEC_USE_COUNT;*/ /* decrements usage count of module */
	printk("Total underruns: %d\n", fifo.underrun);
	printk("Fifo peak: %dK\n", (fifo.max_usage >> 10));

	while (INT_CTRL & MAS_IEN);
	//INT_CTRL &= ~MAS_IEN;
	kfree(fifo.buf);
	
	SPI_C = 0x00;
	
	return 0;
}
ssize_t my_read(struct file *filep,char *buff,size_t count,loff_t *offp )
{
	/* function to copy kernel space buffer to user space*/
	/*if ( copy_to_user(buff,my_data,strlen(my_data)) != 0 )
		printk( "Kernel -> userspace copy failed!\n" );*/
	return 0; //strlen(my_data);

}
ssize_t my_write(struct file *filep,const char *buff,size_t count,loff_t *offp )
{
  
	size_t free;
	
	
	//if (fifo.tail == fifo.head) {
	  // empty fifo
	  //free = fifo.size;
	//} else 
	if (fifo.tail > fifo.head) {
	  // tail is ahead
	  free = fifo.tail - fifo.head;
	} else {
	  free = fifo.size - (fifo.head - fifo.tail);
	}
	free--;

	//printk("free: %d\n", free);
	
	//fifo.max_usage = max(fifo.max_usage, free);
	
	if (INT_CTRL & MAS_IEN)
	    fifo.max_usage = max(fifo.max_usage, free);
	
	if (free == 0) {
	    //printk("MAS3507d: fifo at 100%%\n");
	    INT_CTRL |= MAS_IEN;
	    return 0;
	    //goto out;
	}
	
	if (count > free)
	    count = free;

	//printk("head: %d, tail: %d, count: %d, h+c: %d\n", fifo.head, fifo.tail, count, fifo.head + count);
	// overflow?
	/*if (fifo.head + count > fifo.size) {
	  count = fifo.size - fifo.head;
	  remain = free - count;
	  printk("MAS3507d: avoid overflow: sent=%d, remain=%d\n", count, remain);
	}*/

	//printk("count: %d\n", count);
	
	copy_from_user(&fifo.buf[fifo.head], buff, count);

	fifo.head += count;
	
	if (fifo.head >= fifo.size)
	    fifo.head = 0;
	
	return count;
	/*
out:
	return 0;*/
}



MODULE_AUTHOR("ljalves");
MODULE_DESCRIPTION("MAS3507d driver");

static int r_init(void);
static void r_cleanup(void);

module_init(r_init);
module_exit(r_cleanup);


static int __init r_init(void)
{
	unsigned char i2c_init1[] = {0x3a,0x68,0x93,0xb0,0x00,0x02};
	unsigned char i2c_init2[] = {0x3a,0x68,0x00,0x01};
	unsigned long flags;

	printk("MAS3507d linux driver 0.1\n");
	if(register_chrdev(200,"mas3507d",&my_fops)){
		printk("<1>failed to register");
	}


	// turn off irq
	INT_CTRL &= ~MAS_IEN;
	SPI_C = 0x00;

	//printk("Port opened, i2c sent, spi enabled.\n");
	/*MOD_INC_USE_COUNT;*/ /* increments usage count of module */

	local_irq_save(flags);
	send_i2c(i2c_init1, 6);
	send_i2c(i2c_init2, 4);
	local_irq_restore(flags);
  
	if (request_irq(INT_MAS, mas_interrupt, 0, "MAS3507d", NULL))
		printk(KERN_ERR "MAS3507d: unable to attach IRQ\n");

	return 0;
}
static void __exit r_cleanup(void)
{
	printk("MAS3507d: removed\n");
	unregister_chrdev(201,"mas3507d");
	free_irq(INT_MAS, NULL);
	return ;
}

