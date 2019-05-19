#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/cdev.h> 
#include <linux/uaccess.h>

//Tested with kernel 4.9.79

#define RD0		0
#define RD1		1
#define RD2		2
#define RD3		3
#define RD4		4
#define RD5		5
#define RD6		6
#define RD7		7
#define RA8		8
#define RA9		9
#define RA10	10
#define RA11	11
#define RA12	12
#define RA13	13
#define RA14	14
#define RA15	15
#define RC16	16
#define RC17	17
#define RC18	18
#define RC19	19
#define RC20	20
#define RC21	21
#define RC22	22
#define RC23	23
#define RC24	24
#define RC25	25
#define RC26	26
#define RC27	27

#define BUSACK_PIN	RA8
#define HALT_PIN	RA9
#define RFSH_PIN	RA10
#define M1_PIN	 	RA11
#define RD_PIN		RA12
#define WR_PIN		RA13
#define IORQ_PIN	RA14
#define MREQ_PIN	RA15
#define LE_A_PIN	RC16
#define LE_C_PIN	RC17
//#define LE_D_PIN	RC18
//#define RESET_PIN	RC19
#define LE_D_PIN	RC20
#define DAT_DIR_PIN	RC21
#define BUSRQ_PIN	RC22
#define NMI_PIN		RC23
#define INT_PIN		RC24
#define WAIT_PIN	RC25
#define RESET_PIN	RC26
#define CLK_PIN		RC27

#define Z80_RD		(1 << RD_PIN)
#define Z80_WR  	(1 << WR_PIN)
#define Z80_IORQ	(1 << IORQ_PIN)
#define Z80_MREQ	(1 << MREQ_PIN)
#define Z80_RESET   (1 << RESET_PIN)
#define Z80_WAIT	(1 << WAIT_PIN)
#define Z80_INT		(1 << INT_PIN)
#define LE_A	    (1 << LE_A_PIN)
#define LE_C		(1 << LE_C_PIN)
#define LE_D		(1 << LE_D_PIN)
#define Z80_CLK 	(1 << CLK_PIN)
#define DAT_DIR 	(1 << DAT_DIR_PIN)
#define Z80_RESET	(1 << RESET_PIN)
#define Z80_NMI		(1 << NMI_PIN)
#define Z80_HALT	(1 << HALT_PIN)
#define Z80_BUSACK	(1 << BUSACK_PIN)
#define Z80_M1		(1 << M1_PIN)
#define Z80_RFSH	(1 << RFSH_PIN)
#define Z80_BUSRQ	(1 << BUSRQ_PIN)

#define Z80_CONTROLS (Z80_RD | Z80_WR | Z80_IORQ | Z80_MREQ | Z80_HALT | Z80_BUSACK | Z80_M1 | Z80_RFSH)

#define GPIO_SET *(gpio7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio10) // clears bits which are 1 ignores bits which are 0 
#define GPIO (*(gpio13))

static unsigned int *gpio7;
static unsigned int *gpio10;
static unsigned int *gpio13;

MODULE_LICENSE("Dual BSD/GPL");

/* Declaration of memory.c functions */
int z80t_open(struct inode *inode, struct file *filp);
int z80t_release(struct inode *inode, struct file *filp);
ssize_t z80t_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
ssize_t z80t_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
long z80t_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

/* Structure that declares the usual file */
/* access functions */
struct file_operations z80t_fops = {
  owner: THIS_MODULE,  
  read: z80t_read,
  write: z80t_write,
  open: z80t_open,
  release: z80t_release,
  unlocked_ioctl: z80t_ioctl
};

struct GpioRegisters
{
    uint32_t GPFSEL[6];
    uint32_t Reserved1;
    uint32_t GPSET[2];
    uint32_t Reserved2;
    uint32_t GPCLR[2];
    uint32_t Reserved3;
    uint32_t GPLEV[2];
};

struct GpioRegisters *s_pGpioRegisters;

static void SetGPIOFunction(int gpio, int functionCode)
{
    int registerIndex = gpio / 10;
    int bit = (gpio % 10) * 3;

    unsigned oldValue = s_pGpioRegisters->GPFSEL[registerIndex];
    unsigned mask = 0b111 << bit;
    printk("Changing function of GPIO%d from %x to %x\n", gpio, (oldValue >> bit) & 0b111, functionCode);
    s_pGpioRegisters->GPFSEL[registerIndex] = (oldValue & ~mask) | ((functionCode << bit) & mask);
}

// static void SetGPIOOutputValue(int GPIO, bool outputValue)
// {
    // if (outputValue)
        // s_pGpioRegisters->GPSET[GPIO / 32] = (1 << (GPIO % 32));
    // else
        // s_pGpioRegisters->GPCLR[GPIO / 32] = (1 << (GPIO % 32));
// }

void SetAddress(unsigned short addr)
{
	GPIO_CLR = LE_C;
	GPIO_CLR = LE_C | 0xffff | DAT_DIR;
	GPIO_SET = LE_A | LE_D | addr;
	GPIO_SET = LE_A;
    GPIO_CLR = LE_A;
	GPIO_SET = LE_C | Z80_CONTROLS;
	GPIO_CLR = LE_D | 0xff;
}	

void SetDelay(int j)
{
	for(int i=0; i<j; i++)
	{
	    while(!(GPIO & Z80_CLK));
	    while(GPIO & Z80_CLK);
	}
}

void SetData(int flag, int delay, unsigned char byte)
{
	GPIO_SET = byte;
	GPIO_CLR = flag | Z80_WR;
	while(!(GPIO & Z80_WAIT));
    SetDelay(delay);
   	GPIO_SET = Z80_CONTROLS;
    GPIO_CLR = Z80_RFSH;
    SetDelay(1);
    GPIO_SET = Z80_CONTROLS;
	GPIO_CLR = LE_C;

}   

unsigned char GetData(int flag, int delay)
{
	unsigned char byte;
	GPIO_SET = DAT_DIR | 0xff;
	while(GPIO & Z80_CLK);
	GPIO_CLR = flag | Z80_RD;
	SetDelay(delay);
	while(!(GPIO & Z80_WAIT));
	byte = GPIO;
  	GPIO_SET = LE_D | Z80_CONTROLS;
	while(!(GPIO & Z80_CLK));
    GPIO_CLR = Z80_RFSH;
    SetDelay(1);
    GPIO_SET = Z80_CONTROLS;
	GPIO_CLR = LE_C;
	return byte;
}

char z80read(unsigned short addr)
{
	unsigned char byte;
	SetAddress(addr);
	byte = GetData(Z80_MREQ, 1);
#ifdef DEBUG    
	printf("+%04x:%02xr\n", addr, byte);
#endif
	return byte;	 
}
 
void z80write(unsigned short addr, unsigned char byte)
{
    SetAddress(addr);
    SetData(Z80_MREQ, 2, byte);
    #ifdef DEBUG  
    printk("+%04x:%02xw\n", addr, byte);
    #endif
    return;
}
 
char z80readio(unsigned short addr)
{
	unsigned char byte;
	SetAddress(addr);
	byte = GetData(Z80_IORQ, 2);
#ifdef DEBUG      
	printk("-IO%02x:%02xr\n", addr, byte);
#endif
	return byte;	 
}
 
void z80writeio(unsigned short addr, unsigned char byte)
{
	SetAddress(addr);
	SetData(Z80_IORQ, 3, byte);
#ifdef DEBUG      
	printk("-IO%02x:%02xw\n", addr, byte);
#endif
	return;
}


// static struct timer_list s_BlinkTimer;
// static int s_BlinkPeriod = 1000;
// static const int LedGpioPin = 18;

// static void BlinkTimerHandler(unsigned long unused)
// {
    // static bool on = false;
    // on = !on;
    // SetGPIOOutputValue(LedGpioPin, on);
    // mod_timer(&s_BlinkTimer, jiffies + msecs_to_jiffies(s_BlinkPeriod));
// }

// static ssize_t set_period_callback(struct device* dev,
    // struct device_attribute* attr,
    // const char* buf,
    // size_t count)
// {
    // long period_value = 0;
    // if (kstrtol(buf, 10, &period_value) < 0)
        // return -EINVAL;
    // if (period_value < 10)	//Safety check
    	// return - EINVAL;

    // s_BlinkPeriod = period_value;
    // return count;
// }

/* Global variables of the driver */
/* Major number */
int z80t_major = 60;
/* Buffer to store data */
char z80t_mem[65536];

int z80t_open(struct inode *inode, struct file *filp) 
{

  /* Success */
  return 0;
}
int z80t_release(struct inode *inode, struct file *filp) {
 
  /* Success */
  return 0;
}

#if 0    
ssize_t z80t_read(struct file *filp, char *buf, 
                    size_t count, loff_t *f_pos) { 

    int error_count = 0;
    /* Transfering data to user space */ 
    if (!f_pos)
    {
        if (count + *f_pos > 65535)
            count = 65536 - *f_pos;
        unsigned short eaddr = *f_pos + count;
        for(unsigned short addr = *f_pos; addr < eaddr; addr++)
        {
            z80t_mem[addr] = (char) (z80read(addr));
        }
        error_count = copy_to_user(buf,z80t_mem+*f_pos,count);
        if (!error_count)
            printk(KERN_INFO "Z80Turbo: Sent %d characters to the user\n", count);
            return 0;
    }
    return -EFAULT;
}
#else    
static const char    g_s_Hello_World_string[] = "Hello world from kernel mode!\n\0";
static const ssize_t g_s_Hello_World_size = sizeof(g_s_Hello_World_string);
ssize_t z80t_read(
                        struct file *file_ptr
                       , char __user *user_buffer
                       , size_t count
                       , loff_t *position)
{
    printk( KERN_NOTICE "Simple-driver: Device file is read at offset = %i, read bytes count = %u"
                , (int)*position
                , (unsigned int)count );
    /* If position is behind the end of a file we have nothing to read */
    if( *position >= g_s_Hello_World_size )
        return 0;
    /* If a user tries to read more than we have, read only as many bytes as we have */
    if( *position + count > g_s_Hello_World_size )
        count = g_s_Hello_World_size - *position;
    if( copy_to_user(user_buffer, g_s_Hello_World_string + *position, count) != 0 )
        return -EFAULT;    
    /* Move reading position */
    *position += count;
    return count;
}
#endif

ssize_t z80t_write( struct file *filp, const char *buf,
                      size_t count, loff_t *f_pos) {
    int error_count = 0;
    if (!f_pos)
    {
        error_count = copy_from_user(z80t_mem,buf,count);
        if (count + *f_pos > 65535)
            count = 65536 - *f_pos;
        unsigned short eaddr = *f_pos + count;
        for(unsigned short addr = *f_pos; addr < eaddr; addr++)
        {
            z80write(addr, (unsigned char) (z80t_mem[addr]));
        }
        return count;
    }
    return 0;
}

struct iowrite{
    unsigned short addr;
    unsigned char value;
};

typedef struct iowrite iow; 
long z80t_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    iow *s = (iow *) arg;
    switch(cmd)
    {
        case 0:
            s->value = z80readio(s->addr);
            break;
        case 1:
            s = (iow *) arg;
            z80write(s->addr, s->value);
            break;
        default:
            ret = -1;
    };
    return ret;
}

//static DEVICE_ATTR(period, S_IRWXU | S_IRWXG, NULL, set_period_callback);

static struct class *s_pDeviceClass;
static struct device *s_pDeviceObject;

#define PERIPH_BASE 0x3f000000
#define GPIO_BASE (PERIPH_BASE + 0x200000)

#define GPIO_SET *(gpio7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio10) // clears bits which are 1 ignores bits which are 0
#define GPIO (*(gpio13))

#define DEVICE_NAME "z80t"

static int __init Z80turboModule_init(void)
{
    int result;
    unsigned int *gpio;
    s_pGpioRegisters = (struct GpioRegisters *)ioremap(GPIO_BASE, sizeof(struct GpioRegisters));
    gpio = (unsigned int *)s_pGpioRegisters;
//    SetGPIOFunction(LedGpioPin, 0b001);  	//Configure the pin as output
    for(int i = 0; i < 28; i++)
    {
        SetGPIOFunction(i, 0b001);
    }
	gpio10 = &s_pGpioRegisters->GPCLR[0];//gpio + 10;
    gpio13 = &s_pGpioRegisters->GPLEV[0];//gpio + 13;
    gpio7  = &s_pGpioRegisters->GPSET[0];//gpio + 7;
//    setup_timer(&s_BlinkTimer, BlinkTimerHandler, 0);
//    result = mod_timer(&s_BlinkTimer, jiffies + msecs_to_jiffies(s_BlinkPeriod));
    memset(z80t_mem, 0, sizeof(z80t_mem));
    int ret, id;    
    struct cdev cdev;           
    ret = alloc_chrdev_region( &id, 0, 1, DEVICE_NAME );    
    if ( ret ){    
        printk( "alloc_chrdev_region error %d\n", ret );    
        return ret;    
    }    
        
    cdev_init( &cdev, &z80t_fops );    
    cdev.owner = THIS_MODULE;    
        
    ret = cdev_add( &cdev, id, 1 );    
    if (ret){    
        printk( "cdev_add error %d\n", ret );    
        unregister_chrdev_region( id, 1 );    
        return ret;    
    }
    
    s_pDeviceClass = class_create(THIS_MODULE, DEVICE_NAME);
    BUG_ON(IS_ERR(s_pDeviceClass));

    s_pDeviceObject = device_create(s_pDeviceClass, NULL, id, NULL, DEVICE_NAME);
    BUG_ON(IS_ERR(s_pDeviceObject));

//    result = device_create_file(s_pDeviceObject, &dev_attr_period);
    /* Registering device */
//    result = register_chrdev(z80t_major, DEVICE_NAME, &z80t_fops);    
    //BUG_ON(result < 0);
    return 0;
}

static void __exit Z80turboModule_exit(void)
{
    iounmap(s_pGpioRegisters);
//    device_remove_file(s_pDeviceObject, &dev_attr_period);
    device_destroy(s_pDeviceClass, 0);
    class_destroy(s_pDeviceClass);

//    del_timer(&s_BlinkTimer);
}

module_init(Z80turboModule_init);
module_exit(Z80turboModule_exit);