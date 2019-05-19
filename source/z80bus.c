/*****************************************************************************
**
** Msx Slot Access Code for Raspberry Pi 
** https://github.com/meesokim/msxslot
**
** RPMC(Raspberry Pi MSX Clone) core module
**
** Copyright (C) 2016 Miso Kim meeso.kim@gmail.com
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
**
******************************************************************************
*/

#define RPMC_V5
#include <bcm2835.h>
// Access from ARM Running Linux
#include "rpi-gpio.h"
   
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bcm2835.h>
#include <time.h>
#include <sched.h>
#include <unistd.h>
#include <pthread.h>

#include "barrier.h"
 
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
 
int  mem_fd;
void *gpio_map;
 
// I/O access
volatile unsigned *gpio;
volatile unsigned *gpio10;
volatile unsigned *gpio7;
volatile unsigned *gpio13;
volatile unsigned *gpio1;
volatile unsigned *gclk_base;

void msleep (unsigned int ms) {
    int microsecs;
    struct timeval tv;
    microsecs = ms * 1000;
    tv.tv_sec  = microsecs / 1000000;
    tv.tv_usec = microsecs % 1000000;
    select (0, NULL, NULL, NULL, &tv);  
}
 
 
// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET *(gpio7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio10) // clears bits which are 1 ignores bits which are 0
 
#define GET_GPIO(g) (*(gpio13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
#define GPIO (*(gpio13))
 
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

#define GZ_CLK_BUSY    (1 << 7)
#define GP_CLK0_CTL *(gclk_base + 0x1C)
#define GP_CLK0_DIV *(gclk_base + 0x1D)

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

pthread_mutex_t mutex;

static int setup_io();
int z80read(unsigned short addr);
void z80write(unsigned short addr, unsigned char byte);
int z80readio(unsigned short addr);
void z80writeio(unsigned short addr, unsigned char byte);
static void clear_io();
void setup_gclk();

static void SetAddress(unsigned short addr)
{
	GPIO_CLR = LE_C | 0xffff | DAT_DIR;
	GPIO_CLR = LE_C;
	GPIO_SET = LE_A | addr;
	GPIO_SET = LE_A;
    GPIO_CLR = LE_A;
	GPIO_SET = LE_C | Z80_CONTROLS;
	GPIO_CLR = LE_D | 0xff | DAT_DIR;
}	

static void SetDelay(int j, int wait)
{
	for(int i=0; i<j; i++)
	{
	    while(!(GPIO & Z80_CLK));
	    while(GPIO & Z80_CLK);
        if (wait && !(GPIO & Z80_WAIT))
        {
            i--;
        }
	}
}

static void SetData(int flag, int delay, unsigned char byte)
{
    GPIO_CLR = 0xff;
    GPIO_SET = byte;
    while(GPIO & Z80_CLK);
    GPIO_CLR = flag;
    GPIO_CLR = Z80_WR;
    SetDelay(1, 0);
    SetDelay(delay, 1);
    GPIO_SET = Z80_CONTROLS;
    SetDelay(1, 0);
    GPIO_CLR = LE_C;
    GPIO_SET = LE_D;
}   

static unsigned char raddr = 0;
static unsigned char GetData(int flag, int delay)
{
    unsigned char byte;
    GPIO_SET = DAT_DIR | 0xff;
    while(GPIO & Z80_CLK);
    GPIO_CLR = flag | Z80_RD;
    SetDelay(delay, 1);
    while(!(GPIO & Z80_CLK));
    byte = GPIO;
#if 1    
    if ((flag & Z80_M1) && (flag & Z80_MREQ))
    {
        GPIO_SET = LE_D | flag | Z80_RD;
        if (raddr > 127)
            raddr = 0;
        SetAddress(raddr++);
        GPIO_CLR = Z80_RFSH;
        while((GPIO & Z80_CLK));
        SetDelay(1, 0);
        GPIO_CLR = Z80_MREQ;
        SetDelay(1, 0);
        GPIO_SET = Z80_MREQ;
    }
    else
    {
        GPIO_SET = LE_D | flag | Z80_RD;
    }
#endif
    GPIO_SET = LE_D | Z80_CONTROLS;
    GPIO_CLR = LE_C;
    return byte;
}

int z80read(unsigned short addr)
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
	printf("+%04x:%02xw\n", addr, byte);
#endif
	return;
 }
 
 int z80readio(unsigned short addr)
 {
	unsigned char byte;
	SetAddress(addr);
	byte = GetData(Z80_IORQ, 3);
#ifdef DEBUG      
	printf("-IO%02x:%02xr\n", addr, byte);
#endif
	return byte;	 
 }
 
 void z80writeio(unsigned short addr, unsigned char byte)
   {
	SetAddress(addr);
	SetData(Z80_IORQ, 3, byte);
#ifdef DEBUG      
	printf("-IO%02x:%02xw\n", addr, byte);
#endif
	return;
 }
 
#if 0 
int rtapi_open_as_root(const char *filename, int mode) {
	fprintf (stderr, "euid: %d uid %d\n", geteuid(), getuid());
	seteuid(0);
	fprintf (stderr, "euid: %d uid %d\n", geteuid(), getuid());
	setfsuid(geteuid());
	int r = open(filename, mode);
	setfsuid(getuid());
	return r;
}
#endif
//
// Set up a memory regions to access GPIO
//
static int setup_io()
{
	int i, speed_id, divisor ;	
	if (!bcm2835_init()) return -1;
	gpio = bcm2835_regbase(BCM2835_REGBASE_GPIO);
	for(int i=0; i <= 27; i++)
	{
		bcm2835_gpio_fsel(i, 1);    
		bcm2835_gpio_set_pud(i, BCM2835_GPIO_PUD_DOWN);
	}
	bcm2835_gpio_fsel(CLK_PIN, 0);
	gpio10 = gpio+10;
	gpio7 = gpio+7;
	gpio13 = gpio+13;
	gpio1 = gpio+1;
	//SET_GPIO_ALT(20, 5);
#if 0	
	gclk_base = bcm2835_regbase(BCM2835_REGBASE_CLK);
	if (gclk_base != MAP_FAILED)
	{
		int divi, divr, divf, freq;
		bcm2835_gpio_fsel(20, BCM2835_GPIO_FSEL_ALT5); // GPIO_20
		speed_id = 1;
		freq = 3580000;
		divi = 19200000 / freq ;
		divr = 19200000 % freq ;
		divf = (int)((double)divr * 4096.0 / 19200000.0) ;
		if (divi > 4095)
			divi = 4095 ;		
		divisor = 1 < 12;// | (int)(6648/1024);
		GP_CLK0_CTL = 0x5A000000 | speed_id;    // GPCLK0 off
		while (GP_CLK0_CTL & 0x80);    // Wait for BUSY low
		GP_CLK0_DIV = 0x5A000000 | (divi << 12) | divf; // set DIVI
		usleep(10);
		GP_CLK0_CTL = 0x5A000010 | speed_id;    // GPCLK0 on
		printf("clock enabled: 0x%08x\n", GP_CLK0_CTL );
	}
	else
		printf("clock disabled\n");
#endif	
//	bcm2835_gpio_pud(BCM2835_GPIO_PUD_DOWN);
//	for(int i = 0; i < 8; i++)
		// bcm2835_gpio_pudclk(i, 1);
	// bcm2835_gpio_pudclk(27,1);
    // if (bcm2835_regbase(BCM2835_REGBASE_PADS) != MAP_FAILED)
            // bcm2835_gpio_set_pad(BCM2835_PAD_GROUP_GPIO_0_27, BCM2835_PAD_HYSTERESIS_ENABLED | 12); 	
    GPIO_CLR = Z80_RESET;
    msleep(100);        
	GPIO_SET = LE_C | Z80_CONTROLS | Z80_WAIT | Z80_INT | Z80_NMI | Z80_BUSRQ | Z80_RESET;
	GPIO_CLR = LE_C;
	SetAddress(0xffff);
	return 0;
} // setup_io

#if 0
#define EINVAL 0
int stick_this_thread_to_core(int core_id) {
   int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
   if (core_id < 0 || core_id >= num_cores)
      return EINVAL;

   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(core_id, &cpuset);
   return sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
}
#endif
static void clear_io()
{
//	spi_clear();
}

void z80init()
{
	const struct sched_param priority = {1};
	sched_setscheduler(0, SCHED_FIFO, &priority);  
//	stick_this_thread_to_core(0);
	if (setup_io() == -1)
    {
        printf("GPIO init error\n");
        exit(0);
    }
	printf("Z80 BUS initialized\n");
}

void z80reset()
{
    GPIO_CLR = Z80_RESET;
	msleep(200);
	GPIO_SET = Z80_RESET;
}
void z80close()
{
	clear_io();
}

#ifdef _MAIN


int main(int argc, char **argv)
{
    int g,rep,i,addr, page=4, c= 0,addr0;
    char byte, byte0, io;
    int offset = 0x4000;
    int size = 0x4000;
    FILE *fp = 0;
    struct timespec t1, t2;
    double elapsedTime = 0;
    int binary = 0;
    io = 0;
    int slot = 0;
    if (argc > 1)
    {
     if (strcmp(argv[1], "io"))
        fp = fopen(argv[1], "wb");
     else
        io = 1;
    }
    if (argc > 2)
    {
      offset = atoi(argv[2]);
    }
    if (argc > 3)
    {
      size = atoi(argv[3]);
    }
 
    // Set up gpi pointer for direct register access
    setup_io();
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);
	if (io > 0)
	{
		for(i = 0; i < 256; i++)
		{
			if (i % 16 == 0)
				printf("%02x: ", i);
			printf("%02x ", z80readio(0x98));
			if (i > 0 && i % 16 == 15)
				printf("\n");
		}
		exit(0);
	}
    z80writeio(0xa8, 0x0);
    z80write(0xffff,0x0);
    z80writeio(0xfc, 0x3);
    z80writeio(0xfd, 0x2);
    z80writeio(0xfe, 0x1);
    z80writeio(0xff, 0x0);
	offset = 0x4000;
	for(addr=offset; addr < offset + size; addr ++)
	{
        z80write(addr, addr);
    }
	for(addr=offset; addr < offset + size; addr ++)
	{
        byte = z80read(addr);
	  if (fp)
		  fwrite(&byte, 1, 1, fp);
	  else
	  {
#if 1		 
		if (addr % 16 == 0)
			 printf("\n%04x:", addr);
#if 1		  
		c = 0;
		for(i=0;i<10;i++)
		{
			byte0 = z80read(addr);
            if (byte0 != byte)
                c = 1;
			// if (byte != byte0)
                // printf("\e[31m%02x \e[0m", byte0);
            // else
                // printf("%02x ", byte);
		}
		if (c)  
			printf("\e[31m%02x \e[0m", byte);
		else
			printf("%02x ", byte);
#else
		printf("%02x ", byte);
#endif	
#endif
	  }
	}
	printf("\n");
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t2);
	elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000000.0;      // sec to ns
	elapsedTime += (t2.tv_nsec - t1.tv_nsec) ;   // us to ns	
	if (!binary) {
		printf("elapsed time: %10.2fs, %10.2fns/i\n", elapsedTime/100000000, elapsedTime / size);
	}	
  clear_io();
  return 0;
 
} // main
#endif
