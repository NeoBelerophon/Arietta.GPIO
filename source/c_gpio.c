/*
Copyright (c) 2012-2013 Ben Croston

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "c_gpio.h"

#define SAM9G25_PERI_BASE   0xF0000000
#define GPIO_BASE           (SAM9G25_PERI_BASE + 0xFFFF400)

// GPIO Offsets
#define PIO_PER 	0x0000 //PIO Enable Register 						Write-only
#define PIO_PDR 	0x0004 //PIO Disable Register 						Write-only
#define PIO_PSR 	0x0008 //PIO Status Register  						Read-only
#define PIO_OER 	0x0010 //Output Enable Register 					Write-only
#define PIO_ODR 	0x0014 //Output Disable Register 					Write-only
#define PIO_OSR 	0x0018 //Output Status Register 					Read-only
#define PIO_IFER 	0x0020 //Glitch Input Filter Enable Register 		Write-only
#define PIO_IFDR 	0x0024 //Glitch Input Filter Disable Register 		Write-only
#define PIO_IFSR 	0x0028 //Glitch Input Filter Status Register 		Read-only
#define PIO_SODR 	0x0030 //Set Output Data Register 					Write-only
#define PIO_CODR 	0x0034 //Clear Output Data Register 				Write-only
#define PIO_ODSR 	0x0038 //Output Data Status Register 				Read-only
#define PIO_PDSR 	0x003C //Pin Data Status Register 					Read-only
#define PIO_IER 	0x0040 //Interrupt Enable Register 					Write-only
#define PIO_IDR 	0x0044 //Interrupt Disable Register 				Write-only
#define PIO_IMR 	0x0048 //Interrupt Mask Register 					Read-only
#define PIO_ISR 	0x004C //Interrupt Status Register 					Read-only
#define PIO_MDER 	0x0050 //Multi-driver Enable Register 				Write-only
#define PIO_MDDR 	0x0054 //Multi-driver Disable Register 				Write-only
#define PIO_MDSR 	0x0058 //Multi-driver Status Register 				Read-only
#define PIO_PUDR 	0x0060 //Pull-up Disable Register 					Write-only
#define PIO_PUER 	0x0064 //Pull-up Enable Register 					Write-only
#define PIO_PUSR 	0x0068 //Pad Pull-up Status Register 				Read-only
#define PIO_ABCDSR1 0x0070 //Peripheral Select Register 1 				Read-write
#define PIO_ABCDSR2 0x0074 //Peripheral Select Register 2 				Read-write
#define PIO_IFSCDR 	0x0080 //Input Filter Slow Clock Disable Register 	Write-only
#define PIO_IFSCER 	0x0084 //Input Filter Slow Clock Enable Register 	Write-only
#define PIO_IFSCSR 	0x0088 	//Input Filter Slow Clock Status Register 		Read-only
#define PIO_SCDR 	0x008C 	//Slow Clock Divider Debouncing Register 		Read-write
#define PIO_PPDDR 	0x0090 	//Pad Pull-down Disable Register 				Write-only
#define PIO_PPDER 	0x0094 	//Pad Pull-down Enable Register 				Write-only
#define PIO_PPDSR 	0x0098 	//Pad Pull-down Status Register 				Read-only
#define PIO_OWER 	0x00A0 	//Output Write Enable 							Write-only
#define PIO_OWDR 	0x00A4 	//Output Write Disable 							Write-only
#define PIO_OWSR 	0x00A8 	//Output Write Status Register 					Read-only
#define PIO_AIMER 	0x00B0 	//Additional Interrupt Modes Enable Register 	Write-only
#define PIO_AIMDR 	0x00B4 	//Additional Interrupt Modes Disables Register 	Write-only
#define PIO_AIMMR 	0x00B8 	//Additional Interrupt Modes Mask Register 		Read-only
#define PIO_ESR 	0x00C0 	//Edge Select Register 							Write-only
#define PIO_LSR 	0x00C4 	//Level Select Register 						Write-only
#define PIO_ELSR 	0x00C8 	//Edge/Level Status Register 					Read-only
#define PIO_FELLSR 	0x00D0 	//Falling Edge/Low Level Select Register 		Write-only
#define PIO_REHLSR 	0x00D4 	//Rising Edge/ High Level Select Register 		Write-only
#define PIO_FRLHSR 	0x00D8 	//Fall/Rise - Low/High Status Register 			Read-only
#define PIO_LOCKSR 	0x00E0 	//Lock Status 									Read-only
#define PIO_WPMR 	0x00E4 	//Write Protect Mode Register 					Read-write
#define PIO_WPSR 	0x00E8 	//Write Protect Status Register 				Read-only
#define PIO_SCHMITT 0x0100 	//Schmitt Trigger Register 						Read-write
#define PIO_DELAYR 	0x0110 	//IO Delay Register 							Read-write
#define PIO_DRIVER1 0x0114 	//I/O Drive Register 1 							Read-write
#define PIO_DRIVER2 0x0118 	//I/O Drive Register 2 							Read-write

#define GETMODULEOFFSET(reg, pin) (reg + (pin/32 * 512))
#define GETPINPOSITION(pin) (1 << (pin%32))

#define PAGE_SIZE  (2*1024)
#define BLOCK_SIZE (2*1024)

static volatile uint32_t *gpio_map;

void short_wait(void)
{
    int i;
    
    for (i=0; i<150; i++)     // wait 150 cycles
    {
		asm volatile("nop");
    }
}

int setup(void)
{
    int mem_fd;
    uint8_t *gpio_mem;

    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
    {
        return SETUP_DEVMEM_FAIL;
    }

    if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)
        return SETUP_MALLOC_FAIL;

    if ((uint32_t)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((uint32_t)gpio_mem % PAGE_SIZE);

    gpio_map = (uint32_t *)mmap( (caddr_t)gpio_mem, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, GPIO_BASE);

    if ((uint32_t)gpio_map < 0)
        return SETUP_MMAP_FAIL;

    return SETUP_OK;
}

void clear_event_detect(int gpio)
{
	/*
	int offset = EVENT_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

    *(gpio_map+offset) |= (1 << shift);
    short_wait();
    *(gpio_map+offset) = 0;*/
}

int eventdetected(int gpio)
{
	/*
	int offset, value, bit;
   
    offset = EVENT_DETECT_OFFSET + (gpio/32);
    bit = (1 << (gpio%32));
    value = *(gpio_map+offset) & bit;
    if (value)
    {
        clear_event_detect(gpio);
	}

    return value; */
	return 0;
}

void set_rising_event(int gpio, int enable)
{
	/*
	int offset = RISING_ED_OFFSET + (gpio/32);
    int shift = (gpio%32);

	if (enable)
	    *(gpio_map+offset) |= 1 << shift;
	else
	    *(gpio_map+offset) &= ~(1 << shift);
    clear_event_detect(gpio);
    */
}

void set_falling_event(int gpio, int enable)
{
	/*
	int offset = FALLING_ED_OFFSET + (gpio/32);
    int shift = (gpio%32);

	if (enable)
	{
	    *(gpio_map+offset) |= (1 << shift);
	    *(gpio_map+offset) = (1 << shift);
	} else {
	    *(gpio_map+offset) &= ~(1 << shift);
	}
    clear_event_detect(gpio);
    */
}

void set_high_event(int gpio, int enable)
{
	/*
	int offset = HIGH_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

	if (enable)
	{
	    *(gpio_map+offset) |= (1 << shift);
	} else {
	    *(gpio_map+offset) &= ~(1 << shift);
	}
    clear_event_detect(gpio);

    */
}

void set_low_event(int gpio, int enable)
{
	/*
	int offset = LOW_DETECT_OFFSET + (gpio/32);
    int shift = (gpio%32);

	if (enable)
	    *(gpio_map+offset) |= 1 << shift;
	else
	    *(gpio_map+offset) &= ~(1 << shift);
    clear_event_detect(gpio);
    */
}

void set_pullupdn(int gpio, int pud)
{
    int shift = GETPINPOSITION(gpio);

    
    if (pud == PUD_DOWN)
       *(gpio_map+ GETMODULEOFFSET(PIO_PPDER, gpio)) = shift;
    else if (pud == PUD_UP)
       *(gpio_map+ GETMODULEOFFSET(PIO_PUER,gpio)) = shift;
    else  // pud == PUD_OFF
    {
    	*(gpio_map+ GETMODULEOFFSET(PIO_PUDR,gpio)) = shift;
    	*(gpio_map+ GETMODULEOFFSET(PIO_PPDDR,gpio)) = shift;
    }

}

void setup_gpio(int gpio, int direction, int pud)
{
    int shift = GETPINPOSITION(gpio);

    set_pullupdn(gpio, pud);
    if (direction == OUTPUT)
        *(gpio_map+GETMODULEOFFSET(PIO_OER, gpio)) = shift;
    else  // direction == INPUT
        *(gpio_map+GETMODULEOFFSET(PIO_ODR, gpio)) = shift;
}

// Contribution by Eric Ptak <trouch@trouch.com>
int gpio_function(int gpio)
{
 /*  int offset = FSEL_OFFSET + (gpio/10);
   int shift = (gpio%10)*3;
   int value = *(gpio_map+offset);
   value >>= shift;
   value &= 7;
   return value; // 0=input, 1=output, 4=alt0 */
	return 1;
}

void output_gpio(int gpio, int value)
{
    int offset;
    
    if (value) // value == HIGH
        offset =  GETMODULEOFFSET(PIO_SODR, gpio);
    else       // value == LOW
        offset =  GETMODULEOFFSET(PIO_CODR, gpio);


    *(gpio_map+offset) = GETPINPOSITION(gpio);;
}


int input_gpio(int gpio)
{
   int offset, value, mask;
   
   offset = GETMODULEOFFSET(PIO_PDSR, gpio);
   mask = GETPINPOSITION(gpio);
   value = *(gpio_map+offset) & mask;
   return value;
}

void cleanup(void)
{
    // fixme - set all gpios back to input
    munmap((caddr_t)gpio_map, BLOCK_SIZE);
}


