#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* The base address of the GPIO peripheral (ARM Physical Address) varies
* between Raspberry Pi versions. This is for the Raspberry Pi 4. */

#define GPIO_BASE 0xFE200000UL
#define GPIO_LENGTH (4*1024)

// constants for register offsets
#define GPIO_GPFSEL0 0
#define GPIO_GPFSEL1 1
#define GPIO_GPFSEL2 2
#define GPIO_GPFSEL3 3
#define GPIO_GPFSEL4 4
#define GPIO_GPFSEL5 5

#define GPIO_GPSET0 7
#define GPIO_GPSET1 8

#define GPIO_GPCLR0 10
#define GPIO_GPCLR1 11

#define GPLEV0 13

//led setting
#define LED_GPFSEL GPIO_GPFSEL1
#define LED_GPFBITr 0
#define LED_GPFBITy 3
#define LED_GPFBITg 9
#define LED_GPSET GPIO_GPSET0
#define LED_GPCLR GPIO_GPCLR0
#define LED_GPIO_BITr 10
#define LED_GPIO_BITy 11
#define LED_GPIO_BITg 13

#define BUTTON_GPFSEL GPIO_GPFSEL2
#define BUTTON_GPFBIT 18
#define BUTTON_GPSET GPIO_GPSET0
#define BUTTON_GPCLR GPIO_GPCLR0
#define BUTTON_GPIO_BIT 26

/* Pointer to the mapped GPIO registers */
volatile uint32_t *gpio;

int main(int argc, char *argv[])
{
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
	printf("can't open /dev/mem\n");
	return 1;
}

	gpio = mmap(NULL, GPIO_LENGTH, PROT_READ | PROT_WRITE,
	MAP_SHARED, fd, GPIO_BASE);

	if (gpio == (void *) -1) {
		printf("can't mmap\n");
		return 1;
	}

	/* Write 1 to the GPIO16 init nibble in the Function Select 1 GPIO
	peripheral register to enable GPIO16 as an output */

	gpio[LED_GPFSEL] |= 0b001; //red
	gpio[LED_GPFSEL] |= 0b001000; //yellow
	gpio[LED_GPFSEL] |= 0b001000000000; //green



	/* This loop runs forever */
	while (1) {
		
		/* Set the LED GPIO pin low */
    	gpio[LED_GPCLR] = 0b0010000000000; //red
    	gpio[LED_GPCLR] = 0b00100000000000; //yellow
    	gpio[LED_GPCLR] = 0b0010000000000000; //green
		sleep(1);

        /* Set the LED GPIO pin high */
        gpio[LED_GPSET] = 0b0010000000000;
        sleep(1);

        /* Set the LED GPIO pin low */
        gpio[LED_GPSET] = 0b00100000000000;
        sleep(1);
        gpio[LED_GPCLR] = 0b0010000000000;
        gpio[LED_GPCLR] = 0b00100000000000;
        gpio[LED_GPSET] = 0b0010000000000000;
        sleep(3);

        while(1)
        {
        	if(gpio[GPLEV0] & 0b100000000000000000000000000!= 0) break;
        	// if((gpio[GPLEV0] & (1<<(BUTTON_GPIO_BIT & 31)))!= 0) break;

        }


        
        gpio[LED_GPCLR] = 0b0010000000000000;

        for (int i = 0; i < 2 ; i++){
            gpio[LED_GPSET] = 0b00100000000000;
            usleep(300000);
            gpio[LED_GPCLR] = 0b00100000000000;
            usleep(300000);
        }
        gpio[LED_GPSET] = 0b0010000000000;
	}	
}
