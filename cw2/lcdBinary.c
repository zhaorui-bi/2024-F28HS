
#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1


// APP constants   ---------------------------------

// Wiring (see call to lcdInit in main, using BCM numbering)
// NB: this needs to match the wiring as defined in master-mind.c

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

// -----------------------------------------------------------------------------
// includes 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>

// -----------------------------------------------------------------------------
// prototypes

int failure (int fatal, const char *message, ...);

// -----------------------------------------------------------------------------
// Functions to implement here (or directly in master-mind.c)

/* this version needs gpio as argument, because it is in a separate file */
void digitalWrite (uint32_t *gpio, int pin, int value) {
  int off, res;
  off = (value == LOW) ? 10 : 7;

  asm volatile(
    "\tLDR R1, %[gpio]\n" // Load GPIO base address into R1
    "\tADD R0, R1, %[off]\n" // Calculate the address of the specific GPIO pin register
    "\tMOV R2, #1\n" // Set R2 to 1 (value for setting the pin to HIGH)
    "\tMOV R1, %[pin]\n"
    "\tAND R1, #31\n" // Mask pin number to ensure it's within range (0-31)
    "\tLSL R2, R1\n"
    "\tSTR R2, [R0, #0]\n"
    :
    : [pin] "r"(pin), [gpio] "m"(gpio), [off] "r"(off * 4)
    : "r0", "r1", "r2", "cc"); 
}

// adapted from setPinMode
void pinMode(uint32_t *gpio, int pin, int mode){
  int fsel = pin / 10; // Calculate the function select register (fsel) for the specified pin
  int shift = (pin % 10) * 3; // Calculate the shift amount to position the mode value within the register
  int sgn = (mode == INPUT) ? 0 : 1; // Determine the mode value (0 for INPUT, 1 for OUTPUT)

  asm volatile(
    "\tLDR R0, [%[gpio], %[fsel]]\n"   
    "\tMOV R1, #7\n"  // Prepare a mask (0b111) for clearing the function select bits              
    "\tLSL R1, R1, %[shift]\n"  // Shift the mask to the correct bit position     
    "\tBIC R0, R0, R1\n"                 
    "\tMOV R1, %[sgn]\n"  // Load the mode value (0 or 1) into R1
    "\tLSL R1, R1, %[shift]\n"             
    "\tORR R0, R0, R1\n"  // Set the function select bits according to the mode           
    "\tSTR R0, [%[gpio], %[fsel]]\n"    
    :
    : [gpio]"r"(gpio), [fsel]"r"(fsel * 4), [shift]"r"(shift), [sgn]"r"(sgn)
    : "r0", "r1", "cc");
}

int readButton(uint32_t *gpio, int button) {
  int result;
  asm volatile(
    "\tLDR R0, [%[gpio], #0x34]\n" 
    "\tMOV R1, #1\n"
    "\tLSL R1, R1, %[button]\n"  // Create a mask for the specified button pin
    "\tAND %[result], R0, R1\n"  // Mask the GPIO level register to isolate the button pin's value
    "\tMOV %[result], %[result], LSR %[button]\n" 
    : [result]"=&r"(result)  // Output constraint for the result variable
    : [gpio]"r"(gpio), [button]"r"(button)
    : "r0", "r1", "cc");
  return result;
}

void waitForButton(uint32_t *gpio, int button) {
  while (readButton(gpio, button) == 0)
  {
  }
}
