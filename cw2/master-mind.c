
/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)

// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY   200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

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


// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char* color_names[] = { "red", "green", "blue" };

static int* theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

static int lcdControl ;

/* ***************************************************************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

static unsigned int gpiobase ;
static uint32_t *gpio ;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes

int failure (int fatal, const char *message, ...);
void waitForEnter (void);
void waitForButton (uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite (uint32_t *gpio, int pin, int value);

/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode);

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button);

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton (uint32_t *gpio, int button);
/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq() {
  srand(time(NULL)); // Seed the random number generator
  if (theSeq == NULL) {
      theSeq = (int*)malloc(seqlen * sizeof(int)); // Allocate memory for the sequence
      if (theSeq == NULL) {
          failure(1, "Memory allocation failed\n");
          return;
      }
  }
  // Generate a random sequence
  for (int i = 0; i < seqlen; i++) {
      theSeq[i] = rand() % colors + 1; // Generate a random color index, from 1 to 3.
  }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq) {
  for (int i = 0; i < seqlen; i++) {
      printf("%d ", seq[i]);
  }
}

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, as a pointer to a pair of values */
int *countMatches(int *seq1, int *seq2)
{
  int *result = (int *)malloc(2 * sizeof(int)); // variable to store the matches
  
  int exact_matches = 0;
  int approx_matches = 0;

asm(
      "start:\n"
      "\tMOV R0, #0\n" // Initialize exact match count (R0)
      "\tMOV R3, #0\n" // Initialize approximate match count (R3)
      "\tMOV R1, %[seq1]\n" // Load seq1 pointer (R1)
      "\tMOV R2, %[seq2]\n" // Load seq2 pointer (R2)
      "\tMOV R5, #0\n" // Initialize seq1 count (R5)
      "\tMOV R7, #0\n" // Initialize seq1 index (4*count) (R7)
      "\tMOV R6, #0\n" // Initialize seq2 count (R6)
      "\tMOV R8, #0\n" // Initialize seq2 index (R8)
      "\tB exact_loop\n"

      "exact_loop:\n"
      "\tCMP R5, #3\n" // Compare seq1 count with 3
      "\tBEQ reset\n" // If equal, exit the routine
      "\tLDR R9, [R1, R7]\n" // Load value from seq1[index] into R9
      "\tLDR R10, [R2, R7]\n" // Load value from seq2[index] into R10
      "\tCMP R9, R10\n" // Compare seq1[i] and seq2[i]
      "\tBEQ mark_exact\n" // If exact match, mark and increment exact count
      "\tB exact_increment\n" // i++ and back to next exact loop
      
      "exact_increment:\n" 
      "\tADD R5, R5, #1\n" // Increment seq1 count
      "\tADD R7, R7, #4\n" // Increment seq1 index
      "\tB exact_loop\n"// Branch back to main loop
      
      "mark_exact:\n"
      "\tMOV R9, #0 \n" 
      "\tSTR R9, [R1,R7]\n" // Set seq1[index] to 0 (mark as matched)
      "\tMOV R10, #0 \n"
      "\tSTR R10, [R2, R7]\n" // Set seq2[index] to 0 (mark as matched)
      "\tADD R0, R0, #1\n" // Increment exact match count
      "\tB exact_increment\n" 
      
      "reset:\n"
      "\tMOV R5, #0\n" // Reset seq1 count
      "\tMOV R7, #0\n" // Reset seq1 index
      "\tMOV R6, #0\n" // Reset seq2 count
      "\tMOV R8, #0\n" // Reset seq2 index
      "\tB main_loop\n"

      "main_loop:\n" 
      "\tCMP R5, #3\n" // Compare seq1 count with 3
      "\tBEQ exit_routine\n" // If equal, exit the routine
      "\tLDR R9, [R1, R7]\n" // Load value from seq1[index] into R9
      "\tCMP R9, #0\n" // Chech whether seq1[index] has been matched
      "\tBEQ go_next\n" // If matched, go to next loop
      "\tMOV R6, #0\n" // Reset seq2 count
      "\tMOV R8, #0\n" // Reset seq2 index
      "\tB approx_loop\n" // Branch to approximate matching loop (traverse seq2)
      "\tB go_next\n"

      "go_next:\n"
      "\tADD R5, R5, #1\n" // Increment seq1 count
      "\tADD R7, R7, #4\n" // Increment seq1 index
      "\tB main_loop\n" // Back to main loop

      "approx_loop:\n"
      "\tCMP R6, #3\n" // Compare seq2 count with 3
      "\tBEQ go_next\n" // If equal, go to next loop
      "\tLDR R10, [R2, R8]\n" 
      "\tCMP R9, R10\n" // Compare seq1[i] and seq2[j]
      "\tBEQ mark_approx\n" // If equal, approx_count++
      "\tB approx_increment\n" // j++

      "approx_increment:\n"
      "\tADD R6, R6, #1\n" // Increment seq2 count
      "\tADD R8, R8, #4\n" // Increment seq2 index
      "\tB approx_loop\n"

      "mark_approx:\n"
      "\tMOV R10, #0 \n"
      "\tSTR R10, [R2, R8]\n" // Set seq2[j] to 0 (mark as matched)
      "\tADD R3, R3, #1\n" // Increment approximate match count
      "\tB go_next\n"

      "exit_routine:\n"
      "\tMOV %[exact], R0\n"
      "\tMOV %[approx], R3\n"

      : [exact] "=r"(exact_matches), [approx] "=r"(approx_matches)
      : [seq1] "r"(seq1), [seq2] "r"(seq2), [seqlen] "r"(seqlen)
      : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "cc");

  result[0] = exact_matches;
  result[1] = approx_matches;

  return result;
}

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int* code, int *seq1, int *seq2) {
  int exactMatches = code[0];
  int approximateMatches = code[1];

  printf("\nSecret: ");
  showSeq(seq1); // Display the secret sequence (seq1)
 
  printf("\nGuess: "); 
  showSeq(seq2); // Display the guessed sequence (seq2)
  
  printf("\nExact Matches: %d\n", exactMatches);
  printf("Approximate Matches: %d\n\n", approximateMatches);
}

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* Returns the current time in microseconds */
uint64_t timeInMicroseconds(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

/* Signal handler for the interval timer (SIGALRM) */
void timer_handler (int signum) {
  stopT=timeInMicroseconds();
  timed_out = 1; // Flag indicating timeout
}

/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout){
  startT=timeInMicroseconds(); // Record start time
  struct sigaction sa;
  struct itimerval timer;
  
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &timer_handler;
  sigaction(SIGALRM, &sa, NULL); // Install the signal handler for SIGALRM
  
  timer.it_value.tv_sec = timeout / 1000000;  
  timer.it_value.tv_usec = timeout % 1000000; 

  timer.it_interval.tv_sec = 0;  
  timer.it_interval.tv_usec = 0; 

  setitimer(ITIMER_REAL, &timer, NULL); // Set the interval timer
}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal) //  && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
  vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}


/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}


void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}

/* ======================================================= */
/* SECTION: LCD functions                                  */
/* ------------------------------------------------------- */
/* medium-level interface functions (all in C) */

/* from wiringPi:
 * strobe:
 *	Toggle the strobe (Really the "E") pin to the device.
 *	According to the docs, data is latched on the falling edge.
 *********************************************************************************
 */

void strobe (const struct lcdDataStruct *lcd)
{

  // Note timing changes for new version of delayMicroseconds ()
  digitalWrite (gpio, lcd->strbPin, 1) ; delayMicroseconds (50) ;
  digitalWrite (gpio, lcd->strbPin, 0) ; delayMicroseconds (50) ;
}

/*
 * sentDataCmd:
 *	Send an data or command byte to the display.
 *********************************************************************************
 */

void sendDataCmd (const struct lcdDataStruct *lcd, unsigned char data)
{
  register unsigned char myData = data ;
  unsigned char          i, d4 ;

  if (lcd->bits == 4)
  {
    d4 = (myData >> 4) & 0x0F;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
    strobe (lcd) ;

    d4 = myData & 0x0F ;
    for (i = 0 ; i < 4 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (d4 & 1)) ;
      d4 >>= 1 ;
    }
  }
  else
  {
    for (i = 0 ; i < 8 ; ++i)
    {
      digitalWrite (gpio, lcd->dataPins [i], (myData & 1)) ;
      myData >>= 1 ;
    }
  }
  strobe (lcd) ;
}

/*
 * lcdPutCommand:
 *	Send a command byte to the display
 *********************************************************************************
 */

void lcdPutCommand (const struct lcdDataStruct *lcd, unsigned char command)
{
#ifdef DEBUG
  fprintf(stderr, "lcdPutCommand: digitalWrite(%d,%d) and sendDataCmd(%d,%d)\n", lcd->rsPin,   0, lcd, command);
#endif
  digitalWrite (gpio, lcd->rsPin,   0) ;
  sendDataCmd  (lcd, command) ;
  delay (2) ;
}

void lcdPut4Command (const struct lcdDataStruct *lcd, unsigned char command)
{
  register unsigned char myCommand = command ;
  register unsigned char i ;

  digitalWrite (gpio, lcd->rsPin,   0) ;

  for (i = 0 ; i < 4 ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], (myCommand & 1)) ;
    myCommand >>= 1 ;
  }
  strobe (lcd) ;
}

/*
 * lcdHome: lcdClear:
 *	Home the cursor or clear the screen.
 *********************************************************************************
 */

void lcdHome (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdHome: lcdPutCommand(%d,%d)\n", lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

void lcdClear (struct lcdDataStruct *lcd)
{
#ifdef DEBUG
  fprintf(stderr, "lcdClear: lcdPutCommand(%d,%d) and lcdPutCommand(%d,%d)\n", lcd, LCD_CLEAR, lcd, LCD_HOME);
#endif
  lcdPutCommand (lcd, LCD_CLEAR) ;
  lcdPutCommand (lcd, LCD_HOME) ;
  lcd->cx = lcd->cy = 0 ;
  delay (5) ;
}

/*
 * lcdPosition:
 *	Update the position of the cursor on the display.
 *	Ignore invalid locations.
 *********************************************************************************
 */

void lcdPosition (struct lcdDataStruct *lcd, int x, int y)
{
  // struct lcdDataStruct *lcd = lcds [fd] ;

  if ((x > lcd->cols) || (x < 0))
    return ;
  if ((y > lcd->rows) || (y < 0))
    return ;

  lcdPutCommand (lcd, x + (LCD_DGRAM | (y>0 ? 0x40 : 0x00)  /* rowOff [y] */  )) ;

  lcd->cx = x ;
  lcd->cy = y ;
}



/*
 * lcdDisplay: lcdCursor: lcdCursorBlink:
 *	Turn the display, cursor, cursor blinking on/off
 *********************************************************************************
 */

void lcdDisplay (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_DISPLAY_CTRL ;
  else
    lcdControl &= ~LCD_DISPLAY_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursor (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_CURSOR_CTRL ;
  else
    lcdControl &= ~LCD_CURSOR_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

void lcdCursorBlink (struct lcdDataStruct *lcd, int state)
{
  if (state)
    lcdControl |=  LCD_BLINK_CTRL ;
  else
    lcdControl &= ~LCD_BLINK_CTRL ;

  lcdPutCommand (lcd, LCD_CTRL | lcdControl) ; 
}

/*
 * lcdPutchar:
 *	Send a data byte to be displayed on the display. We implement a very
 *	simple terminal here - with line wrapping, but no scrolling. Yet.
 *********************************************************************************
 */

void lcdPutchar (struct lcdDataStruct *lcd, unsigned char data)
{
  digitalWrite (gpio, lcd->rsPin, 1) ;
  sendDataCmd  (lcd, data) ;

  if (++lcd->cx == lcd->cols)
  {
    lcd->cx = 0 ;
    if (++lcd->cy == lcd->rows)
      lcd->cy = 0 ;

    lcdPutCommand (lcd, lcd->cx + (LCD_DGRAM | (lcd->cy>0 ? 0x40 : 0x00) )) ;
  }
}


/*
 * lcdPuts:
 *	Send a string to be displayed on the display
 *********************************************************************************
 */

void lcdPuts (struct lcdDataStruct *lcd, const char *string)
{
  while (*string)
    lcdPutchar (lcd, *string++) ;
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c) { 
  pinMode(gpio, led, OUTPUT);  

  for (int i = 0; i < c; i++) {
    digitalWrite(gpio, led, HIGH);  
    delay(500);                   
    digitalWrite(gpio, led, LOW);   
    delay(500);                    
  }
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main (int argc, char *argv[])
{
  struct lcdDataStruct *lcd ;
  int bits, rows, cols ;
  unsigned char func ;

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin,  clrOff, setOff, off, res;
  int fd ;

  int  exact, contained;
  char str1[32];
  char str2[32];
  
  struct timeval t1, t2 ;
  int t ;

  char buf [32] ;

  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;
  
  bits = 4; 
  cols = 16; 
  rows = 2; 
  
  if (geteuid () != 0)
    fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int*) malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0xFE200000;

  // -----------------------------------------------------------------------------
  // memory mapping 
  // Open the master /dev/memory device

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
  if ((int32_t)gpio == -1)
    return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

  // -------------------------------------------------------
  // Configuration of LED and BUTTON
  digitalWrite(gpio, LED,    0); pinMode(gpio, LED,   OUTPUT); // green
  digitalWrite(gpio, LED2,   0); pinMode(gpio, LED2,  OUTPUT); // red
  digitalWrite(gpio, BUTTON, 0); pinMode(gpio, BUTTON, INPUT);
  // -------------------------------------------------------
  // INLINED version of lcdInit (can only deal with one LCD attached to the RPi):
  // you can use this code as-is, but you need to implement digitalWrite() and
  // pinMode() which are called from this code
  // Create a new LCD:
  lcd = (struct lcdDataStruct *) malloc (sizeof (struct lcdDataStruct)) ;
  if (lcd == NULL)
    return -1;

  // hard-wired GPIO pins
  lcd->rsPin   = RS_PIN ;
  lcd->strbPin = STRB_PIN ;
  lcd->bits    = 4 ;
  lcd->rows    = rows ;  // # of rows on the display
  lcd->cols    = cols ;  // # of cols on the display
  lcd->cx      = 0 ;     // x-pos of cursor
  lcd->cy      = 0 ;     // y-pos of curosr

  lcd->dataPins [0] = DATA0_PIN ;
  lcd->dataPins [1] = DATA1_PIN ;
  lcd->dataPins [2] = DATA2_PIN ;
  lcd->dataPins [3] = DATA3_PIN ;
  
  digitalWrite(gpio, lcd->rsPin,   0) ; pinMode(gpio, lcd->rsPin,   OUTPUT) ;
  digitalWrite(gpio, lcd->strbPin, 0) ; pinMode(gpio, lcd->strbPin, OUTPUT) ;
  
  for (i = 0 ; i < bits ; ++i)
  {
    digitalWrite (gpio, lcd->dataPins [i], 0) ;
    pinMode      (gpio, lcd->dataPins [i], OUTPUT) ;
  }
  
  delay (35) ; // mS

  if (bits == 4)
  {
    func = LCD_FUNC | LCD_FUNC_DL ;			// Set 8-bit mode 3 times
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    func = LCD_FUNC ;					// 4th set: 4-bit mode
    lcdPut4Command (lcd, func >> 4) ; delay (35) ;
    lcd->bits = 4 ;
  }
  else
  {
    failure(TRUE, "setup: only 4-bit connection supported\n");
    func = LCD_FUNC | LCD_FUNC_DL ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
    lcdPutCommand  (lcd, func     ) ; delay (35) ;
  }

  if (lcd->rows > 1)
  {
    func |= LCD_FUNC_N ;
    lcdPutCommand (lcd, func) ; delay (35) ;
  }

  // Rest of the initialisation sequence
  lcdDisplay     (lcd, TRUE) ;
  lcdCursor      (lcd, FALSE) ;
  lcdCursorBlink (lcd, FALSE) ;
  lcdClear       (lcd) ;

  lcdPutCommand (lcd, LCD_ENTRY   | LCD_ENTRY_ID) ;    // set entry mode to increment address counter after write
  lcdPutCommand (lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL) ;  // set display shift to right-to-left

  // END lcdInit ------
  // -----------------------------------------------------------------------------
  // Start of game
  
  fprintf (stderr, "Start\n");
  lcdPosition(lcd,0,0);  // Set LCD cursor position to row 0, column 0
  lcdPuts(lcd,"WELCOME"); // Display "WELCOME" on the LCD screen
  
  /* initialise the secret sequence */
  initSeq(); 
  printf("Secret: ");
  showSeq(theSeq);
  printf("\n");

  // -----------------------------------------------------------------------------
  // +++++ main loop
  while (!found) {
    attempts++; // Attempts counter
    if (attempts > 14) // Exit the loop if attempts exceed 14
    {
      fprintf (stderr, "Game Over");
      break;
    }

    fprintf (stderr, "********* Attempt %d **********\n", attempts);
    fprintf (stderr, "Input number 1:\n");
    for(int i=0;i<3;i++){ // Read three numbers through butten
      waitForButton(gpio, BUTTON); // Wait for button press
      fprintf (stderr, "** Button pressed **\n");
      delay(500); 
      int consecutivePressCount = 1;
      initITimer(4000000); // Initialize interval timer (4 seconds)
      // Collect consecutive button presses within the timeout period
      while (1) {
        int input = readButton(gpio, BUTTON);
        if (input == 1) {
          delay(500);
          // Button pressed
          if (consecutivePressCount<3){
            fprintf (stderr, "** Button pressed **\n");
            consecutivePressCount++;
          }
        } else {
          // After 4 second, set timed_out to 0 and break
          if (timed_out) {
            timed_out = 0;
            break;
          }
        }
      }
      
      blinkN(gpio, LED2, 1);
      // Blink green LED to ensure the input
      blinkN(gpio, LED, consecutivePressCount); 
      attSeq[i] = consecutivePressCount;
      if (i<2)
        fprintf (stderr, "Input number %d:\n", i+2);
    }
    
    // Compare the attempt sequence with the secret sequence
    for(int i=0;i<seqlen;i++){
      cpy1[i]=theSeq[i];
      cpy2[i]=attSeq[i];
    }
  
    int* result = countMatches(cpy1,cpy2);
    
    showMatches(result,theSeq,attSeq); // Display match results
    
    int exactMatches = result[0];
    int approximateMatches = result[1];
    
    // Blink green LED to show the exact matches 
    blinkN(gpio, LED2, 2);
    if(exactMatches!=0)
      blinkN(gpio, LED, exactMatches);
    delay(1000);
    
    blinkN(gpio, LED2, 1);
    // Blink green LED to show the exact matche
    if(approximateMatches!=0)
      blinkN(gpio, LED, approximateMatches);
    delay(1000);
    
    // Show match results on LCD or output
    lcdClear(lcd);
    char str1[20];
    char str2[20];
    sprintf(str1,"Exact: %d",exactMatches);
    sprintf(str2,"Approx: %d",approximateMatches);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,str1);
    lcdPosition(lcd,0,1);
    lcdPuts(lcd,str2);
    
    // Check if the sequence is fully matched
    if (exactMatches == seqlen) {
        found = 1;
        break;
    }
    
    blinkN(gpio, LED2, 3); // Present the start of next loop
  }
  
  fprintf (stderr, "Game completed in %d rounds\n", attempts);
  if (found) {
    fprintf (stderr, "Success\n");
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"SUCCESS");
    char tempStr[20];
    lcdPosition(lcd,0,1);
    sprintf(tempStr,"attempts: %d",attempts);
    lcdPuts(lcd,tempStr);
  } else {
    fprintf (stderr, "Fail\n");
    lcdClear(lcd);
    lcdPosition(lcd,0,0);
    lcdPuts(lcd,"Game Over");
    char tempStr[20];
    lcdPosition(lcd,0,1);
    sprintf(tempStr,"attempts: %d",attempts);
    lcdPuts(lcd,tempStr);
  }
  return 0;
}
