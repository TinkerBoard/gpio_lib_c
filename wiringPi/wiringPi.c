/*
 * wiringPi:
 *	Arduino look-a-like Wiring library for the Raspberry Pi
 *	Copyright (c) 2012-2015 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose


#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"

#ifndef	TRUE
#define	TRUE	(1==1)
#define	FALSE	(1==2)
#endif

// Environment Variables

#define	ENV_DEBUG	"WIRINGPI_DEBUG"
#define	ENV_CODES	"WIRINGPI_CODES"
#define	ENV_GPIOMEM	"WIRINGPI_GPIOMEM"


// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFF00)

struct wiringPiNodeStruct *wiringPiNodes = NULL ;

// BCM Magic

#define	BCM_PASSWORD		0x5A000000


// The BCM2835 has 54 GPIO pins.
//	BCM2835 data sheet, Page 90 onwards.
//	There are 6 control registers, each control the functions of a block
//	of 10 pins.
//	Each control register has 10 sets of 3 bits per GPIO pin - the ALT values
//
//	000 = GPIO Pin X is an input
//	001 = GPIO Pin X is an output
//	100 = GPIO Pin X takes alternate function 0
//	101 = GPIO Pin X takes alternate function 1
//	110 = GPIO Pin X takes alternate function 2
//	111 = GPIO Pin X takes alternate function 3
//	011 = GPIO Pin X takes alternate function 4
//	010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//	X / 10 + ((X % 10) * 3)

// Port function select bits

#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:
//
// Updates in September 2015 - all now static variables (and apologies for the caps)
//	due to the Pi v2 and the new /dev/gpiomem interface

static volatile unsigned int RASPBERRY_PI_PERI_BASE ;
static volatile unsigned int GPIO_PADS ;
static volatile unsigned int GPIO_CLOCK_BASE ;
static volatile unsigned int GPIO_BASE ;
static volatile unsigned int GPIO_TIMER ;
static volatile unsigned int GPIO_PWM ;

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

// PWM
//	Word offsets into the PWM control region

#define	PWM_CONTROL 0
#define	PWM_STATUS  1
#define	PWM0_RANGE  4
#define	PWM0_DATA   5
#define	PWM1_RANGE  8
#define	PWM1_DATA   9

//	Clock regsiter offsets

#define	PWMCLK_CNTL	40
#define	PWMCLK_DIV	41

#define	PWM0_MS_MODE    0x0080  // Run in MS mode
#define	PWM0_USEFIFO    0x0020  // Data from FIFO
#define	PWM0_REVPOLAR   0x0010  // Reverse polarity
#define	PWM0_OFFSTATE   0x0008  // Ouput Off state
#define	PWM0_REPEATFF   0x0004  // Repeat last value if FIFO empty
#define	PWM0_SERIAL     0x0002  // Run in serial mode
#define	PWM0_ENABLE     0x0001  // Channel Enable

#define	PWM1_MS_MODE    0x8000  // Run in MS mode
#define	PWM1_USEFIFO    0x2000  // Data from FIFO
#define	PWM1_REVPOLAR   0x1000  // Reverse polarity
#define	PWM1_OFFSTATE   0x0800  // Ouput Off state
#define	PWM1_REPEATFF   0x0400  // Repeat last value if FIFO empty
#define	PWM1_SERIAL     0x0200  // Run in serial mode
#define	PWM1_ENABLE     0x0100  // Channel Enable

// Timer
//	Word offsets

#define	TIMER_LOAD	(0x400 >> 2)
#define	TIMER_VALUE	(0x404 >> 2)
#define	TIMER_CONTROL	(0x408 >> 2)
#define	TIMER_IRQ_CLR	(0x40C >> 2)
#define	TIMER_IRQ_RAW	(0x410 >> 2)
#define	TIMER_IRQ_MASK	(0x414 >> 2)
#define	TIMER_RELOAD	(0x418 >> 2)
#define	TIMER_PRE_DIV	(0x41C >> 2)
#define	TIMER_COUNTER	(0x420 >> 2)

// Locals to hold pointers to the hardware

static volatile uint32_t *gpio ;
static volatile uint32_t *pwm ;
static volatile uint32_t *clk ;
static volatile uint32_t *pads ;

#ifdef	USE_TIMER
static volatile uint32_t *timer ;
static volatile uint32_t *timerIrqRaw ;
#endif

static asusversion=0;

// Data for use with the boardId functions.
//	The order of entries here to correspond with the PI_MODEL_X
//	and PI_VERSION_X defines in wiringPi.h
//	Only intended for the gpio command - use at your own risk!

static int piModel2 = FALSE ;

const char *piModelNames [16] =
{
  "Model A",	//  0
  "Model B",	//  1
  "Model A+",	//  2
  "Model B+",	//  3
  "Pi 2",	//  4
  "Alpha",	//  5
  "CM",		//  6
  "Unknown07",	// 07
  "Unknown08",	// 08
  "Pi Zero",	// 09
  "ASUS pi",	// 10
  "Unknown11",	// 11
  "Unknown12",	// 12
  "Unknown13",	// 13
  "Unknown14",	// 14
  "Unknown15",	// 15
} ;

const char *piRevisionNames [16] =
{
  "00",
  "01",
  "02",
  "03",
  "04",
  "05",
  "06",
  "07",
  "08",
  "09",
  "10",
  "11",
  "12",
  "13",
  "14",
  "15",
} ;

const char *piMakerNames [16] =
{
  "Sony",	//	 0
  "Egoman",	//	 1
  "Embest",	//	 2
  "Unknown",	//	 3
  "Asus",	//	 4
  "Unknown05",	//	 5
  "Unknown06",	//	 6
  "Unknown07",	//	 7
  "Unknown08",	//	 8
  "Unknown09",	//	 9
  "Unknown10",	//	10
  "Unknown11",	//	11
  "Unknown12",	//	12
  "Unknown13",	//	13
  "Unknown14",	//	14
  "Unknown15",	//	15
} ;

const int piMemorySize [8] =
{
   256,		//	 0
   512,		//	 1
  1024,		//	 2
     0,		//	 3
     0,		//	 4
     0,		//	 5
     0,		//	 6
     0,		//	 7
} ;

// Time for easy calculations

static uint64_t epochMilli, epochMicro ;

// Misc

static int wiringPiMode = WPI_MODE_UNINITIALISED ;
static volatile int    pinPass = -1 ;
static pthread_mutex_t pinMutex ;

// Debugging & Return codes

int wiringPiDebug       = FALSE ;
int wiringPiReturnCodes = FALSE ;

// Use /dev/gpiomem ?

int wiringPiTryGpioMem  = FALSE ;

// sysFds:
//	Map a file descriptor from the /sys/class/gpio/gpioX/value

static int sysFds [258] =
{
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1,
} ;

// ISR Data

static void (*isrFunctions [258])(void) ;

// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more innefficient than all the bit-twidling, but it
//	does tend to make it all a bit clearer. At least to me!

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//	Cope for 3 different board revisions here.

static int *pinToGpio ;

// Revision 1, 1.1:

static int pinToGpioR1 [64] =
{
  17, 18, 21, 22, 23, 24, 25, 4,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
   0,  1,				// I2C  - SDA1, SCL1				wpi  8 -  9
   8,  7,				// SPI  - CE1, CE0				wpi 10 - 11
  10,  9, 11, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
  14, 15,				// UART - Tx, Rx				wpi 15 - 16

// Padding:

      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;

// Revision 2:

static int pinToGpioR2 [64] =
{
  17, 18, 27, 22, 23, 24, 25, 4,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
   2,  3,				// I2C  - SDA0, SCL0				wpi  8 -  9
   8,  7,				// SPI  - CE1, CE0				wpi 10 - 11
  10,  9, 11, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
  14, 15,				// UART - Tx, Rx				wpi 15 - 16
  28, 29, 30, 31,			// Rev 2: New GPIOs 8 though 11			wpi 17 - 20
   5,  6, 13, 19, 26,			// B+						wpi 21, 22, 23, 24, 25
  12, 16, 20, 21,			// B+						wpi 26, 27, 28, 29
   0,  1,				// B+						wpi 30, 31

// Padding:

  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;


// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

static int *physToGpio ;

static int physToGpioR1 [64] =
{
  -1,		// 0
  -1, -1,	// 1, 2
   0, -1,
   1, -1,
   4, 14,
  -1, 15,
  17, 18,
  21, -1,
  22, 23,
  -1, 24,
  10, -1,
   9, 25,
  11,  8,
  -1,  7,	// 25, 26

                                              -1, -1, -1, -1, -1,	// ... 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;

static int physToGpioR2 [64] =
{
  -1,		// 0
  -1, -1,	// 1, 2
   2, -1,
   3, -1,
   4, 14,
  -1, 15,
  17, 18,
  27, -1,
  22, 23,
  -1, 24,
  10, -1,
   9, 25,
  11,  8,
  -1,  7,	// 25, 26

// B+

   0,  1,
   5, -1,
   6, 12,
  13, -1,
  19, 16,
  26, 20,
  -1, 21,

// the P5 connector on the Rev 2 boards:

  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  28, 29,
  30, 31,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
} ;

// gpioToGPFSEL:
//	Map a BCM_GPIO pin to it's Function Selection
//	control port. (GPFSEL 0-5)
//	Groups of 10 - 3 bits per Function - 30 bits per port

static uint8_t gpioToGPFSEL [] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
} ;


// gpioToShift
//	Define the shift up for the 3 bits per pin in each GPFSEL port

static uint8_t gpioToShift [] =
{
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
} ;


// gpioToGPSET:
//	(Word) offset to the GPIO Set registers for each GPIO pin

static uint8_t gpioToGPSET [] =
{
   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
   8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
} ;

// gpioToGPCLR:
//	(Word) offset to the GPIO Clear registers for each GPIO pin

static uint8_t gpioToGPCLR [] =
{
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
} ;


// gpioToGPLEV:
//	(Word) offset to the GPIO Input level registers for each GPIO pin

static uint8_t gpioToGPLEV [] =
{
  13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
} ;


#ifdef notYetReady
// gpioToEDS
//	(Word) offset to the Event Detect Status

static uint8_t gpioToEDS [] =
{
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
} ;

// gpioToREN
//	(Word) offset to the Rising edge ENable register

static uint8_t gpioToREN [] =
{
  19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
  20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
} ;

// gpioToFEN
//	(Word) offset to the Falling edgde ENable register

static uint8_t gpioToFEN [] =
{
  22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
  23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
} ;
#endif


// GPPUD:
//	GPIO Pin pull up/down register

#define	GPPUD	37

// gpioToPUDCLK
//	(Word) offset to the Pull Up Down Clock regsiter

static uint8_t gpioToPUDCLK [] =
{
  38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
  39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
} ;


// gpioToPwmALT
//	the ALT value to put a GPIO pin into PWM mode

static uint8_t gpioToPwmALT [] =
{
          0,         0,         0,         0,         0,         0,         0,         0,	//  0 ->  7
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0,         0,         0, 	//  8 -> 15
          0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
          0,         0,         0,         0,         0,         0,         0,         0,	// 32 -> 39
  FSEL_ALT0, FSEL_ALT0,         0,         0,         0, FSEL_ALT0,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63
} ;


// gpioToPwmPort
//	The port value to put a GPIO pin into PWM mode

static uint8_t gpioToPwmPort [] =
{
          0,         0,         0,         0,         0,         0,         0,         0,	//  0 ->  7
          0,         0,         0,         0, PWM0_DATA, PWM1_DATA,         0,         0, 	//  8 -> 15
          0,         0, PWM0_DATA, PWM1_DATA,         0,         0,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
          0,         0,         0,         0,         0,         0,         0,         0,	// 32 -> 39
  PWM0_DATA, PWM1_DATA,         0,         0,         0, PWM1_DATA,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63

} ;

// gpioToGpClkALT:
//	ALT value to put a GPIO pin into GP Clock mode.
//	On the Pi we can really only use BCM_GPIO_4 and BCM_GPIO_21
//	for clocks 0 and 1 respectively, however I'll include the full
//	list for completeness - maybe one day...

#define	GPIO_CLOCK_SOURCE	1

// gpioToGpClkALT0:

static uint8_t gpioToGpClkALT0 [] =
{
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0,         0,	//  0 ->  7
          0,         0,         0,         0,         0,         0,         0,         0, 	//  8 -> 15
          0,         0,         0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
  FSEL_ALT0,         0, FSEL_ALT0,         0,         0,         0,         0,         0,	// 32 -> 39
          0,         0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0,         0,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63
} ;

// gpioToClk:
//	(word) Offsets to the clock Control and Divisor register

static uint8_t gpioToClkCon [] =
{
         -1,        -1,        -1,        -1,        28,        30,        32,        -1,	//  0 ->  7
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1, 	//  8 -> 15
         -1,        -1,        -1,        -1,        28,        30,        -1,        -1, 	// 16 -> 23
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 24 -> 31
         28,        -1,        28,        -1,        -1,        -1,        -1,        -1,	// 32 -> 39
         -1,        -1,        28,        30,        28,        -1,        -1,        -1,	// 40 -> 47
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 48 -> 55
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 56 -> 63
} ;

static uint8_t gpioToClkDiv [] =
{
         -1,        -1,        -1,        -1,        29,        31,        33,        -1,	//  0 ->  7
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1, 	//  8 -> 15
         -1,        -1,        -1,        -1,        29,        31,        -1,        -1, 	// 16 -> 23
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 24 -> 31
         29,        -1,        29,        -1,        -1,        -1,        -1,        -1,	// 32 -> 39
         -1,        -1,        29,        31,        29,        -1,        -1,        -1,	// 40 -> 47
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 48 -> 55
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 56 -> 63
} ;


/*
 * Functions
 *********************************************************************************
 */
//jason add for asuspi
static int  mem_fd0;
static void* gpio_map0[9];
static volatile unsigned* gpio0[9];


static int  mem_fd4;
static void *grf_map;
static volatile unsigned *grf;

static int  mem_fdp;
static void *pwm_map;
static volatile unsigned *pwm;

static int  mem_fdpmu;
static void *pmu_map;
static volatile unsigned *pmu;

static int  mem_fdcru;
static void *cru_map;
static volatile unsigned *cru;

static int pwm_divisor = 124;
static int pwm_range = 1024;

#define	MAX_PIN_NUM		      (0x40) 
#define CENTERPWM	0x01
//asuspi gpio
//asus
#define SERIAL       40
#define SPI          41
#define I2C          42
#define PWM          43
#define GPIO		 44
#define TS_XXXX		 45
#define RESERVED	 46
#define I2S		 47
#define GPS_MAG		 48
#define HSADCT		 49
#define USB		 50
#define HDMI		 51
#define SC_XXX		 52
#define GPIOIN		 53
#define GPIOOUT		 54
#define CLKOUT		 55
#define CLK1_27M	 56


#define GPIO0_C1		17			//7----->17

#define GPIO5_B0		(8+152)		//7----->160
#define GPIO5_B1		(9+152)		//8----->161
#define GPIO5_B2		(10+152)	//7----->162
#define GPIO5_B3		(11+152)	//7----->163
#define GPIO5_B4		(12+152)	//7----->164
#define GPIO5_B5		(13+152)	//7----->165
#define GPIO5_B6		(14+152)	//7----->166
#define GPIO5_B7		(15+152)	//7----->167
#define GPIO5_C0		(16+152)	//7----->168
#define GPIO5_C1		(17+152)	//7----->169
#define GPIO5_C2		(18+152)	//7----->170
#define GPIO5_C3		(19+152)	//7----->171

#define GPIO6_A0		(184)		//7----->184
#define GPIO6_A1		(1+184)		//7----->185
#define GPIO6_A3		(3+184)		//7----->187
#define GPIO6_A4		(4+184)		//7----->188

#define GPIO7_A7		(7+216)		//7----->223
#define GPIO7_B0		(8+216)		//7----->224
#define GPIO7_B1		(9+216)		//7----->225
#define GPIO7_B2		(10+216)	//7----->226
#define GPIO7_C1		(17+216)	//7----->233
#define GPIO7_C2		(18+216)	//7----->234
#define GPIO7_C6		(22+216)	//7----->238
#define GPIO7_C7		(23+216)	//7----->239

#define GPIO8_A3		(3+248)		//7----->251
#define GPIO8_A4		(4+248)		//3----->252
#define	GPIO8_A5		(5+248)		//5----->253
#define GPIO8_A6		(6+248)		//7----->254
#define GPIO8_A7		(7+248)		//7----->255
#define GPIO8_B0		(8+248)		//7----->256
#define GPIO8_B1		(9+248)		//7----->257

#define RK3288_PMU		0xff730000
#define PMU_GPIO0C_IOMUX	0x008c
#define PMU_GPIO0C_P		0x006c

#define RK3288_GPIO(x)		(GPIO0_BASE+x*GPIO_LENGTH+(x>0)*GPIO_CHANNEL)
#define GPIO_LENGTH 		0x00010000
#define GPIO_CHANNEL 		0x00020000
#define GPIO0_BASE		0xff750000
#define GPIO_BANK		9
#define RK3288_GRF_PHYS		0xff770000

#define GRF_GPIO5B_IOMUX	0x0050
#define GRF_GPIO5C_IOMUX	0x0054
#define GRF_GPIO6A_IOMUX	0x005c
#define GRF_GPIO6B_IOMUX	0x0060
#define GRF_GPIO6C_IOMUX	0x0064
#define GRF_GPIO7A_IOMUX	0x006c
#define GRF_GPIO7B_IOMUX	0x0070
#define GRF_GPIO7CL_IOMUX	0x0074
#define GRF_GPIO7CH_IOMUX	0x0078
#define GRF_GPIO8A_IOMUX	0x0080
#define GRF_GPIO8B_IOMUX	0x0084

#define GRF_GPIO5B_P	0x0184
#define GRF_GPIO5C_P	0x0188
#define GRF_GPIO6A_P	0x0190
#define GRF_GPIO6B_P	0x0194
#define GRF_GPIO6C_P	0x0198
#define GRF_GPIO7A_P	0x01a0
#define GRF_GPIO7B_P	0x01a4
#define GRF_GPIO7C_P	0x01a8
#define GRF_GPIO8A_P	0x01b0
#define GRF_GPIO8B_P	0x01b4


#define RK3288_PWM 			0xff680000
#define RK3288_PWM0_CNT			0x0000
#define RK3288_PWM0_PERIOD		0x0004
#define RK3288_PWM0_DUTY		0x0008
#define RK3288_PWM0_CTR			0x000c
#define RK3288_PWM1_CNT			0x0010
#define RK3288_PWM1_PERIOD		0x0014
#define RK3288_PWM1_DUTY		0x0018
#define RK3288_PWM1_CTR			0x001c
#define RK3288_PWM2_CNT			0x0020
#define RK3288_PWM2_PERIOD		0x0024
#define RK3288_PWM2_DUTY		0x0028
#define RK3288_PWM2_CTR			0x002c
#define RK3288_PWM3_CNT			0x0030
#define RK3288_PWM3_PERIOD		0x0034
#define RK3288_PWM3_DUTY		0x0038
#define RK3288_PWM3_CTR			0x003c


#define GPIO_SWPORTA_DR_OFFSET		0x0000
#define	GPIO_SWPORTA_DDR_OFFSET		0x0004
#define GPIO_INTEN_OFFSET		0x0030		
#define GPIO_INTMASK_OFFSET		0x0034
#define GPIO_INTTYPE_LEVEL_OFFSET	0x0038
#define GPIO_INT_POLARITY_OFFSET	0x003c
#define GPIO_INT_STATUS_OFFSET		0x0040
#define GPIO_INT_RAWSTATUS_OFFSET	0x0044
#define GPIO_DEBOUNCE_OFFSET		0x0048	
#define GPIO_PORTA_EOF_OFFSET		0x004c
#define GPIO_EXT_PORTA_OFFSET		0x0050
#define GPIO_LS_SYNC_OFFSET		0x0060

#define RK3288_CRU			0xff760000
#define CRU_CLKSEL2_CON			0x0068

//#define PWM0 26
//#define PWM1 27
#define PWM2 GPIO7_C6
#define PWM3 GPIO7_C7
////////////////////



/*
#define RK3128_GPIO(x)		(0x2007c000+x*0x00004000)
#define RK3128_BASE			0x20000000
#define RK3128_GRF_PHYS 	(RK3128_BASE+0x8000)
#define GRF_GPIO0A_IOMUX 	(0x00a8)
#define GRF_GPIO0B_IOMUX 	(0x00ac)
#define GRF_GPIO0C_IOMUX 	(0x00b0)
#define GRF_GPIO0D_IOMUX 	(0x00b4)
#define GRF_GPIO1A_IOMUX 	(0x00b8)
#define GRF_GPIO1B_IOMUX 	(0x00bc)
#define GRF_GPIO1C_IOMUX 	(0x00c0)
#define GRF_GPIO1D_IOMUX 	(0x00c4)
#define GRF_GPIO2A_IOMUX 	(0x00c8)
#define GRF_GPIO2B_IOMUX 	(0x00cc)
#define GRF_GPIO2C_IOMUX 	(0x00d0)
#define GRF_GPIO2D_IOMUX 	(0x00d4)
#define GRF_GPIO3A_IOMUX 	(0x00d8)
#define GRF_GPIO3B_IOMUX 	(0x00dc)
#define GRF_GPIO3C_IOMUX 	(0x00e0)
#define GRF_GPIO3D_IOMUX 	(0x00e4)

#define GPIO_SWPORTA_DR_OFFSET		0x0000
#define	GPIO_SWPORTA_DDR_OFFSET		0x0004
#define GPIO_INTEN_OFFSET		0x0030		
#define GPIO_INTMASK_OFFSET		0x0034
#define GPIO_INTTYPE_LEVEL_OFFSET	0x0038
#define GPIO_INT_POLARITY_OFFSET	0x003c
#define GPIO_INT_STATUS_OFFSET		0x0040
#define GPIO_INT_RAWSTATUS_OFFSET	0x0044
#define GPIO_DEBOUNCE_OFFSET		0x0048	
#define GPIO_PORTA_EOF_OFFSET		0x004c
#define GPIO_EXT_PORTA_OFFSET		0x0050
#define GPIO_LS_SYNC_OFFSET		0x0060

#define RK3128_PWM 			0x20050000
#define RK3128_PWM0_CNT			0x0000
#define RK3128_PWM0_PERIOD		0x0004
#define RK3128_PWM0_DUTY		0x0008
#define RK3128_PWM0_CTR			0x000c
#define RK3128_PWM1_CNT			0x0010
#define RK3128_PWM1_PERIOD		0x0014
#define RK3128_PWM1_DUTY		0x0018
#define RK3128_PWM1_CTR			0x001c
#define RK3128_PWM2_CNT			0x0020
#define RK3128_PWM2_PERIOD		0x0024
#define RK3128_PWM2_DUTY		0x0028
#define RK3128_PWM2_CTR			0x002c
#define RK3128_PWM3_CNT			0x0030
#define RK3128_PWM3_PERIOD		0x0034
#define RK3128_PWM3_DUTY		0x0038
#define RK3128_PWM3_CTR			0x003c

#define PWM0 26
#define PWM1 27
#define PWM2 28
#define PWM3 128
*/
static int physToGpio_AP [64] =
{
   -1,          		// 0
   -1,  	-1,     	//1, 2
   GPIO8_A4,  	-1,     	//3, 4
   GPIO8_A5,  	-1,     	//5, 6
   GPIO0_C1,  	GPIO5_B1,     	//7, 8
   -1,  	GPIO5_B0,     	//9, 10
   GPIO5_B4,  	GPIO6_A0,     	//11, 12
   GPIO5_B6,  	-1,     	//13, 14
   GPIO5_B7,  	GPIO5_B2,     	//15, 16
   -1,  	GPIO5_B3,     	//17, 18
   GPIO8_B1,  	-1,     	//19, 20
   GPIO8_B0,  	GPIO5_C3,     	//21, 22
   GPIO8_A6,  	GPIO8_A7,     	//23, 24
   -1, 		GPIO8_A3,     	//25, 26
   GPIO7_C1,   	GPIO7_C2,     	//27, 28
   GPIO5_B5,  	-1,     	//29, 30
   GPIO5_C0, 	GPIO7_C7,     	//31, 32      
   GPIO7_C6,  	-1,     	//33, 34
   GPIO6_A1, 	GPIO7_A7,     	//35, 36
   GPIO7_B0,  	GPIO6_A3,     	//37, 38
   -1,  	GPIO6_A4,     	//39, 40
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1, -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;

static int pinToGpio_AP [64] =
{
   GPIO5_B4,  	GPIO6_A0,        //0, 1
   GPIO5_B6,  	GPIO5_B7,        //2, 3
   GPIO5_B2,  	GPIO5_B3,        //4, 5
   GPIO5_C3,  	GPIO0_C1,        //6, 7
   GPIO8_A4,  	GPIO8_A5,        //8, 9
   GPIO8_A7,  	GPIO8_A3,        //10, 11
   GPIO8_B1,  	GPIO8_B0,        //12, 13
   GPIO8_A6,  	GPIO5_B1,        //14, 15
   GPIO5_B0,  	-1,              //16, 17
   -1,  	-1,        	 //18, 19
   -1,  	GPIO5_B5,        //20, 21
   GPIO5_C0,   	GPIO7_C6,        //22, 23
   GPIO6_A1,  	GPIO7_B0,        //24, 25
   GPIO7_C7, 	GPIO7_A7,        //26, 27
   GPIO6_A3,  	GPIO6_A4,        //28. 29
   GPIO7_C1,   	GPIO7_C2,        //30, 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;





/*
 * wiringPiFailure:
 *	Fail. Or not.
 *********************************************************************************
 */

int wiringPiFailure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}


/*
 * piBoardRev:
 *	Return a number representing the hardware revision of the board.
 *	This is not strictly the board revision but is used to check the
 *	layout of the GPIO connector - and there are 2 types that we are
 *	really interested in here. The very earliest Pi's and the
 *	ones that came after that which switched some pins ....
 *
 *	Revision 1 really means the early Model A and B's.
 *	Revision 2 is everything else - it covers the B, B+ and CM.
 *		... and the Pi 2 - which is a B+ ++  ...
 *		... and the Pi 0 - which is an A+ ...
 *
 *	The main difference between the revision 1 and 2 system that I use here
 *	is the mapping of the GPIO pins. From revision 2, the Pi Foundation changed
 *	3 GPIO pins on the (original) 26-way header - BCM_GPIO 22 was dropped and
 *	replaced with 27, and 0 + 1 - I2C bus 0 was changed to 2 + 3; I2C bus 1.
 *
 *********************************************************************************
 */

static void piBoardRevOops (const char *why)
{
  fprintf (stderr, "piBoardRev: Unable to determine board revision from /proc/cpuinfo\n") ;
  fprintf (stderr, " -> %s\n", why) ;
  fprintf (stderr, " ->  You may want to check:\n") ;
  fprintf (stderr, " ->  http://www.raspberrypi.org/phpBB3/viewtopic.php?p=184410#p184410\n") ;
  exit (EXIT_FAILURE) ;
}

int asuspi(void)
{
  FILE *cpuFd ;
  char line [120] ;
  char *d;
	//printf("asuspi\n");
  if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
    piBoardRevOops ("Unable to open /proc/cpuinfo") ;

  while (fgets (line, 120, cpuFd) != NULL)
  {
    if (strncmp (line, "Hardware", 8) == 0)
    break ;
  }
		
	fclose (cpuFd) ;
	if (strncmp (line, "Hardware", 8) != 0)
		piBoardRevOops ("No \"Hardware\" line") ;
	
  for (d = &line [strlen (line) - 1] ; (*d == '\n') || (*d == '\r') ; --d)
    *d = 0 ;

  if (wiringPiDebug)
  printf ("piboardRev: Hardware string: %s\n", line) ;
	
  if (strstr(line,"Rockchip (Device Tree)") != NULL)
  {
		if (wiringPiDebug)
	printf ("Hardware:%s\n",line) ;
	return 1 ;
  }
  else
  {
		if (wiringPiDebug)
	printf ("Hardware:%s\n",line) ;
	return 0 ;
  }
}//int asuspi(void)

int isAsusPiGpioPin40(int pin) {
  int i;

  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS)){
    if ((pin >= 0) && (pin <= 63))
      return 1;
  } else if ((wiringPiMode == WPI_MODE_GPIO) || (wiringPiMode == WPI_MODE_GPIO_SYS)) {
  for (i = 0; i < 64; i++)
    if((pinToGpio_AP[i] == pin) && (pin > 0))
      return 1;
  }
  return 0;
}

int asus_get_pin_mode(int pin)
{
	int value,func;
	/*
	value = ((*(grf+GRF_GPIO0A_IOMUX/4+pin/8))>>((pin%8)*2)) & 0x0003;
	printf("value = %x\n",value);
	if(value == 0)
	{
		value = ((*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4))>>(pin%32)) & 0x00000001;
	}
	else
	{
		value = 4;
	}
	*/
	if (wiringPiDebug)
	printf("pin = %d\n",pin);
	switch(pin)
	{
		//GPIO0
		//case 17 : value = ((*(grf+GRF_GPIO0A_IOMUX/4+gpio/8))>>((gpio%8)*2)) & 0x00000003;  break;
		case 17 : value =  ((*(pmu+PMU_GPIO0C_IOMUX/4))>>((pin%8)*2)) & 0x00000003;  
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=CLKOUT;		break;
				case 2: func=CLK1_27M;	break;
				default: func=-1;		break;
			}
			break;			
		//GPIO5B
		case 160 : 
		case 161 :
		case 162 :
		case 163 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO5B_IOMUX/4));
			value = ((*(grf+GRF_GPIO5B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SERIAL;	break;
				case 2: func=TS_XXXX;	break;
				case 3: func=RESERVED;	break;
				default: func=-1;		break;
			}
			break;
		case 164 :
		case 165 :
		case 166 :
		case 167 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO5B_IOMUX/4));
			value = ((*(grf+GRF_GPIO5B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SPI;		break;
				case 2: func=TS_XXXX;	break;
				case 3: func=SERIAL;	break;
				default: func=-1;		break;
			}
			break;
		
		//GPIO5C
		case 168 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO5C_IOMUX/4));
			value = ((*(grf+GRF_GPIO5C_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SPI;		break;
				case 2: func=TS_XXXX;	break;
				case 3: func=RESERVED;	break;
				default: func=-1;		break;
			}
			break;
		case 169 :
		case 170 :
		case 171 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO5C_IOMUX/4));
			value = ((*(grf+GRF_GPIO5C_IOMUX/4))>>((pin%8)*2)) & 0x00000001;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=TS_XXXX;		break;
				default: func=-1;		break;
			}
			break;

		//GPIO6A
		case 184 : 
		case 185 :
		case 187 :
		case 188 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO6A_IOMUX/4));
			value = ((*(grf+GRF_GPIO6A_IOMUX/4))>>((pin%8)*2)) & 0x00000001;	
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=I2S;		break;
				default: func=-1;		break;
			}
			break;

		//GPIO7A7
		case 223 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO7A_IOMUX/4));
			value = ((*(grf+GRF_GPIO7A_IOMUX/4))>>((pin%8)*2)) & 0x00000003; 
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SERIAL;		break;
				case 2: func=GPS_MAG;	break;
				case 3: func=HSADCT;	break;
				default: func=-1;		break;
			}
			break;

		//GPIO7B
		case 224 : 
		case 225 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO7B_IOMUX/4));
			value = ((*(grf+GRF_GPIO7B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SERIAL;	break;
				case 2: func=GPS_MAG;	break;
				case 3: func=HSADCT;	break;
				default: func=-1;		break;
			}
			break;
		case 226 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO7B_IOMUX/4));
			value = ((*(grf+GRF_GPIO7B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SERIAL;	break;
				case 2: func=USB;		break;
				case 3: func=RESERVED;	break;
				default: func=-1;		break;
			}
			break;
		//GPIO7C
		case 233 : 
		case 234 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO7CL_IOMUX/4));
			value = ((*(grf+GRF_GPIO7CL_IOMUX/4))>>((pin%8)*2)) & 0x00000001;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=I2C;		break;
				default: func=-1;		break;
			}
			break;
		case 238 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO7CH_IOMUX/4));
			value = ((*(grf+GRF_GPIO7CH_IOMUX/4))>>(((pin-4)%8)*4)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SERIAL;	break;
				case 2: func=SERIAL;	break;
				case 3: func=PWM;		break;
				default: func=-1;		break;
			}
			break;
		case 239 : 
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO7CH_IOMUX/4));
			value = ((*(grf+GRF_GPIO7CH_IOMUX/4))>>(((pin-4)%8)*4)) & 0x00000007;
			switch(value)
			{
				case 0:
					func=GPIO;		break;
				case 1: 
				case 2:
					func=SERIAL;		break;
				case 3:
					func=PWM;			break;
				case 4:
					func=HDMI;			break;
				case 5:
				case 6:
				case 7:
					func=RESERVED;		break;
				default: 
					func=-1;		break;
			}
			break;

		//GPIO8A
		case 251 : 
		case 254 :
		case 255 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO8A_IOMUX/4));
			value = ((*(grf+GRF_GPIO8A_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SPI;		break;
				case 2: func=SC_XXX;	break;
				case 3: func=RESERVED;	break;
				default: func=-1;		break;
			}
			break;
		case 252 : 
		case 253 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO8A_IOMUX/4));
			value = ((*(grf+GRF_GPIO8A_IOMUX/4))>>((pin%8)*2)) & 0x00000003;  
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=I2C;		break;
				case 2: func=SC_XXX;	break;
				case 3: func=RESERVED;	break;
				default: func=-1;		break;
			}
			break;

		//GPIO8B
		case 256 : 
		case 257 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO8B_IOMUX/4));
			value = ((*(grf+GRF_GPIO8B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=SPI;		break;
				case 2: func=SC_XXX;	break;
				case 3: func=RESERVED;	break;
				default: func=-1;		break;
			}
			break;
		default:
			func=-1; break;
	}

    if (func == GPIO)
    {
      if(pin>=24)
      {
        if (*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) && 1<<((pin+8)%32))
          func = GPIOOUT;
        else
          func = GPIOIN;
      }
      else
      {
        if (*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) && 1<<(pin%32))
          func = GPIOOUT;
        else
          func = GPIOIN;
      }
    }

	if (wiringPiDebug)
	printf("hjptestfor:gpio=%d,func=%d\n",pin,func);
	return func;



	//return value;
}

void asus_set_pinmode_as_gpio(int pin)
{
	switch(pin)
	{
		//GPIO0
		case 17 : 
			*(pmu+PMU_GPIO0C_IOMUX/4) = (*(pmu+PMU_GPIO0C_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)));
			if (wiringPiDebug)
			printf("pmu = 0x%x\n",*(pmu+PMU_GPIO0C_IOMUX/4));
			break;
		//GPIO1D0:act-led
		case 48 :
			break;
		//GPIO5B
		case 160 : 
		case 161 :
		case 162 :
		case 163 :			
		case 164 :
		case 165 :
		case 166 :
		case 167 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO5B_IOMUX/4));
			*(grf+GRF_GPIO5B_IOMUX/4) = (*(grf+GRF_GPIO5B_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)));
			break;
		
		//GPIO5C
		case 168 : 
		case 169 :
		case 170 :
		case 171 :
			*(grf+GRF_GPIO5C_IOMUX/4) =  (*(grf+GRF_GPIO5C_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)));
			break;

		//GPIO6A
		case 184 : 
		case 185 :
		case 187 :
		case 188 :
			*(grf+GRF_GPIO6A_IOMUX/4) =  (*(grf+GRF_GPIO6A_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)));
			break;

		//GPIO7A7
		case 223 : 
			*(grf+GRF_GPIO7A_IOMUX/4) =  (*(grf+GRF_GPIO7A_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))); 
			break;

		//GPIO7B
		case 224 : 
		case 225 : 
		case 226 : 
			*(grf+GRF_GPIO7B_IOMUX/4) =  (*(grf+GRF_GPIO7B_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))); 
			break;
		//GPIO7C
		case 233 : 
		case 234 : 
			*(grf+GRF_GPIO7CL_IOMUX/4) = (*(grf+GRF_GPIO7CL_IOMUX/4) | (0x0f<<(16+(pin%8)*4))) & (~(0x0f<<((pin%8)*4))); 
			break;
		case 238 : 			
		case 239 : 
			*(grf+GRF_GPIO7CH_IOMUX/4) =  (*(grf+GRF_GPIO7CH_IOMUX/4) | (0x0f<<(16+(pin%8-4)*4))) & (~(0x0f<<((pin%8-4)*4)));  
			break;

		//GPIO8A
		case 251 : 
		case 254 :
		case 255 :			
		case 252 : 
		case 253 :
			*(grf+GRF_GPIO8A_IOMUX/4) =  (*(grf+GRF_GPIO8A_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))); 
			break;
		//GPIO8B
		case 256 : 
		case 257 :
			*(grf+GRF_GPIO8B_IOMUX/4) = (*(grf+GRF_GPIO8B_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))); 
			break;
		default:
			printf("wrong gpio\n");
	}	//switch(pin)

}

int asus_set_pin_mode(int pin, int mode)
{
  if(INPUT == mode)
  {
    asus_set_pinmode_as_gpio(pin);	
    if(pin>=24)
    {
      *(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) &= ~(1<<((pin+8)%32));
    }
    else
    {
      *(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) &= ~(1<<(pin%32));
    }
    //if (wiringPiDebug)
      //printf("Input mode set over reg val: 0x%x\n",*(grf+GRF_GPIO0A_IOMUX/4+pin/8));
  }
  else if(OUTPUT == mode)
  {
    asus_set_pinmode_as_gpio(pin);
    if(pin>=24)
    {
      *(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) |= (1<<((pin+8)%32));
    }
    else
    {
      *(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) |= (1<<(pin%32));
    }
    //if (wiringPiDebug)
      //printf("Out mode ready set val: 0x%x\n",*(grf+GRF_GPIO0A_IOMUX/4+pin/8));

    //if (wiringPiDebug)
      //printf("Out mode set over reg val: 0x%x\n",*(grf+GRF_GPIO0A_IOMUX/4+pin/8));
  } 
  else if(PWM_OUTPUT == mode)
  {
    //set pin PWMx to pwm mode
    if(pin == PWM2)
    {
      *(grf+GRF_GPIO7CH_IOMUX/4) =  (*(grf+GRF_GPIO7CH_IOMUX/4) | (0x0f<<(16+(pin%8-4)*4))) | (0x03<<((pin%8-4)*4));
      if (wiringPiDebug)
        printf("grf data2 = 0x%x\n",*(grf+GRF_GPIO7CH_IOMUX/4));
    }
    else if(pin == PWM3)
    {
      *(grf+GRF_GPIO7CH_IOMUX/4) =  (*(grf+GRF_GPIO7CH_IOMUX/4) | (0x0f<<(16+(pin%8-4)*4))) | (0x03<<((pin%8-4)*4));

      if (wiringPiDebug)
        printf("grf data3 = 0x%x\n",*(grf+GRF_GPIO7CH_IOMUX/4));
    }
    else
    {
      printf("This pin cannot set as pwm out\n");
    }	
    //*(grf+GRF_GPIO0A_IOMUX/4+pin/8) = (*(grf+GRF_GPIO0A_IOMUX/4+pin/8) | (0x03<<((pin%8)*2+16))) | ((0x03<<((pin%8)*2)));
  }
  else if(GPIO_CLOCK == mode)
  {
    if(pin == 17)
    {
      *(pmu+PMU_GPIO0C_IOMUX/4) = (*(pmu+PMU_GPIO0C_IOMUX/4) & (~(0x03<<((pin%8)*2)))) | (0x01<<((pin%8)*2));
      if (wiringPiDebug)
        printf("pmu = 0x%x\n",*(pmu+PMU_GPIO0C_IOMUX/4));
    }
    else
      printf("This pin cannot set as gpio clock\n");
  }
}

void asus_digitalWrite(int pin, int value)
{
	if (wiringPiDebug)
	printf("pin = %d\n",pin);
	if(value==1)
	{
		if(pin>=24)
		{
			*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DR_OFFSET/4) |= (1<<((pin-24)%32));
		}
		else
		{
			*(gpio0[pin/32]+GPIO_SWPORTA_DR_OFFSET/4) |= (1<<(pin%32));
		}	
	}
	else
	{
		if(pin>=24)
		{
			*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DR_OFFSET/4) &= ~(1<<((pin-24)%32));
		}
		else
		{
			*(gpio0[pin/32]+GPIO_SWPORTA_DR_OFFSET/4) &= ~(1<<(pin%32));
		}
			
	}
	if (wiringPiDebug)		
	printf("gpio = %d , output value = %d\n",pin,value);
}

int asus_digitalRead(int pin)
{
	int value, mask;
	
	
	if(pin>=24)
	{
		mask = (1 << (pin-24)%32);
		value = (((*(gpio0[(pin-24)/32+1]+GPIO_EXT_PORTA_OFFSET/4)) & mask)>>(pin-24)%32); 
	}
	else
	{
		mask = (1 << pin%32);
		value = (((*(gpio0[pin/32]+GPIO_EXT_PORTA_OFFSET/4)) & mask)>>pin%32);
	}	
	if (wiringPiDebug)	
	printf("gpio = %d,input value = %x\n",pin,value);
   return value;
}

void asus_pullUpDnControl (int pin, int pud)
{
	static int bit0,bit1;
	//printf("pullupdn %d\n",pud);//pud 2:up 1:down 0:off
	if(pud == 2)
	{
		bit0 = 1;
		bit1 = 0;
	}
	else if(pud == 1)
	{
		bit0 = 0;
		bit1 = 1;
	}
	else
	{
		bit0 = 0;
		bit1 = 0;
	}
	
	switch(pin)
	{
		//GPIO0
		case 17 : 
			*(pmu+PMU_GPIO0C_P/4) = (*(grf+PMU_GPIO0C_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
//			printf("pmu = 0x%x\n",*(pmu+PMU_GPIO0C_IOMUX/4));
			break;
		//case 17 : value =  0x00000003;  break;
		//GPIO5B
		case 160 : 
		case 161 :
		case 162 :
		case 163 :			
		case 164 :
		case 165 :
		case 166 :
		case 167 :
			//printf("hjptestfor:value=0x%x\n",*(grf+GRF_GPIO5B_IOMUX/4));
			*(grf+GRF_GPIO5B_P/4) = (*(grf+GRF_GPIO5B_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
			break;
		
		//GPIO5C
		case 168 : 
		case 169 :
		case 170 :
		case 171 :
			*(grf+GRF_GPIO5C_P/4) = (*(grf+GRF_GPIO5C_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
			break;

		//GPIO6A
		case 184 : 
		case 185 :
		case 187 :
		case 188 :
			*(grf+GRF_GPIO6A_P/4) = (*(grf+GRF_GPIO6A_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
			//printf("pin = %d *(grf+GRF_GPIO6A_P/4) = %x\n",pin ,grf+GRF_GPIO6A_P/4);
			break;

		//GPIO7A7
		case 223 : 
			*(grf+GRF_GPIO7A_P/4) = (*(grf+GRF_GPIO7A_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;

		//GPIO7B
		case 224 : 
		case 225 : 
		case 226 : 
			*(grf+GRF_GPIO7B_P/4) = (*(grf+GRF_GPIO7B_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;
		//GPIO7C
		case 233 : 
		case 234 : 			
		case 238 : 			
		case 239 : 
			*(grf+GRF_GPIO7C_P/4) = (*(grf+GRF_GPIO7C_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));  
			break;

		//GPIO8A
		case 251 : 
		case 254 :
		case 255 :			
		case 252 : 
		case 253 :
			*(grf+GRF_GPIO8A_P/4) = (*(grf+GRF_GPIO8A_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;
		//GPIO8B
		case 256 : 
		case 257 :
			*(grf+GRF_GPIO8B_P/4) = (*(grf+GRF_GPIO8B_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;
		default:
			printf("wrong gpio\n");
	}	//switch(pin)

	
	/*rk3368
		if(pud==PUD_DOWN)
		{
			*(grf+RK3368_GRF_GPIO1A_P/4+pin/8) = (0x02<<((pin%8)*2)) | (0x03<<(16+(pin%8)*2));
		}
		else if (pud == PUD_UP)
		{
			*(grf+RK3368_GRF_GPIO1A_P/4+pin/8) = (0x01<<((pin%8)*2)) | (0x03<<(16+(pin%8)*2));
		}
		else //pub==PUD_OFF
		{
			*(grf+RK3368_GRF_GPIO1A_P/4+pin/8) = (0x03<<((pin%8)*2)) | (0x03<<(16+(pin%8)*2));
		}
	*/		

}

void asus_pwm_set_period(int pwm_ch,unsigned int range)
{
	if (wiringPiDebug)
	printf("jason asus_pwm_set_period\n");
	*(pwm+RK3288_PWM0_PERIOD/4+pwm_ch*4) = range;
}

void asus_pwm_set_duty(int pwm_ch,unsigned int duty)
{
	if (wiringPiDebug)
	printf("jason asus_pwm_set_duty\n");
	*(pwm+RK3288_PWM0_DUTY/4+pwm_ch*4) = duty;
}

void asus_pwm_enable(int pwm_ch)
{
	if (wiringPiDebug)
	printf("jason asus_pwm_enable\n");
	*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) |= (1<<0); 
}

void asus_pwm_disable(int pwm_ch)
{

	if(wiringPiDebug)
	printf("jason asus_pwm_disable\n");	
	*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) &= ~(1<<0); 
}

void asus_pwm_start(int pwm_ch,int mode,unsigned int range,unsigned int duty)
{
	int pin;
	switch (pwm_ch)
	{
		case 2:pin=PWM2;break;
		case 3:pin=PWM3;break;
		default:pin=0;break;
	}

	if(wiringPiDebug)
	printf("jason asus_pwm_start\n");

	if(asus_get_pin_mode(pin)==PWM)
	{
	
	
		asus_pwm_disable(pwm_ch);
		asus_pwm_set_period(pwm_ch,range);
		asus_pwm_set_duty(pwm_ch,range-duty);
		if(mode == CENTERPWM)
		{
			*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) |= (1<<5);		
		}
		else
		{
			*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) &= ~(1<<5);
		}
		*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) |= (1<<1);
		*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) &= ~(1<<2);
		*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) |= (1<<4);
		
		asus_pwm_enable(pwm_ch);
	}
	else
	{
		printf("please set this pin to pwmmode first\n");
	}
}

void asus_pwm_stop(int pwm_ch)
{
	asus_pwm_disable(pwm_ch);
}


int piBoardRev (void)
{
  FILE *cpuFd ;
  char line [120] ;
  char *c ;
  static int  boardRev = -1 ;

	
	if(asuspi())
	  {
		asusversion = ASUSVER;
			if (wiringPiDebug)
				printf ("piboardRev:  %d\n", asusversion) ;
			return ASUSVER ;
	  }


  if (boardRev != -1)	// No point checking twice
    return boardRev ;

  if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
    piBoardRevOops ("Unable to open /proc/cpuinfo") ;

// Start by looking for the Architecture to make sure we're really running
//	on a Pi. I'm getting fed-up with people whinging at me because
//	they can't get it to work on weirdFruitPi boards...

  while (fgets (line, 120, cpuFd) != NULL)
    if (strncmp (line, "Hardware", 8) == 0)
      break ;

  if (strncmp (line, "Hardware", 8) != 0)
    piBoardRevOops ("No hardware line") ;

  if (wiringPiDebug)
    printf ("piboardRev: Hardware: %s\n", line) ;

// See if it's BCM2708 or BCM2709

  if (strstr (line, "BCM2709") != NULL)	// Pi v2 - no point doing anything more at this point
  //if (strstr (line, "Rockchip RK3128") != NULL)	// Pi v2 - no point doing anything more at this point
  {
    piModel2 = TRUE ;
    fclose (cpuFd) ;
    return boardRev = 2 ;
  }
  else if (strstr (line, "BCM2708") == NULL)
  {
    fprintf (stderr, "Unable to determine hardware version. I see: %s,\n", line) ;
    fprintf (stderr, " - expecting BCM2708 or BCM2709.\n") ;
    fprintf (stderr, "If this is a genuine Raspberry Pi then please report this\n") ;
    fprintf (stderr, "to projects@drogon.net. If this is not a Raspberry Pi then you\n") ;
    fprintf (stderr, "are on your own as wiringPi is designed to support the\n") ;
    fprintf (stderr, "Raspberry Pi ONLY.\n") ;
    exit (EXIT_FAILURE) ;
  }

// Now do the rest of it as before - we just need to see if it's an older
//	Rev 1 as anything else is rev 2.

// Isolate the Revision line

  rewind (cpuFd) ;
  while (fgets (line, 120, cpuFd) != NULL)
    if (strncmp (line, "Revision", 8) == 0)
      break ;

  fclose (cpuFd) ;

  if (strncmp (line, "Revision", 8) != 0)
    piBoardRevOops ("No \"Revision\" line") ;

// Chomp trailing CR/NL

  for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
    *c = 0 ;
  
  if (wiringPiDebug)
    printf ("piboardRev: Revision string: %s\n", line) ;

// Scan to the first character of the revision number

  for (c = line ; *c ; ++c)
    if (*c == ':')
      break ;

  if (*c != ':')
    piBoardRevOops ("Bogus \"Revision\" line (no colon)") ;

// Chomp spaces

  ++c ;
  while (isspace (*c))
    ++c ;

  if (!isxdigit (*c))
    piBoardRevOops ("Bogus \"Revision\" line (no hex digit at start of revision)") ;

// Make sure its long enough

  if (strlen (c) < 4)
    piBoardRevOops ("Bogus revision line (too small)") ;

// If you have overvolted the Pi, then it appears that the revision
//	has 100000 added to it!
// The actual condition for it being set is:
//	 (force_turbo || current_limit_override || temp_limit>85) && over_voltage>0


// This test is not correct for the new encoding scheme, so we'll remove it here as
//	we don't really need it at this point.

/********************
  if (wiringPiDebug)
    if (strlen (c) != 4)
      printf ("piboardRev: This Pi has/is (force_turbo || current_limit_override || temp_limit>85) && over_voltage>0\n") ;
*******************/

// Isolate  last 4 characters:

  c = c + strlen (c) - 4 ;

  if (wiringPiDebug)
    printf ("piboardRev: last4Chars are: \"%s\"\n", c) ;

  if ( (strcmp (c, "0002") == 0) || (strcmp (c, "0003") == 0))
    boardRev = 1 ;
  else
    boardRev = 2 ;	// Covers everything else from the B revision 2 to the B+, the Pi v2 and CM's.

  if (wiringPiDebug)
    printf ("piBoardRev: Returning revision: %d\n", boardRev) ;

  return boardRev ;
}


/*
 * piBoardId:
 *	Return the real details of the board we have.
 *
 *	This is undocumented and really only intended for the GPIO command.
 *	Use at your own risk!
 *
 *	Seems there are some boards with 0000 in them (mistake in manufacture)
 *	So the distinction between boards that I can see is:
 *
 *		0000 - Error
 *		0001 - Not used 
 *
 *	Original Pi boards:
 *		0002 - Model B,  Rev 1,   256MB, Egoman
 *		0003 - Model B,  Rev 1.1, 256MB, Egoman, Fuses/D14 removed.
 *
 *	Newer Pi's with remapped GPIO:
 *		0004 - Model B,  Rev 2,   256MB, Sony
 *		0005 - Model B,  Rev 2,   256MB, Qisda
 *		0006 - Model B,  Rev 2,   256MB, Egoman
 *		0007 - Model A,  Rev 2,   256MB, Egoman
 *		0008 - Model A,  Rev 2,   256MB, Sony
 *		0009 - Model A,  Rev 2,   256MB, Qisda
 *		000d - Model B,  Rev 2,   512MB, Egoman	(Red Pi, Blue Pi?)
 *		000e - Model B,  Rev 2,   512MB, Sony
 *		000f - Model B,  Rev 2,   512MB, Qisda
 *		0010 - Model B+, Rev 1.2, 512MB, Sony
 *		0011 - Pi CM,    Rev 1.2, 512MB, Sony
 *		0012 - Model A+  Rev 1.2, 256MB, Sony
 *		0014 - Pi CM,    Rev 1.1, 512MB, Sony (Actual Revision might be different)
 *		0015 - Model A+  Rev 1.1, 256MB, Sony
 *
 *	A small thorn is the olde style overvolting - that will add in
 *		1000000
 *
 *	The Pi compute module has an revision of 0011 or 0014 - since we only
 *	check the last digit, then it's 1, therefore it'll default to not 2 or
 *	3 for a	Rev 1, so will appear as a Rev 2. This is fine for the most part, but
 *	we'll properly detect the Compute Module later and adjust accordingly.
 *
 * And then things changed with the introduction of the v2...
 *
 * For Pi v2 and subsequent models - e.g. the Zero:
 *
 *   [USER:8] [NEW:1] [MEMSIZE:3] [MANUFACTURER:4] [PROCESSOR:4] [TYPE:8] [REV:4]
 *   NEW          23: will be 1 for the new scheme, 0 for the old scheme
 *   MEMSIZE      20: 0=256M 1=512M 2=1G
 *   MANUFACTURER 16: 0=SONY 1=EGOMAN 2=EMBEST
 *   PROCESSOR    12: 0=2835 1=2836
 *   TYPE         04: 0=MODELA 1=MODELB 2=MODELA+ 3=MODELB+ 4=Pi2 MODEL B 5=ALPHA 6=CM
 *   REV          00: 0=REV0 1=REV1 2=REV2
 *********************************************************************************
 */

void piBoardId (int *model, int *rev, int *mem, int *maker, int *warranty)
{
  FILE *cpuFd ;
  char line [120] ;
  char *c ;
  unsigned int revision ;
  int bRev, bType, bProc, bMfg, bMem, bWarranty ;

//	Will deal with the properly later on - for now, lets just get it going...
//  unsigned int modelNum ;

  (void)piBoardRev () ;	// Call this first to make sure all's OK. Don't care about the result.

  if ((cpuFd = fopen ("/proc/cpuinfo", "r")) == NULL)
    piBoardRevOops ("Unable to open /proc/cpuinfo") ;

  while (fgets (line, 120, cpuFd) != NULL)
    if (strncmp (line, "Revision", 8) == 0)
      break ;

  fclose (cpuFd) ;

  if (strncmp (line, "Revision", 8) != 0)
    piBoardRevOops ("No \"Revision\" line") ;

// Chomp trailing CR/NL

  for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
    *c = 0 ;
  
  if (wiringPiDebug)
    printf ("piboardId: Revision string: %s\n", line) ;

// Need to work out if it's using the new or old encoding scheme:

// Scan to the first character of the revision number

  for (c = line ; *c ; ++c)
    if (*c == ':')
      break ;

  if (*c != ':')
    piBoardRevOops ("Bogus \"Revision\" line (no colon)") ;

// Chomp spaces

  ++c ;
  while (isspace (*c))
    ++c ;

  if (!isxdigit (*c))
    piBoardRevOops ("Bogus \"Revision\" line (no hex digit at start of revision)") ;

  revision = (unsigned int)strtol (c, NULL, 16) ; // Hex number with no leading 0x

// Check for new way:

  if ((revision &  (1 << 23)) != 0)	// New way
  {
    if (wiringPiDebug)
      printf ("piBoardId: New Way: revision is: 0x%08X\n", revision) ;

    bRev      = (revision & (0x0F <<  0)) >>  0 ;
    bType     = (revision & (0xFF <<  4)) >>  4 ;
    bProc     = (revision & (0x0F << 12)) >> 12 ;	// Not used for now.
    bMfg      = (revision & (0x0F << 16)) >> 16 ;
    bMem      = (revision & (0x07 << 20)) >> 20 ;
    bWarranty = (revision & (0x03 << 24)) != 0 ;
    
    *model    = bType ;
    *rev      = bRev ;
    *mem      = bMem ;
    *maker    = bMfg  ;
    *warranty = bWarranty ;

    if (wiringPiDebug)
      printf ("piboardId: rev: %d, type: %d, proc: %d, mfg: %d, mem: %d, warranty: %d\n",
		bRev, bType, bProc, bMfg, bMem, bWarranty) ;
  }
  else					// Old way
  {
    if (wiringPiDebug)
      printf ("piBoardId: Old Way: revision is: %s\n", c) ;

    if (!isdigit (*c))
      piBoardRevOops ("Bogus \"Revision\" line (no digit at start of revision)") ;

// Make sure its long enough

    if (strlen (c) < 4)
      piBoardRevOops ("Bogus \"Revision\" line (not long enough)") ;

// If longer than 4, we'll assume it's been overvolted

    *warranty = strlen (c) > 4 ;
  
// Extract last 4 characters:

    c = c + strlen (c) - 4 ;
	 if (wiringPiDebug)
      printf ("piBoardId: 2: revision is: %s\n", c) ;
// Fill out the replys as appropriate

    /**/ if (strcmp (c, "0002") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_1   ; *mem = 0 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "0003") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_1_1 ; *mem = 0 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "0004") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 0 ; *maker = PI_MAKER_SONY    ; }
    else if (strcmp (c, "0005") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 0 ; *maker = PI_MAKER_UNKNOWN ; }
    else if (strcmp (c, "0006") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 0 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "0007") == 0) { *model = PI_MODEL_A  ; *rev = PI_VERSION_2   ; *mem = 0 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "0008") == 0) { *model = PI_MODEL_A  ; *rev = PI_VERSION_2   ; *mem = 0 ; *maker = PI_MAKER_SONY ;  ; }
    else if (strcmp (c, "0009") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 0 ; *maker = PI_MAKER_UNKNOWN ; }
    else if (strcmp (c, "000d") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 1 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "000e") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 1 ; *maker = PI_MAKER_SONY    ; }
    else if (strcmp (c, "000f") == 0) { *model = PI_MODEL_B  ; *rev = PI_VERSION_2   ; *mem = 1 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "0010") == 0) { *model = PI_MODEL_BP ; *rev = PI_VERSION_1_2 ; *mem = 1 ; *maker = PI_MAKER_SONY    ; }
    else if (strcmp (c, "0011") == 0) { *model = PI_MODEL_CM ; *rev = PI_VERSION_1_2 ; *mem = 1 ; *maker = PI_MAKER_SONY    ; }
    else if (strcmp (c, "0012") == 0) { *model = PI_MODEL_AP ; *rev = PI_VERSION_1_2 ; *mem = 0 ; *maker = PI_MAKER_SONY    ; }
    else if (strcmp (c, "0013") == 0) { *model = PI_MODEL_BP ; *rev = PI_VERSION_1_2 ; *mem = 1 ; *maker = PI_MAKER_EGOMAN  ; }
    else if (strcmp (c, "0014") == 0) { *model = PI_MODEL_CM ; *rev = PI_VERSION_1_2 ; *mem = 1 ; *maker = PI_MAKER_SONY    ; }
    else if (strcmp (c, "0015") == 0) { *model = PI_MODEL_AP ; *rev = PI_VERSION_1_1 ; *mem = 0 ; *maker = PI_MAKER_SONY    ; }
	else if (strcmp (c, "0000") == 0) { *model = PI_MODEL_ASUSPI;  *rev = PI_VERSION_1_2;  *mem = 2048;  *maker = PI_MAKER_ASUS;}
	else                              { *model = 0           ; *rev = 0              ; *mem =   0 ; *maker = 0 ;               }
  }
}
 


/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int wpiPinToGpio (int wpiPin)
{
  return pinToGpio [wpiPin & 63] ;
}


/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int physPinToGpio (int physPin)
{
  return physToGpio [physPin & 63] ;
}


/*
 * setPadDrive:
 *	Set the PAD driver value
 *********************************************************************************
 */

void setPadDrive (int group, int value)
{
  uint32_t wrVal ;

  if(asusversion == ASUSVER)
  {
    printf("setPadDrive is not available for ASUS Tinker.\n");
    return;
  }

  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    if (RASPBERRY_PI_PERI_BASE == 0)	// Ignore for now
      return ;

    if ((group < 0) || (group > 2))
      return ;

    wrVal = BCM_PASSWORD | 0x18 | (value & 7) ;
    *(pads + group + 11) = wrVal ;

    if (wiringPiDebug)
    {
      printf ("setPadDrive: Group: %d, value: %d (%08X)\n", group, value, wrVal) ;
      printf ("Read : %08X\n", *(pads + group + 11)) ;
    }
  }
}


/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */

int getAlt (int pin)
{
  int fSel, shift, alt ;

  if(asusversion == ASUSVER)
  {
    printf("getAlt is not available for ASUS Tinker. Maybe you can use getPinMode.\n");
    return 0;
  }
  else
  {

  pin &= 63 ;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return 0 ;

  fSel    = gpioToGPFSEL [pin] ;
  shift   = gpioToShift  [pin] ;

  alt = (*(gpio + fSel) >> shift) & 7 ;

  return alt ;
  }
}

/*
 * getPinMode:
 *      Returns the mode for a given port.
 *********************************************************************************
 */

int getPinMode (int pin)
{
  int fsel;
  if(asusversion == ASUSVER)
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return 0 ;

    fsel = asus_get_pin_mode(pin) ;

    return fsel;
  }
  else
  {
    printf("This func is only for Asus Tinker.");
    return 0;
  }
}

/*
 * pwmSetMode:
 *	Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */

void pwmSetMode (int mode)
{
  if(asusversion == ASUSVER)
  {
    printf("We only have Mark:space mode\n");
  }
  else
  {
    if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
    {
      if (mode == PWM_MODE_MS)
        *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE | PWM0_MS_MODE | PWM1_MS_MODE ;
      else
        *(pwm + PWM_CONTROL) = PWM0_ENABLE | PWM1_ENABLE ;
    }
  }
}


/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 *********************************************************************************
 */

void pwmSetRange (unsigned int range)
{
  if(asusversion == ASUSVER)
  {
    if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
      pwm_range = range;
  }
  else
  {
    if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
    {
      if (RASPBERRY_PI_PERI_BASE == 0)	// Ignore for now
        return ;

      *(pwm + PWM0_RANGE) = range ; delayMicroseconds (10) ;
      *(pwm + PWM1_RANGE) = range ; delayMicroseconds (10) ;
    }
  }
}


/*
 * pwmSetClock:
 *	Set/Change the PWM clock. Originally my code, but changed
 *	(for the better!) by Chris Hall, <chris@kchall.plus.com>
 *	after further study of the manual and testing with a 'scope
 *********************************************************************************
 */

void pwmSetClock (int divisor)
{
  uint32_t pwm_control ;
  divisor &= 4095 ;
  if(asusversion == ASUSVER)
  {
    if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
      pwm_divisor = divisor;
  }
  else
  {
    if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
    {
      if (RASPBERRY_PI_PERI_BASE == 0)	// Ignore for now
        return ;

      if (wiringPiDebug)
        printf ("Setting to: %d. Current: 0x%08X\n", divisor, *(clk + PWMCLK_DIV)) ;

      pwm_control = *(pwm + PWM_CONTROL) ;		// preserve PWM_CONTROL

      // We need to stop PWM prior to stopping PWM clock in MS mode otherwise BUSY
      // stays high.

      *(pwm + PWM_CONTROL) = 0 ;				// Stop PWM

// Stop PWM clock before changing divisor. The delay after this does need to
// this big (95uS occasionally fails, 100uS OK), it's almost as though the BUSY
// flag is not working properly in balanced mode. Without the delay when DIV is
// adjusted the clock sometimes switches to very slow, once slow further DIV
// adjustments do nothing and it's difficult to get out of this mode.

      *(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x01 ;	// Stop PWM Clock
        delayMicroseconds (110) ;			// prevents clock going sloooow

      while ((*(clk + PWMCLK_CNTL) & 0x80) != 0)	// Wait for clock to be !BUSY
        delayMicroseconds (1) ;

      *(clk + PWMCLK_DIV)  = BCM_PASSWORD | (divisor << 12) ;

      *(clk + PWMCLK_CNTL) = BCM_PASSWORD | 0x11 ;	// Start PWM clock
      *(pwm + PWM_CONTROL) = pwm_control ;		// restore PWM_CONTROL

      if (wiringPiDebug)
        printf ("Set     to: %d. Now    : 0x%08X\n", divisor, *(clk + PWMCLK_DIV)) ;
    }
  }
}


/*
 * gpioClockSet:
 *	Set the freuency on a GPIO clock pin
 *********************************************************************************
 */

void gpioClockSet (int pin, int freq)
{
  int divi, divr, divf ;

if(asusversion == ASUSVER)
  {
  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return ;

  if(pin != 17)
  {
     printf("This pin cannot set as gpio clock\n");
     return;
  }

  divi = 297000000 / freq ;
  divr = 297000000 % freq ;

  if (divi > 31)
    divi = 31 ;


  *(cru+CRU_CLKSEL2_CON/4) = (*(cru+CRU_CLKSEL2_CON/4) & (~(0x1F<<8))) | divi<<(8+16) | (divi<<8);
      if (wiringPiDebug)
        printf("cru = 0x%x\n",*(cru+CRU_CLKSEL2_CON/4));

  }
else{
  pin &= 63 ;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return ;
  
  if (RASPBERRY_PI_PERI_BASE == 0)	// Ignore for now
    return ;

  divi = 19200000 / freq ;
  divr = 19200000 % freq ;
  divf = (int)((double)divr * 4096.0 / 19200000.0) ;

  if (divi > 4095)
    divi = 4095 ;

  *(clk + gpioToClkCon [pin]) = BCM_PASSWORD | GPIO_CLOCK_SOURCE ;		// Stop GPIO Clock
  while ((*(clk + gpioToClkCon [pin]) & 0x80) != 0)				// ... and wait
    ;

  *(clk + gpioToClkDiv [pin]) = BCM_PASSWORD | (divi << 12) | divf ;		// Set dividers
  *(clk + gpioToClkCon [pin]) = BCM_PASSWORD | 0x10 | GPIO_CLOCK_SOURCE ;	// Start Clock
}
}


/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  while (node != NULL)
    if ((pin >= node->pinBase) && (pin <= node->pinMax))
      return node ;
    else
      node = node->next ;

  return NULL ;
}


/*
 * wiringPiNewNode:
 *	Create a new GPIO node into the wiringPi handling system
 *********************************************************************************
 */

static void pinModeDummy             (struct wiringPiNodeStruct *node, int pin, int mode)  { return ; }
static void pullUpDnControlDummy     (struct wiringPiNodeStruct *node, int pin, int pud)   { return ; }
static int  digitalReadDummy         (struct wiringPiNodeStruct *node, int pin)            { return LOW ; }
static void digitalWriteDummy        (struct wiringPiNodeStruct *node, int pin, int value) { return ; }
static void pwmWriteDummy            (struct wiringPiNodeStruct *node, int pin, int value) { return ; }
static int  analogReadDummy          (struct wiringPiNodeStruct *node, int pin)            { return 0 ; }
static void analogWriteDummy         (struct wiringPiNodeStruct *node, int pin, int value) { return ; }

struct wiringPiNodeStruct* wiringPiNewNode (int pinBase, int numPins)
{
  int    pin ;
  struct wiringPiNodeStruct *node ;

// Minimum pin base is 64

  if (pinBase < 64)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

// Check all pins in-case there is overlap:

  for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
    if (wiringPiFindNode (pin) != NULL)
      (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

  node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
  if (node == NULL)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

  node->pinBase         = pinBase ;
  node->pinMax          = pinBase + numPins - 1 ;
  node->pinMode         = pinModeDummy ;
  node->pullUpDnControl = pullUpDnControlDummy ;
  node->digitalRead     = digitalReadDummy ;
  node->digitalWrite    = digitalWriteDummy ;
  node->pwmWrite        = pwmWriteDummy ;
  node->analogRead      = analogReadDummy ;
  node->analogWrite     = analogWriteDummy ;
  node->next            = wiringPiNodes ;
  wiringPiNodes         = node ;

  return node ;
}


#ifdef notYetReady
/*
 * pinED01:
 * pinED10:
 *	Enables edge-detect mode on a pin - from a 0 to a 1 or 1 to 0
 *	Pin must already be in input mode with appropriate pull up/downs set.
 *********************************************************************************
 */

void pinEnableED01Pi (int pin)
{
  pin = pinToGpio [pin & 63] ;
}
#endif


/*
 *********************************************************************************
 * Core Functions
 *********************************************************************************
 */

/*
 * pinModeAlt:
 *	This is an un-documented special to let you set any pin to any mode
 *********************************************************************************
 */

void pinModeAlt (int pin, int mode)
{
  int fSel, shift ;

  if(asusversion == ASUSVER)
  {
    printf("Can not set ALT directly\n");
    return;
  }
  if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    fSel  = gpioToGPFSEL [pin] ;
    shift = gpioToShift  [pin] ;

    *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | ((mode & 0x7) << shift) ;
  }
}


/*
 * pinMode:
 *	Sets the mode of a pin to be input, output or PWM output
 *********************************************************************************
 */

void pinMode (int pin, int mode)
{
  int    fSel, shift, alt ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  int origPin = pin ;
  if(asusversion == ASUSVER)
  {
    if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
    {
      /**/ if (wiringPiMode == WPI_MODE_PINS)
        pin = pinToGpio [pin] ;
      else if (wiringPiMode == WPI_MODE_PHYS)
        pin = physToGpio [pin] ;
      else if (wiringPiMode != WPI_MODE_GPIO)
        return ;

      softPwmStop  (origPin) ;
      softToneStop (origPin) ;

      fSel    = gpioToGPFSEL [pin] ;
      shift   = gpioToShift  [pin] ;

      /**/ if (mode == INPUT)
        asus_set_pin_mode(pin,INPUT); // Sets bits to zero = input
      else if (mode == OUTPUT)
        asus_set_pin_mode(pin, OUTPUT);
      else if (mode == SOFT_PWM_OUTPUT)
        softPwmCreate (origPin, 0, 100) ;
      else if (mode == PWM_OUTPUT)
      {
	
        if((pin==PWM2)||(pin==PWM3))
        {
          asus_set_pin_mode(pin,PWM_OUTPUT);
          pwmSetRange (1024) ;            // Default range of 1024
          pwmSetClock (124) ;              // 74.25Mhz / 124 = 599KHz
          return;
        }

        printf("the pin you choose is not surport hardware PWM\n");
        printf("or you can use it in softPwm mode\n");  
        return ;
       }
       else if (mode == GPIO_CLOCK)
       {
         asus_set_pin_mode(pin,GPIO_CLOCK);

        // Set pin to GPIO_CLOCK mode and set the clock frequency to 1000KHz
        delayMicroseconds (110) ;
        gpioClockSet      (origPin, 1000000) ;
      }
    }//if ((pin & PI_GPIO_MASK) == 0)
    else
    {
      if ((node = wiringPiFindNode (pin)) != NULL)
        node->pinMode (node, pin, mode) ;
      return ;
    }
  }//if(asusversion == ASUSVER)
  else
  {
    if ((pin & PI_GPIO_MASK) == 0)		// On-board pin
    {
      /**/ if (wiringPiMode == WPI_MODE_PINS)
        pin = pinToGpio [pin] ;
      else if (wiringPiMode == WPI_MODE_PHYS)
        pin = physToGpio [pin] ;
      else if (wiringPiMode != WPI_MODE_GPIO)
        return ;
  
      softPwmStop  (origPin) ;
      softToneStop (origPin) ;
  
      fSel    = gpioToGPFSEL [pin] ;
      shift   = gpioToShift  [pin] ;
  
      if (mode == INPUT)
        *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) ; // Sets bits to zero = input
      else if (mode == OUTPUT)
        *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift) ;
      else if (mode == SOFT_PWM_OUTPUT)
        softPwmCreate (origPin, 0, 100) ;
      else if (mode == SOFT_TONE_OUTPUT)
        softToneCreate (origPin) ;
      else if (mode == PWM_TONE_OUTPUT)
      {
        pinMode (origPin, PWM_OUTPUT) ;	// Call myself to enable PWM mode
        pwmSetMode (PWM_MODE_MS) ;
      }
      else if (mode == PWM_OUTPUT)
      {
        if ((alt = gpioToPwmALT [pin]) == 0)	// Not a hardware capable PWM pin
  	    return ;
  
        // Set pin to PWM mode
        *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift) ;
        delayMicroseconds (110) ;		// See comments in pwmSetClockWPi
  
        pwmSetMode  (PWM_MODE_BAL) ;	// Pi default mode
        pwmSetRange (1024) ;		// Default range of 1024
        pwmSetClock (32) ;		// 19.2 / 32 = 600KHz - Also starts the PWM
      }
      else if (mode == GPIO_CLOCK)
      {
        if ((alt = gpioToGpClkALT0 [pin]) == 0)	// Not a GPIO_CLOCK pin
  	    return ;
  
        // Set pin to GPIO_CLOCK mode and set the clock frequency to 100KHz
        *(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (alt << shift) ;
        delayMicroseconds (110) ;
        gpioClockSet      (pin, 100000) ;
      }
    }
    else
    {
      if ((node = wiringPiFindNode (pin)) != NULL)
        node->pinMode (node, pin, mode) ;
      return ;
    }
  }
}


/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin
 *	The Arduino only has pull-ups and these are enabled by writing 1
 *	to a port when in input mode - this paradigm doesn't quite apply
 *	here though.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
//  printf("pin = %d\n",pin);
if(asusversion == ASUSVER)
{
	//printf("pin = %d\n",pin);
	if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  	{
    		/**/ if (wiringPiMode == WPI_MODE_PINS)
      		pin = pinToGpio [pin] ;
    		else if (wiringPiMode == WPI_MODE_PHYS)
      		pin = physToGpio [pin] ;
    		else if (wiringPiMode != WPI_MODE_GPIO)
      		return ;

    		asus_pullUpDnControl(pin, pud);
  	}
  	else						// Extension module
  	{
    		if ((node = wiringPiFindNode (pin)) != NULL)
      		node->pullUpDnControl (node, pin, pud) ;
    	return ;
  	}
	
}//if(version == ASUSVER)
else 
 {
  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

    *(gpio + GPPUD)              = pud & 3 ;		delayMicroseconds (5) ;
    *(gpio + gpioToPUDCLK [pin]) = 1 << (pin & 31) ;	delayMicroseconds (5) ;
    
    *(gpio + GPPUD)              = 0 ;			delayMicroseconds (5) ;
    *(gpio + gpioToPUDCLK [pin]) = 0 ;			delayMicroseconds (5) ;
  }
  else						// Extension module
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pullUpDnControl (node, pin, pud) ;
    return ;
  }
 }
}


/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */

int digitalRead (int pin)
{
  char c ;
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] == -1)
	return LOW ;

      lseek  (sysFds [pin], 0L, SEEK_SET) ;
      read   (sysFds [pin], &c, 1) ;
      return (c == '0') ? LOW : HIGH ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return LOW ;

    if (asus_digitalRead(pin)!= 0)
      return HIGH ;
    else
      return LOW ;
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead (node, pin) ;
  }
}


/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */

void digitalWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
//printf("mode  = %d,pin = %d\n",wiringPiMode,pin);
  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
    {
      if (sysFds [pin] != -1)
      {
	if (value == LOW)
	  write (sysFds [pin], "0\n", 2) ;
	else
	  write (sysFds [pin], "1\n", 2) ;
      }
      return ;
    }
    else if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

	asus_digitalWrite(pin,value);

    
  }
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite (node, pin, value) ;
  }
}


/*
 * pwmWrite:
 *	Set an output PWM value
 *********************************************************************************
 */

void pwmWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;
  if(asusversion == ASUSVER)
  {
    if ((pin & PI_GPIO_MASK) == 0)          // On-Board Pin
    {
      if (wiringPiMode == WPI_MODE_PINS)
        pin = pinToGpio [pin] ;
      else if (wiringPiMode == WPI_MODE_PHYS)
        pin = physToGpio [pin] ;
      else if (wiringPiMode != WPI_MODE_GPIO)
        return ;

      if (pin == PWM2)
        asus_pwm_start(2,0,pwm_divisor*pwm_range,pwm_divisor*value);
      else if (pin == PWM3)
        asus_pwm_start(3,0,pwm_divisor*pwm_range,pwm_divisor*value);

    }
    else
    {
      if ((node = wiringPiFindNode (pin)) != NULL)
        node->pwmWrite (node, pin, value) ;
    }
  }
  else
  {
	if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
	{
		if (RASPBERRY_PI_PERI_BASE == 0)	// Ignore for now
			return ;

		/**/ if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio [pin] ;
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio [pin] ;
		else if (wiringPiMode != WPI_MODE_GPIO)
			return ;

		*(pwm + gpioToPwmPort [pin]) = value ;
	}
	else
	{
		if ((node = wiringPiFindNode (pin)) != NULL)
		node->pwmWrite (node, pin, value) ;
	}
}
}

/*
 * analogRead:
 *	Read the analog value of a given Pin. 
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

int analogRead (int pin)
{
	struct wiringPiNodeStruct *node = wiringPiNodes ;

	if ((node = wiringPiFindNode (pin)) == NULL)
		return 0 ;
	else
		return node->analogRead (node, pin) ;
}


/*
 * analogWrite:
 *	Write the analog value to the given Pin. 
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

void analogWrite (int pin, int value)
{
	struct wiringPiNodeStruct *node = wiringPiNodes ;

	if ((node = wiringPiFindNode (pin)) == NULL)
		return ;

	node->analogWrite (node, pin, value) ;
}


/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */

void pwmToneWrite (int pin, int freq)
{
	int range ;
	if(asusversion == ASUSVER)
	{
		printf("If you want to use hardware pwm ,please read readme\n");
	
	}
  	else
  	{
		if (RASPBERRY_PI_PERI_BASE == 0)	// Ignore for now
		return ;

		if (freq == 0)
		pwmWrite (pin, 0) ;             // Off
		else
		{
			range = 600000 / freq ;
			pwmSetRange (range) ;
			pwmWrite (pin, freq / 2) ;
		}
	}	
  
}


/*
 * digitalWriteByte:
 *	Pi Specific
 *	Write an 8-bit byte to the first 8 GPIO pins - try to do it as
 *	fast as possible.
 *	However it still needs 2 operations to set the bits, so any external
 *	hardware must not rely on seeing a change as there will be a change 
 *	to set the outputs bits to zero, then another change to set the 1's
 *********************************************************************************
 */

void digitalWriteByte (int value)
{
	uint32_t pinSet = 0 ;
	uint32_t pinClr = 0 ;
	int mask = 1 ;
	int pin ;

	/**/ if (wiringPiMode == WPI_MODE_GPIO_SYS||wiringPiMode == WPI_MODE_GPIO)
	{
		for (pin = 0 ; pin < 8 ; ++pin)
		{
			pinMode(pin,OUTPUT);
			digitalWrite (pin, value & mask) ;
			mask <<= 1 ;
		}
		return ;
	}
	else if(wiringPiMode == WPI_MODE_PINS)
	{
		for (pin = 0 ; pin < 8 ; ++pin)
		{
			
			pinMode(pinToGpio [pin],OUTPUT);
			digitalWrite (pin, value & mask) ;
			mask <<= 1 ;
		}

	}
	else if(wiringPiMode == WPI_MODE_PHYS)
	{
		for (pin = 0 ; pin < 8 ; ++pin)
		{
			
			pinMode( physToGpio[pin],OUTPUT);
			digitalWrite (pin, value & mask) ;
			mask <<= 1 ;
		}
	}
}


/*
 * waitForInterrupt:
 *	Pi Specific.
 *	Wait for Interrupt on a GPIO pin.
 *	This is actually done via the /sys/class/gpio interface regardless of
 *	the wiringPi access mode in-use. Maybe sometime it might get a better
 *	way for a bit more efficiency.
 *********************************************************************************
 */

int waitForInterrupt (int pin, int mS)
{
	int fd, x ;
	uint8_t c ;
	struct pollfd polls ;

	/**/ if (wiringPiMode == WPI_MODE_PINS)
		pin = pinToGpio [pin] ;
	else if (wiringPiMode == WPI_MODE_PHYS)
		pin = physToGpio [pin] ;

	if ((fd = sysFds [pin]) == -1)
		return -2 ;

	// Setup poll structure

	polls.fd     = fd ;
	polls.events = POLLPRI ;	// Urgent data!

	// Wait for it ...

	x = poll (&polls, 1, mS) ;

	// Do a dummy read to clear the interrupt
	//	A one character read appars to be enough.
	//	Followed by a seek to reset it.

	(void)read (fd, &c, 1) ;
	lseek (fd, 0, SEEK_SET) ;

	return x ;
}


/*
 * interruptHandler:
 *	This is a thread and gets started to wait for the interrupt we're
 *	hoping to catch. It will call the user-function when the interrupt
 *	fires.
 *********************************************************************************
 */

static void *interruptHandler (void *arg)
{
	int myPin ;

	(void)piHiPri (55) ;	// Only effective if we run as root

	myPin   = pinPass ;
	pinPass = -1 ;

	for (;;)
		if (waitForInterrupt (myPin, -1) > 0)
			isrFunctions [myPin] () ;

	return NULL ;
}


/*
 * wiringPiISR:
 *	Pi Specific.
 *	Take the details and create an interrupt handler that will do a call-
 *	back to the user supplied function.
 *********************************************************************************
 */

int wiringPiISR (int pin, int mode, void (*function)(void))
{
	pthread_t threadId ;
	const char *modeS ;
	char fName   [64] ;
	char  pinS [8] ;
	pid_t pid ;
	int   count, i ;
	char  c ;
	int   bcmGpioPin ;
	int boardRev ;

	boardRev = piBoardRev () ;
	if (boardRev == ASUSVER) {
		if (!isAsusPiGpioPin40(pin))
			return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin must be GPIO40Pin (%d)\n", pin) ;
	} else {
		if ((pin < 0) || (pin > 63))
			return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin must be 0-63 (%d)\n", pin) ;
	}

	/**/ if (wiringPiMode == WPI_MODE_UNINITIALISED)
		return wiringPiFailure (WPI_FATAL, "wiringPiISR: wiringPi has not been initialised. Unable to continue.\n") ;
	else if (wiringPiMode == WPI_MODE_PINS)
		bcmGpioPin = pinToGpio [pin] ;
	else if (wiringPiMode == WPI_MODE_PHYS)
		bcmGpioPin = physToGpio [pin] ;
	else
		bcmGpioPin = pin ;

	// Now export the pin and set the right edge
	//	We're going to use the gpio program to do this, so it assumes
	//	a full installation of wiringPi. It's a bit 'clunky', but it
	//	is a way that will work when we're running in "Sys" mode, as
	//	a non-root user. (without sudo)

	if (mode != INT_EDGE_SETUP)
	{
		/**/ if (mode == INT_EDGE_FALLING)
			modeS = "falling" ;
		else if (mode == INT_EDGE_RISING)
			modeS = "rising" ;
		else
			modeS = "both" ;

		sprintf (pinS, "%d", bcmGpioPin) ;

		if ((pid = fork ()) < 0)	// Fail
			return wiringPiFailure (WPI_FATAL, "wiringPiISR: fork failed: %s\n", strerror (errno)) ;

		if (pid == 0)	// Child, exec
		{
			/**/ if (access ("/usr/local/bin/gpio", X_OK) == 0)
			{
				execl ("/usr/local/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
					return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
			}
			else if (access ("/usr/bin/gpio", X_OK) == 0)
			{
				execl ("/usr/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
					return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
			}
			else
				return wiringPiFailure (WPI_FATAL, "wiringPiISR: Can't find gpio program\n") ;
		}
		else		// Parent, wait
			wait (NULL) ;
	}

	// Now pre-open the /sys/class node - but it may already be open if
	//	we are in Sys mode...

	if (sysFds [bcmGpioPin] == -1)
	{
		sprintf (fName, "/sys/class/gpio/gpio%d/value", bcmGpioPin) ;
		if ((sysFds [bcmGpioPin] = open (fName, O_RDWR)) < 0)
			return wiringPiFailure (WPI_FATAL, "wiringPiISR: unable to open %s: %s\n", fName, strerror (errno)) ;
	}

	// Clear any initial pending interrupt

	ioctl (sysFds [bcmGpioPin], FIONREAD, &count) ;
	for (i = 0 ; i < count ; ++i)
		read (sysFds [bcmGpioPin], &c, 1) ;

	isrFunctions [pin] = function ;

	pthread_mutex_lock (&pinMutex) ;
	pinPass = pin ;
	pthread_create (&threadId, NULL, interruptHandler, NULL) ;
	while (pinPass != -1)
		delay (1) ;
	pthread_mutex_unlock (&pinMutex) ;

	return 0 ;
}


/*
 * initialiseEpoch:
 *	Initialise our start-of-time variable to be the current unix
 *	time in milliseconds and microseconds.
 *********************************************************************************
 */

static void initialiseEpoch (void)
{
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
  epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
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


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int howLong)
{
  struct timeval tNow, tLong, tEnd ;

  gettimeofday (&tNow, NULL) ;
  tLong.tv_sec  = howLong / 1000000 ;
  tLong.tv_usec = howLong % 1000000 ;
  timeradd (&tNow, &tLong, &tEnd) ;

  while (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *********************************************************************************
 */

unsigned int millis (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

  return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *********************************************************************************
 */

unsigned int micros (void)
{
  struct timeval tv ;
  uint64_t now ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;

  return (uint32_t)(now - epochMicro) ;
}


/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 *
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 *********************************************************************************
 */

int wiringPiSetup (void)
{
  int   fd ;
  int   boardRev ;
  int   model, rev, mem, maker, overVolted ;
   int i;

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (getenv (ENV_GPIOMEM) != NULL)
    wiringPiTryGpioMem = TRUE ;

  if (wiringPiDebug)
  {
    printf ("wiringPi: wiringPiSetup called\n") ;
    if (wiringPiTryGpioMem)
      printf ("wiringPi: Using /dev/gpiomem\n") ;
  }

  boardRev = piBoardRev () ;

  /**/ if (boardRev == ASUSVER)	// A, B, Rev 2, B+, CM, Pi2
  {
  	pinToGpio =  pinToGpio_AP ;
    physToGpio = physToGpio_AP ;
     
  }
  else 				// A, B, Rev 1, 1.1 
  {
     
	pinToGpio =  pinToGpioR1 ;
    physToGpio = physToGpioR1 ;
  }

  

// Open the master /dev/ memory control device
// Open the master /dev/memory device
   if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
   return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  if(boardRev == ASUSVER)
  {
		

	//printf("setup\n");
    	for(i=0;i<9;i++)
    	{
		if ((mem_fd0 = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) 
		{
         	  	printf("can't open /dev/mem \n");
       	       	  	return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
      		}

        // mmap GPIO 
        	gpio_map0[i] = mmap(
                	NULL,             // Any adddress in our space will do 
                	PAGE_SIZE,       // Map length 
               	 	PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
                	MAP_SHARED,       // Shared with other processes 
                	mem_fd0,           // File to map 
                	RK3288_GPIO(i)         //Offset to GPIO peripheral 
               		);
		if ((uint32_t)gpio_map0[i] < 0)
        	return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
        	close(mem_fd0); // No need to keep mem_fd open after mmap 
		gpio0[i] = (volatile unsigned *)gpio_map0[i];
		if (wiringPiDebug)
		printf("GPIO[%d]_SWPORTA_DDR = 0x%x\n",i,*(gpio0[i]+1));
   	}//for
  
/////////////mmap grf////////////
  	if((mem_fd4 = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
                printf("can't open /dev/mem \n");
                return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    	}

        // mmap GPIO 
    	grf_map = mmap(
                NULL,             // Any adddress in our space will do 
                PAGE_SIZE,       // Map length 
                PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
                MAP_SHARED,       // Shared with other processes 
                mem_fd4,           // File to map 
                RK3288_GRF_PHYS         //Offset to GPIO peripheral 
                );
	if (wiringPiDebug)
    	printf ("jason pwm_map = %x\n",grf_map) ;
	if ((uint32_t)grf_map < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    	close(mem_fd4); // No need to keep mem_fd open after mmap
    	grf = (volatile unsigned *)grf_map;
////////////////////////////    	
////////////mmap pwm////////
	if((mem_fdp = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
                printf("can't open /dev/mem \n");
                return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    	}
	pwm_map = mmap(
                NULL,             // Any adddress in our space will do 
                PAGE_SIZE,       // Map length 
                PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
                MAP_SHARED,       // Shared with other processes 
                mem_fdp,           // File to map 
                RK3288_PWM         //Offset to GPIO peripheral 
                );
	if (wiringPiDebug)
    	printf ("jason pwm_map = %x\n",pwm_map) ;
	if ((uint32_t)pwm_map < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    	close(mem_fdp); // No need to keep mem_fd open after mmap
    	pwm = (volatile unsigned *)pwm_map;
	if (wiringPiDebug)
	printf("RK3288_PWM0_CTR = 0x%x\n",*(pwm));
////////////////////////////
////////////mmap pmu//////////
	
	if((mem_fdpmu = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
                printf("can't open /dev/mem \n");
                return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    	}
	pmu_map = mmap(
                NULL,             // Any adddress in our space will do 
                PAGE_SIZE,       // Map length 
                PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
                MAP_SHARED,       // Shared with other processes 
                mem_fdpmu,           // File to map 
                RK3288_PMU         //Offset to GPIO peripheral 
                );
	if (wiringPiDebug)
    	printf ("jason pmu_map = %x\n",pmu_map) ;
	if ((uint32_t)pmu_map < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    	close(mem_fdpmu); // No need to keep mem_fdpmu open after mmap
    	pmu = (volatile unsigned *)pmu_map;
	if (wiringPiDebug)
	printf("RK3288_PMU = 0x%x\n",*(pmu));

///////////////////////////////
////////////mmap cru//////////

        if((mem_fdcru = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
                printf("can't open /dev/mem \n");
                return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
        }
        cru_map = mmap(
                NULL,             // Any adddress in our space will do
                PAGE_SIZE,       // Map length
                PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
                MAP_SHARED,       // Shared with other processes
                mem_fdcru,           // File to map
                RK3288_CRU         //Offset to GPIO peripheral
                );
        if (wiringPiDebug)
            printf ("jason cru_map = %x\n",cru_map) ;
        if ((uint32_t)cru_map < 0)
            return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
        close(mem_fdcru); // No need to keep mem_fdcru open after mmap
        cru = (volatile unsigned *)cru_map;
        if (wiringPiDebug)
        printf("RK3288_CRU = 0x%x\n",*(cru));

///////////////////////////////



  }//if(boardRev == ASUSVER)





// Map the individual hardware components

//	GPIO:
	else
	{
 	 gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE) ;
 	 if ((int32_t)gpio == -1)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

//	PWM

  pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PWM) ;
  if ((int32_t)pwm == -1)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror (errno)) ;
 
//	Clock control (needed for PWM)

  clk = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_CLOCK_BASE) ;
  if ((int32_t)clk == -1)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (CLOCK) failed: %s\n", strerror (errno)) ;
 
//	The drive pads

  pads = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_PADS) ;
  if ((int32_t)pads == -1)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (PADS) failed: %s\n", strerror (errno)) ;

#ifdef	USE_TIMER
//	The system timer

  timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_TIMER) ;
  if ((int32_t)timer == -1)
    return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (TIMER) failed: %s\n", strerror (errno)) ;

// Set the timer to free-running, 1MHz.
//	0xF9 is 249, the timer divide is base clock / (divide+1)
//	so base clock is 250MHz / 250 = 1MHz.

  *(timer + TIMER_CONTROL) = 0x0000280 ;
  *(timer + TIMER_PRE_DIV) = 0x00000F9 ;
  timerIrqRaw = timer + TIMER_IRQ_RAW ;
#endif
		}
  initialiseEpoch () ;

// If we're running on a compute module, then wiringPi pin numbers don't really many anything...

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;
  if (model == PI_MODEL_CM)
    wiringPiMode = WPI_MODE_GPIO ;
  else
    wiringPiMode = WPI_MODE_PINS ;

  return 0 ;
}


/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupGpio (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupGpio called\n") ;

  wiringPiMode = WPI_MODE_GPIO ;
  return 0 ;
}


/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupPhys (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupPhys called\n") ;

  wiringPiMode = WPI_MODE_PHYS ;

  return 0 ;
}


/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */

int wiringPiSetupSys (void)
{
  int boardRev ;
  int pin ;
  char fName [128] ;

  if (getenv (ENV_DEBUG) != NULL)
    wiringPiDebug = TRUE ;

  if (getenv (ENV_CODES) != NULL)
    wiringPiReturnCodes = TRUE ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupSys called\n") ;

  boardRev = piBoardRev () ;

  if (boardRev == ASUSVER)
  {
     pinToGpio =  pinToGpio_AP ;
    physToGpio = physToGpio_AP ;
  }
  else
  {
     pinToGpio =  pinToGpioR1 ;
    physToGpio = physToGpioR1 ;
  }

// Open and scan the directory, looking for exported GPIOs, and pre-open
//	the 'value' interface to speed things up for later
  
  for (pin = 0 ; pin < 64 ; ++pin)
  {
    if (pinToGpio[pin] != -1) {
      sprintf (fName, "/sys/class/gpio/gpio%d/value", pinToGpio[pin]) ;
      sysFds [pinToGpio[pin]] = open (fName, O_RDWR) ;
    }
  }

  initialiseEpoch () ;

  wiringPiMode = WPI_MODE_GPIO_SYS ;

  return 0 ;
}
