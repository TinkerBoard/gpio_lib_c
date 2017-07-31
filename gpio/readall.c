/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2015 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

extern int wpMode ;

#ifndef TRUE
#  define       TRUE    (1==1)
#  define       FALSE   (1==2)
#endif

#define SERIAL       40
#define PWM          43
#define GPIOIN           53
#define GPIOOUT          54
#define CLKOUT           55


static const char *asusPinModeToString (int mode)
{
  if (mode == SERIAL)
    return "SERL";
  else if ( mode == PWM)
    return "PWM";
  else if ( mode == GPIOIN)
    return "IN";
  else if ( mode == GPIOOUT)
    return "OUT";
  else if ( mode == CLKOUT)
    return "CLK";
  else
    return " ";
}

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal (void)
{
  int pin ;

  printf ("+------+---------+--------+\n") ;
  printf ("|  Pin | Digital | Analog |\n") ;
  printf ("+------+---------+--------+\n") ;

  for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
    printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

  printf ("+------+---------+--------+\n") ;
}


/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 *********************************************************************************
 */

static char *alts [] =
{
  "IN", "OUT", "ALT5", "ALT4", "ALT0", "ALT1", "ALT2", "ALT3"
} ;

static int physToWpi [64] = 
{
  -1,           // 0
  -1, -1,       // 1, 2
   8, -1,
   9, -1,
   7, 15,
  -1, 16,
   0,  1,
   2, -1,
   3,  4,
  -1,  5,
  12, -1,
  13,  6,
  14, 10,
  -1, 11,       // 25, 26
  30, 31,	// Actually I2C, but not used
  21, -1,
  22, 26,
  23, -1,
  24, 27,
  25, 28,
  -1, 29,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  -1, -1,
  17, 18,
  19, 20,
  -1, -1, -1, -1, -1, -1, -1, -1, -1
} ;

static char *physNames [64] = 
{
  NULL,

  "   3.3v", "5v     ",//1,2
  "  SDA.1", "5V     ",//3,4
  "  SCL.1", "0v     ",//5,6
  "GPIO0C1", "TxD1   ",//7,8
  "     0v", "RxD1   ",//9,10
  "GPIO5B4", "GPIO6A0",//11,12
  "GPIO5B6", "0v     ",//13,14
  "GPIO5B7", "GPIO5B2",//15,16
  "   3.3v", "GPIO5B3",//17,18
  "  MOSI1", "0v     ",//19,20
  "  MISO1", "GPIO5C3",//21,22
  "  SCLK1", "CE0    ",//23,24
  "     0v", "CE1    ",//25,26
  "  SDA.2", "SCL.2  ",//27,28
  "GPIO5B5", "0v     ",//29.30
  "GPIO5C0", "GPIO7C7",//31,32
  "GPIO7C6", "0v     ",//33,34
  "GPIO6A1", "GPIO7A7",//35,36
  "GPIO7B0", "GPIO6A3",//37,38
  "     0v", "GPIO6A4",//39,40
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
  "GPIO.17", "GPIO.18",
  "GPIO.19", "GPIO.20",
   NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
} ;


/*
 * readallPhys:
 *	Given a physical pin output the data on it and the next pin:
 *| BCM | wPi |   Name  | Mode | Val| Physical |Val | Mode | Name    | wPi | BCM |
 *********************************************************************************
 */

static void readallPhys (int physPin, int model)
{
  int pin ;
	
  if (physPinToGpio (physPin) == -1)
    printf (" |     |    ") ;
  else
    printf (" | %3d | %3d", physPinToGpio (physPin), physToWpi [physPin]) ;

  printf (" | %s", physNames [physPin]) ;

  if (physToWpi [physPin] == -1)
    printf (" |      |  ") ;
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

   if (model == PI_MODEL_ASUSPI)
     printf (" | %4s", asusPinModeToString(getPinMode (pin))) ;
   else
     printf (" | %4s", alts [getAlt (pin)]) ;

   printf (" | %d", digitalRead (pin)) ;
  }

// Pin numbers:

  printf (" | %2d", physPin) ;
  ++physPin ;
  printf (" || %-2d", physPin) ;

// Same, reversed

  if (physToWpi [physPin] == -1)
    printf (" |   |     ") ;
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

   printf (" | %d", digitalRead (pin)) ; 
   //printf (" |     ") ;
   if (model == PI_MODEL_ASUSPI)
     printf (" | %-4s", asusPinModeToString(getPinMode (pin))) ;
   else
     printf (" | %-4s", alts [getAlt (pin)]) ;
  }

  printf (" | %-5s", physNames [physPin]) ;

  if (physToWpi[physPin] == -1)
    printf (" |     |    ") ;
  else
  	{
    	printf (" | %-3d | %-3d", physToWpi [physPin], physPinToGpio (physPin)) ;
  	}
  printf (" |\n") ;
}


void cmReadall (void)
{
  int pin ;

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;
  printf ("| Pin | Mode | Value |      | Pin | Mode | Value |\n") ;
  printf ("+-----+------+-------+      +-----+------+-------+\n") ;

  for (pin = 0 ; pin < 28 ; ++pin)
  {
    printf ("| %3d ", pin) ;
   // printf ("| %-4s ", alts [getAlt (pin)]) ;
    printf ("| %s  ", digitalRead (pin) == HIGH ? "High" : "Low ") ;
    printf ("|      ") ;
    printf ("| %3d ", pin + 28) ;
  //  printf ("| %-4s ", alts [getAlt (pin + 28)]) ;
    printf ("| %s  ", digitalRead (pin + 28) == HIGH ? "High" : "Low ") ;
    printf ("|\n") ;
  }

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;
}


/*
 * abReadall:
 *	Read all the pins on the model A or B.
 *********************************************************************************
 */

void abReadall (int model, int rev)
{
  int pin ;
  char *type ;

  if (model == PI_MODEL_A)
    type = " A" ;
  else
    if (rev == PI_VERSION_2)
      type = "B2" ;
    else
      type = "B1" ;

  printf (" +-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type) ;
  printf (" | CPU | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | CPU |\n") ;
  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 26 ; pin += 2)
    readallPhys (pin, model) ;

  if (rev == PI_VERSION_2) // B version 2
  {
    printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
    for (pin = 51 ; pin <= 54 ; pin += 2)
      readallPhys (pin, model) ;
  }

  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf (" | CPU | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | CPU|\n") ;
  printf (" +-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type) ;
}


/*
 * piPlusReadall:
 *	Read all the pins on the model A+ or the B+
 *********************************************************************************
 */

static void plus2header (int model)
{
  /**/ if (model == PI_MODEL_AP)
    printf (" +-----+-----+---------+------+---+--A Plus--+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_BP)
    printf (" +-----+-----+---------+------+---+--B Plus--+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_ZERO)
    printf (" +-----+-----+---------+------+---+-Pi Zero--+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_ASUSPI)
    printf (" +-----+-----+---------+------+---+--Tinker--+---+------+---------+-----+-----+\n") ;
  else
    printf (" +-----+-----+---------+------+---+---Pi ----+---+------+---------+-----+-----+\n") ;
}


void piPlusReadall (int model)
{
  int pin ;

  plus2header (model) ;

  printf (" | CPU | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | CPU |\n") ;
  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 40 ; pin += 2)
    readallPhys (pin, model) ;
  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf (" | CPU | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | CPU |\n") ;

  plus2header (model) ;
}


void doReadall (void)
{
  int model, rev, mem, maker, overVolted ;

  if (wiringPiNodes != NULL)	// External readall
  {
    doReadallExternal () ;
    return ;
  }

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

  /**/ if ((model == PI_MODEL_A) || (model == PI_MODEL_B))
    abReadall (model, rev) ;
  else if ((model == PI_MODEL_BP) || (model == PI_MODEL_AP) || (model == PI_MODEL_2) || (model == PI_MODEL_ZERO)||(PI_MODEL_ASUSPI))
    piPlusReadall (model) ;
  else if (model == PI_MODEL_CM)
    cmReadall () ;
  else
    printf ("Oops - unable to determine board type... model: %d\n", model) ;
}
