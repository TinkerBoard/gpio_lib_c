#ifndef	__WIRING_TB_H__
#define	__WIRING_TB_H__

#include "RKIO.h"
#include "wiringPi.h"

static int pwm_divisor = 124;
static int pwm_range = 1024;

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

void tinker_board_setup(void);
int asus_get_pin_mode(int pin);
void asus_set_pinmode_as_gpio(int pin);
int asus_set_pin_mode(int pin, int mode);
void asus_digitalWrite(int pin, int value);
int asus_digitalRead(int pin);
void asus_pullUpDnControl (int pin, int pud);
void asus_pwm_set_period(int pwm_ch,unsigned int range);
void asus_pwm_set_duty(int pwm_ch,unsigned int duty);
void asus_pwm_enable(int pwm_ch);
void asus_pwm_disable(int pwm_ch);
void asus_pwm_start(int pwm_ch,int mode,unsigned int range,unsigned int duty);
void asus_pwm_stop(int pwm_ch);
void asus_set_gpioClockFreq(int pin, int freq);
int asus_get_pinAlt(int pin);
#endif
