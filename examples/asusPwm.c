/*
*	use asus pi hardware pwm
*	func:		void asus_pwm_start(int pwm_ch,int mode,int range,int duty);
*	parameter¡G		pwm_ch:pwm chanel 2,3
*				mode:center 1
*					 left	0
*				range:set pwm period (0~0xffffffff)
*				duty:set pwm duty (0~0xffffffff)
*	note:if you want tu use  asus_pwm_start(),please use pinMode() to set the pin's mode to PWM_OUTPUT first
*
*	note: Since the clock of pwm is 74250000hz. You should not set pwm period more 
*	than 74250000 or your pwm frequency will lower than 1hz.
*
*	func:	void asus_pwm_stop(int pwm_ch)
*	parameter¡G		pwm_ch:pwm chanel 2,3
*
*/

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main (void)
{
	wiringPiSetupGpio();
	pinMode (239, PWM_OUTPUT) ;

	//range=1s, period=0.5s, freq=1hz, val=0.5
	asus_pwm_start(3,0,74250000,37125000);
	while(1)
	{
	
	}

}
