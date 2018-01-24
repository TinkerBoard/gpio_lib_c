#ifndef	__WIRING_TB_H__
#define	__WIRING_TB_H__

#include "RKIO.h"

int* asus_get_physToGpio(int rev);
int* asus_get_pinToGpio(int rev);
int tinker_board_setup(int rev);
int asus_get_pin_mode(int pin);
void asus_set_pinmode_as_gpio(int pin);
void asus_set_pin_mode(int pin, int mode);
void asus_digitalWrite(int pin, int value);
int asus_digitalRead(int pin);
void asus_pullUpDnControl (int pin, int pud);
void asus_set_pwmPeriod(int pin, unsigned int period);
void asus_set_pwmRange(unsigned int range);
void asus_set_pwmFrequency(int pin, int divisor);
void asus_set_pwmClock(int divisor);
void asus_pwm_write(int pin, int value);
void asus_pwmToneWrite(int pin, int freq);
void asus_set_gpioClockFreq(int pin, int freq);
int asus_get_pinAlt(int pin);
void asus_set_pinAlt(int pin, int alt);
void asus_set_GpioDriveStrength(int pin, int drv_type);
int asus_get_GpioDriveStrength(int pin);
void asus_cleanup(void);
#endif
