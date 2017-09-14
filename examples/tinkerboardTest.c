/*
*	Testing wiringPi's functions in TinkerBoard.
*/

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <wiringPi.h>
#define SHOWINFO 1
#define OSCILLOSCOPTEST 1
#define PAUSE if(OSCILLOSCOPTEST && OSCILLOSCOPTEST) { \
	printf("Press Enter key to continue..."); \
	fgetc(stdin); \
}
void showAPIStatus(char* APIName, int Succ)
{
    if( SHOWINFO || !Succ)
    {
	char* Status = (Succ) ? "Successful" : "Failed";
        printf("[%-10s] API: %s\n", Status, APIName);
    }
}

int main()
{
    printf("\n\n\n====  TESTING ====\n");
    //Get Pin
    assert(wiringPiSetup() == 0);
    assert(wpiPinToGpio(7) == 17);
    assert(wpiPinToGpio(26) == 239);
    assert(wpiPinToGpio(12) == 257);
    showAPIStatus("wpiPinToGpio", 1);
    assert(physPinToGpio(7) == 17);
    assert(physPinToGpio(32) == 239);
    assert(physPinToGpio(19) == 257);
    showAPIStatus("physPinToGpio", 1);
    //wiringPiSetup Testing
    pinMode(7, INPUT);
    pinMode(26, INPUT);
    pinMode(12, INPUT);
    assert(getAlt(7) == 0);
    assert(getAlt(26) == 0);
    assert(getAlt(26) == 0);
    pinMode(7, GPIO_CLOCK);
    assert(getAlt(7) == 0b100); //Alt0
    pinMode(26, PWM_OUTPUT);
    assert(getAlt(26) == 0b110); //Alt2
    pinModeAlt(7, 0b001); //OUTP
    assert(getAlt(7) == 1);
    digitalWrite(7, 1);
    assert(digitalRead(7) == 1);
    digitalWrite(7, 0);
    assert(digitalRead(7) == 0);
    showAPIStatus("pinModeAlt", 1);
    pinMode(7, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(12, OUTPUT);
    assert(getAlt(7) == 1);
    assert(getAlt(26) == 1);
    assert(getAlt(12) == 1);
    showAPIStatus("getAlt", 1);
    digitalWrite(7, 1);
    assert(digitalRead(7) == 1);
    digitalWrite(7, 0);
    assert(digitalRead(7) == 0);
    digitalWrite(26, 1);
    assert(digitalRead(26) == 1);
    digitalWrite(26, 0);
    assert(digitalRead(26) == 0);
    digitalWrite(12, 1);
    assert(digitalRead(12) == 1);
    digitalWrite(12, 0);
    assert(digitalRead(12) == 0);
    showAPIStatus("pinMode", 1);
    showAPIStatus("digitalWrite", 1);
    showAPIStatus("digitalRead", 1);
    showAPIStatus("wiringPiSetup", 1);
    //ISCUKKISCOPE
    printf("\n\n\n==== USE OSCILLOSCOPE FOR TESTING ====\n");
    printf("==== Connect to Physical PIN 7(Clock Output) ====\n");
    PAUSE
    pinMode(7, OUTPUT);
    digitalWrite(7, 1);
    showAPIStatus("pinMode, digitalWrite [Check Pin 7 (HIGH) by Oscilloscop]", 1);
    PAUSE
    digitalWrite(7, 0);
    showAPIStatus("pinMode, digitalWrite [Check Pin 7 (LOW) by Oscilloscop]", 1);
    PAUSE
    pinMode(7, GPIO_CLOCK);
    gpioClockSet(7, 10 * 1000000);
    showAPIStatus("pinMode, gpioClockSet [Check Pin 7 (10MHz) by Oscilloscop]", 1);
    PAUSE
    gpioClockSet(7, 20 * 1000000);
    showAPIStatus("gpioClockSet [Check Pin 7 (20MHz) by Oscilloscop]", 1);
    PAUSE
    gpioClockSet(7, 50 * 1000000);
    showAPIStatus("gpioClockSet [Check Pin 7 (50MHz) by Oscilloscop]", 1);
    PAUSE
    printf("==== Connect to Physical PIN 33 (PWM2) ====\n");
    PAUSE
    pinMode(23, OUTPUT);
    digitalWrite(23, 1);
    showAPIStatus("pinMode, digitalWrite [Check Pin 33 (HIGH) by Oscilloscop]", 1);
    PAUSE
    digitalWrite(23, 0);
    showAPIStatus("pinMode, digitalWrite [Check Pin 33 (LOW) by Oscilloscop]", 1);
    PAUSE
    pinMode(23, PWM_OUTPUT);
    pwmSetRange(1024);
    pwmSetClock(124);
    pwmWrite(23, 512);
    showAPIStatus("pinMode [Check Pin 33 (Origial Pulse) by Oscilloscop]", 1);
    PAUSE
    pwmSetClock(248);
    showAPIStatus("pwmSetClock [Check Pin 33 (Changed Pulse) by Oscilloscop]", 1);
    PAUSE
    pwmSetRange(2048);
    showAPIStatus("pwmSetRange [Check Pin 33 (Changed Pulse) by Oscilloscop]", 1);
    PAUSE
    pwmWrite(23, 1024);
    showAPIStatus("pwmWrite [Check Pin 33 (Changed Pulse) by Oscilloscop]", 1);
    PAUSE
	pwmSetClock(124);
	pwmToneWrite(23, 5 * 1000);
    showAPIStatus("pwmToneWrite [Check Pin 33 (5kHz) by Oscilloscop]", 1);
    PAUSE
	pwmToneWrite(23, 20 * 1000);
    showAPIStatus("pwmToneWrite [Check Pin 33 (20kHz) by Oscilloscop]", 1);
    PAUSE
    printf("==== CHECK Physical PIN 32 (PWM3) ====\n");
    PAUSE
    pinMode(26, OUTPUT);
    digitalWrite(26, 1);
    showAPIStatus("pinMode, digitalWrite [Check Pin 32 (HIGH) by Oscilloscop]", 1);
    PAUSE
    digitalWrite(26, 0);
    showAPIStatus("pinMode, digitalWrite [Check Pin 32 (LOW) by Oscilloscop]", 1);
    PAUSE
    pinModeAlt(26, 0b110);
    //pinMode(26, PWM_OUTPUT);
    pwmSetRange(1024);
    pwmSetClock(124);
    pwmWrite(26, 512);
    showAPIStatus("pinMode [Check Pin 32 (Origial Pulse) by Oscilloscop]", 1);
    PAUSE
    pwmSetClock(248);
    showAPIStatus("pwmSetClock [Check Pin 32 (Changed Pulse) by Oscilloscop]", 1);
    PAUSE
    pwmSetRange(2048);
    showAPIStatus("pwmSetRange [Check Pin 32 (Changed Pulse) by Oscilloscop]", 1);
    PAUSE
    pwmWrite(26, 1024);
    showAPIStatus("pwmWrite [Check Pin 32 (Changed Pulse) by Oscilloscop]", 1);
    PAUSE
	pwmSetClock(124);
	pwmToneWrite(26, 5 * 1000);
    showAPIStatus("pwmToneWrite [Check Pin 32 (5kHz) by Oscilloscop]", 1);
    PAUSE
	pwmToneWrite(26, 20 * 1000);
    showAPIStatus("pwmToneWrite [Check Pin 32 (20kHz) by Oscilloscop]", 1);
    PAUSE
    return 0;
}
