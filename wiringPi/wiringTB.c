#include "wiringTB.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#define CONFIG_I2S_SHORT

#define	BLOCK_SIZE		(4*1024)

// Pin modes

#define	INPUT			 0
#define	OUTPUT			 1
#define	PWM_OUTPUT		 2
#define	GPIO_CLOCK		 3
#define	SOFT_PWM_OUTPUT		 4
#define	SOFT_TONE_OUTPUT	 5
#define	PWM_TONE_OUTPUT		 6

// Port function select bits

#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

//jason add for asuspi
static int  mem_fd;
static void* gpio_map0[9];
static volatile unsigned* gpio0[9];

static void *grf_map;
static volatile unsigned *grf;

static void *pwm_map;
static volatile unsigned *pwm;

static void *pmu_map;
static volatile unsigned *pmu;

static void *cru_map;
static volatile unsigned *cru;

int* asus_get_physToGpio(int rev)
{
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
	return physToGpio_AP;
}
int* asus_get_pinToGpio(int rev)
{
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
	return pinToGpio_AP;
}

int tinker_board_setup(int rev)
{
	int i;
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) 
	{
		if ((mem_fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
		{
			printf("can't open /dev/mem and /dev/gpiomem\n");
			printf("wiringPiSetup: Unable to open /dev/mem and /dev/gpiomem: %s\n", strerror (errno));
			return -1;
		}
    }
	for(i=0;i<9;i++)
    {
		// mmap GPIO 
		#ifdef ANDROID
		gpio_map0[i] = mmap64(
		#else
		gpio_map0[i] = mmap(
		#endif
			NULL,             // Any adddress in our space will do 
			BLOCK_SIZE,       // Map length 
			PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
			MAP_SHARED,       // Shared with other processes 
			mem_fd,           // File to map 
			RK3288_GPIO(i)         //Offset to GPIO peripheral 
		);
		if (gpio_map0[i] == MAP_FAILED)
		{
			printf("wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno));
			return -1;
		}
		gpio0[i] = (volatile unsigned *)gpio_map0[i];
   	}//for
	/////////////mmap grf////////////
	#ifdef ANDROID
	grf_map = mmap64(
	#else
	grf_map = mmap(
	#endif
		NULL,             // Any adddress in our space will do 
		BLOCK_SIZE,       // Map length 
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
		MAP_SHARED,       // Shared with other processes 
		mem_fd,           // File to map 
		RK3288_GRF_PHYS         //Offset to GPIO peripheral 
	);
	if (grf_map  == MAP_FAILED)
	{
		printf("wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno));
		return -1;
	}
    grf = (volatile unsigned *)grf_map;
	////////////////////////////  
	////////////mmap pwm////////
	#ifdef ANDROID
	pwm_map = mmap64(
	#else
	pwm_map = mmap(
	#endif
		NULL,             // Any adddress in our space will do 
		BLOCK_SIZE,       // Map length 
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
		MAP_SHARED,       // Shared with other processes 
		mem_fd,           // File to map 
		RK3288_PWM         //Offset to GPIO peripheral 
	);
	if (pwm_map == MAP_FAILED)
	{
		printf("wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno));
		return -1;
	}
    pwm = (volatile unsigned *)pwm_map;
	////////////////////////////
	////////////mmap pmu//////////
	#ifdef ANDROID
	pmu_map = mmap64(
	#else
	pmu_map = mmap(
	#endif
		NULL,             // Any adddress in our space will do 
		BLOCK_SIZE,       // Map length 
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
		MAP_SHARED,       // Shared with other processes 
		mem_fd,           // File to map 
		RK3288_PMU         //Offset to GPIO peripheral 
	);
	if (pmu_map == MAP_FAILED)
	{
		printf("wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno));
		return -1;
	}
    pmu = (volatile unsigned *)pmu_map;
	///////////////////////////////
	////////////mmap cru//////////
	#ifdef ANDROID
	cru_map = mmap64(
	#else
	cru_map = mmap(
	#endif
		NULL,             // Any adddress in our space will do
		BLOCK_SIZE,       // Map length
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
		MAP_SHARED,       // Shared with other processes
		mem_fd,           // File to map
		RK3288_CRU         //Offset to GPIO peripheral
	);
	if (cru_map == MAP_FAILED)
	{
		printf("wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno));
		return -1;
	}
	cru = (volatile unsigned *)cru_map;
	///////////////////////////////
	close(mem_fd); // No need to keep mem_fdcru open after mmap
	return 0;
}
int gpioToBank(int gpio)
{
	if(gpio < 24)
		return 0;
	else
		return (int)((gpio - 24) / 32) + 1;
}

int gpioToBankPin(int gpio)
{
	if(gpio < 24)
		return gpio;
	else
		return (gpio - 24) % 32;
}

int gpio_is_valid(int gpio)
{
	switch (gpio)
	{
		case 17:
		case 160:
		case 161:
		case 162:
		case 163:
		case 164:
		case 165:
		case 166:
		case 167:
		case 168:
		case 171:
		case 184:
		case 185:
		case 187:
		case 188:
		case 223:
		case 224:
		case 233:
		case 234:
		case 238:
		case 239:
		case 251:
		case 252:
		case 253:
		case 254:
		case 255:
		case 256:
		case 257:
			return 1;
		default:
			return 0;
	}
}

int gpio_clk_disable(int gpio)
{
	int bank, bank_clk_en;
	int write_bit, reg_offset;
	bank = gpioToBank(gpio);
	write_bit = (bank != 0) ? bank : 4;
	reg_offset = (bank != 0) ? CRU_CLKGATE14_CON : CRU_CLKGATE17_CON;
	bank_clk_en = (*(cru+reg_offset/4) >> write_bit) & 0x1;
	*(cru+reg_offset/4) = (*(cru+reg_offset/4) & ~(0x1 << write_bit)) | (0x1 << (16 + write_bit));
	return bank_clk_en;
}
void gpio_clk_recovery(int gpio, int flag)
{
	int bank;
	int write_bit, reg_offset;
	bank = gpioToBank(gpio);
	write_bit = (bank != 0) ? bank : 4;
	reg_offset = (bank != 0) ? CRU_CLKGATE14_CON : CRU_CLKGATE17_CON;
	*(cru+reg_offset/4) = (*(cru+reg_offset/4) | (flag << write_bit)) | (0x1 << (16 + write_bit));
}
int asus_get_pin_mode(int pin)
{
	int value, func;
	int bank_clk_en;
	bank_clk_en = gpio_clk_disable(pin);
	switch(pin)
	{
		//GPIO0
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
			value = ((*(grf+GRF_GPIO7CL_IOMUX/4))>>((pin%8)*2)) & 0x00000001;
			switch(value)
			{
				case 0: func=GPIO;		break;
				case 1: func=I2C;		break;
				default: func=-1;		break;
			}
			break;
		case 238 : 
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
			if (*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) & 1<<((pin+8)%32))
				func = OUTPUT;
			else
				func = INPUT;
		}
		else
		{
			if (*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) & 1<<(pin%32))
				func = OUTPUT;
			else
				func = INPUT;
		}
	}
	gpio_clk_recovery(pin, bank_clk_en);
	return func;
}

void asus_set_pinmode_as_gpio(int pin)
{
	switch(pin)
	{
		//GPIO0
		case 17 : 
			*(pmu+PMU_GPIO0C_IOMUX/4) = (*(pmu+PMU_GPIO0C_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)));
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
		case 185 :
			#ifdef CONFIG_I2S_SHORT
			*(grf+GRF_GPIO6A_IOMUX/4) =  (*(grf+GRF_GPIO6A_IOMUX/4) | (0x0f<<((pin%8)*2+16))) & (~(0x0f<<((pin%8)*2)));
			*(grf+GRF_GPIO6A_P/4) = ((*(grf+GRF_GPIO6A_P/4) | (0x03<<(((186)%8)*2+16))) & (~(0x03<<(((186)%8)*2)))) | (0<<(((186)%8)*2+1)) | (0<<(((186)%8)*2));
			#else
			*(grf+GRF_GPIO6A_IOMUX/4) =  (*(grf+GRF_GPIO6A_IOMUX/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)));
			#endif
			break;
		case 184 : 
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
			break;
	}	//switch(pin)

}

void asus_set_pin_mode(int pin, int mode)
{
	int bank_clk_en;
	int bank, bank_pin;
	if(!gpio_is_valid(pin))
		return;
	bank = gpioToBank(pin);
	bank_pin = gpioToBankPin(pin);
	bank_clk_en = gpio_clk_disable(pin);
	if(INPUT == mode)
	{
		asus_set_pinmode_as_gpio(pin);
		*(gpio0[bank]+GPIO_SWPORTA_DDR_OFFSET/4) &= ~(1<<bank_pin);
	}
	else if(OUTPUT == mode)
	{
		asus_set_pinmode_as_gpio(pin);
		*(gpio0[bank]+GPIO_SWPORTA_DDR_OFFSET/4) |= (1<<bank_pin);
	} 
	else if(PWM_OUTPUT == mode)
	{
		//set pin PWMx to pwm mode
		if(pin == PWM2)
		{
			*(grf+GRF_GPIO7CH_IOMUX/4) =  (*(grf+GRF_GPIO7CH_IOMUX/4) | (0x0f<<(16+(pin%8-4)*4))) | (0x03<<((pin%8-4)*4));
		}
		else if(pin == PWM3)
		{
			*(grf+GRF_GPIO7CH_IOMUX/4) =  (*(grf+GRF_GPIO7CH_IOMUX/4) | (0x0f<<(16+(pin%8-4)*4))) | (0x03<<((pin%8-4)*4));
		}
		else
		{
			printf("This pin cannot set as pwm out\n");
		}
	}
	else if(GPIO_CLOCK == mode)
	{
		if(pin == 17)
		{
			*(pmu+PMU_GPIO0C_IOMUX/4) = (*(pmu+PMU_GPIO0C_IOMUX/4) & (~(0x03<<((pin%8)*2)))) | (0x01<<((pin%8)*2));
		}
		else
			printf("This pin cannot set as gpio clock\n");
	}
	gpio_clk_recovery(pin, bank_clk_en);
}

void asus_digitalWrite(int pin, int value)
{
	int bank_clk_en;
	int bank, bank_pin;
	if(!gpio_is_valid(pin))
		return;
	bank = gpioToBank(pin);
	bank_pin = gpioToBankPin(pin);
	bank_clk_en = gpio_clk_disable(pin);
	if(value > 0)
	{
		*(gpio0[bank]+GPIO_SWPORTA_DR_OFFSET/4) |= (1<<bank_pin);
	}
	else
	{
		*(gpio0[bank]+GPIO_SWPORTA_DR_OFFSET/4) &= ~(1<<bank_pin);
	}
	gpio_clk_recovery(pin, bank_clk_en);
}

int asus_digitalRead(int pin)
{
	int value;
	int bank_clk_en;
	int bank, bank_pin;
	bank = gpioToBank(pin);
	bank_pin = gpioToBankPin(pin);
	bank_clk_en = gpio_clk_disable(pin);
	value = (((*(gpio0[bank]+GPIO_EXT_PORTA_OFFSET/4)) & (1 << bank_pin)) >> bank_pin); 
	gpio_clk_recovery(pin, bank_clk_en);
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
			*(pmu+PMU_GPIO0C_P/4) = ((*(pmu+PMU_GPIO0C_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
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
			*(grf+GRF_GPIO5B_P/4) = ((*(grf+GRF_GPIO5B_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
			break;
		
		//GPIO5C
		case 168 : 
		case 169 :
		case 170 :
		case 171 :
			*(grf+GRF_GPIO5C_P/4) = ((*(grf+GRF_GPIO5C_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
			break;

		//GPIO6A
		case 184 : 
		case 185 :
		case 187 :
		case 188 :
			*(grf+GRF_GPIO6A_P/4) = ((*(grf+GRF_GPIO6A_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));
			break;

		//GPIO7A7
		case 223 : 
			*(grf+GRF_GPIO7A_P/4) = ((*(grf+GRF_GPIO7A_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;

		//GPIO7B
		case 224 : 
		case 225 : 
		case 226 : 
			*(grf+GRF_GPIO7B_P/4) = ((*(grf+GRF_GPIO7B_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;
		//GPIO7C
		case 233 : 
		case 234 : 			
		case 238 : 			
		case 239 : 
			*(grf+GRF_GPIO7C_P/4) = ((*(grf+GRF_GPIO7C_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2));  
			break;

		//GPIO8A
		case 251 : 
		case 254 :
		case 255 :			
		case 252 : 
		case 253 :
			*(grf+GRF_GPIO8A_P/4) = ((*(grf+GRF_GPIO8A_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;
		//GPIO8B
		case 256 : 
		case 257 :
			*(grf+GRF_GPIO8B_P/4) = ((*(grf+GRF_GPIO8B_P/4) | (0x03<<((pin%8)*2+16))) & (~(0x03<<((pin%8)*2)))) | (bit1<<((pin%8)*2+1)) | (bit0<<((pin%8)*2)); 
			break;
		default:
			printf("wrong gpio\n");
			break;
	}	//switch(pin)
}

int asus_get_pwm_value(int pin)
{
	unsigned int range;
	unsigned int value;
	int PWM_PERIOD_OFFSET = -1;
	int PWM_DUTY_OFFSET = -1;
	switch (pin)
	{
		case PWM2:
			PWM_PERIOD_OFFSET=RK3288_PWM2_PERIOD;
			PWM_DUTY_OFFSET=RK3288_PWM2_DUTY;
			break;
		case PWM3:
			PWM_PERIOD_OFFSET=RK3288_PWM3_PERIOD;
			PWM_DUTY_OFFSET=RK3288_PWM3_DUTY;
			break;
		default:
			break;
	}
	if(asus_get_pin_mode(pin)==PWM && PWM_PERIOD_OFFSET != -1 && PWM_DUTY_OFFSET != -1)
	{
		range = *(pwm+PWM_PERIOD_OFFSET/4);	//Get period
		value = range - *(pwm+PWM_DUTY_OFFSET/4); //Get duty
		return value;
	}
	else
	{
		return -1; 
	}
}

void asus_set_pwmRange(unsigned int range)
{
	int pwm2_value = asus_get_pwm_value(PWM2);
	int pwm3_value = asus_get_pwm_value(PWM3);
	*(pwm+RK3288_PWM2_CTR/4) &= ~(1<<0);	//Disable PWM2
	*(pwm+RK3288_PWM3_CTR/4) &= ~(1<<0);	//Disable PWM3		
	*(pwm+RK3288_PWM2_PERIOD/4) = range;	//Set period PWM2
	*(pwm+RK3288_PWM3_PERIOD/4) = range;	//Set period PWM3
	*(pwm+RK3288_PWM2_CTR/4) |= (1<<0); //Enable PWM2
	*(pwm+RK3288_PWM3_CTR/4) |= (1<<0); //Enable PWM3
	if(pwm2_value != -1)
		asus_pwm_write(PWM2, pwm2_value);
	if(pwm3_value != -1)
		asus_pwm_write(PWM3, pwm3_value);
}

void asus_set_pwmClock(int divisor)
{
	if (divisor > 0xff)
		divisor = 0x100;
	else if(divisor < 2)
		divisor = 0x02;
	*(pwm+RK3288_PWM2_CTR/4) &= ~(1<<0);	//Disable PWM2
	*(pwm+RK3288_PWM3_CTR/4) &= ~(1<<0);	//Disable PWM3	
	*(pwm+RK3288_PWM2_CTR/4) = (*(pwm+RK3288_PWM2_CTR/4) & ~(0xff << 16)) | ((0xff & (divisor/2)) << 16) | (1<<9) ;	//PWM2 div
	*(pwm+RK3288_PWM3_CTR/4) = (*(pwm+RK3288_PWM3_CTR/4) & ~(0xff << 16)) | ((0xff & (divisor/2)) << 16) | (1<<9) ;	//PWM3 div
	*(pwm+RK3288_PWM2_CTR/4) |= (1<<0); //Enable PWM2
	*(pwm+RK3288_PWM3_CTR/4) |= (1<<0); //Enable PWM3
}

void asus_pwm_write(int pin, int value)
{
	int mode = 0;
	unsigned int range;
	int PWM_CTRL_OFFSET = -1;
	int PWM_PERIOD_OFFSET = -1;
	int PWM_DUTY_OFFSET = -1;
	switch (pin)
	{
		case PWM2:
			PWM_CTRL_OFFSET=RK3288_PWM2_CTR;
			PWM_PERIOD_OFFSET=RK3288_PWM2_PERIOD;
			PWM_DUTY_OFFSET=RK3288_PWM2_DUTY;
			break;
		case PWM3:
			PWM_CTRL_OFFSET=RK3288_PWM3_CTR;
			PWM_PERIOD_OFFSET=RK3288_PWM3_PERIOD;
			PWM_DUTY_OFFSET=RK3288_PWM3_DUTY;
			break;
		default:
			break;
	}
	if(asus_get_pin_mode(pin)==PWM && PWM_CTRL_OFFSET != -1 && PWM_PERIOD_OFFSET != -1 && PWM_DUTY_OFFSET != -1)
	{
		range = *(pwm+PWM_PERIOD_OFFSET/4);
		*(pwm+PWM_CTRL_OFFSET/4) &= ~(1<<0);	//Disable PWM
		*(pwm+PWM_DUTY_OFFSET/4) = range - value; //Set duty
		if(mode == CENTERPWM)
		{
			*(pwm+PWM_CTRL_OFFSET/4) |= (1<<5);		
		}
		else
		{
			*(pwm+PWM_CTRL_OFFSET/4) &= ~(1<<5);
		}
		*(pwm+PWM_CTRL_OFFSET/4) |= (1<<1);
		*(pwm+PWM_CTRL_OFFSET/4) &= ~(1<<2);
		*(pwm+PWM_CTRL_OFFSET/4) |= (1<<4);
		*(pwm+PWM_CTRL_OFFSET/4) |= (1<<0); //Enable PWM
	}
	else
	{
		printf("please set this pin to pwmmode first\n");
	}
}

void asus_pwmToneWrite(int pin, int freq)
{
	int divi, pwm_clock, range;
	switch (pin)
	{
		case PWM2:divi=((*(pwm+RK3288_PWM2_CTR/4) >> 16) & 0xff) << 1; break;
		case PWM3:divi=((*(pwm+RK3288_PWM3_CTR/4) >> 16) & 0xff) << 1; break;
		default:divi=-1;break;
	}
	if(divi == 0)
		divi = 512;
	if (freq == 0)
		asus_pwm_write (pin, 0) ;
	else
	{
		pwm_clock = 74250000 / divi;		//74.25Mhz / divi
		range = pwm_clock / freq ;
		asus_set_pwmRange (range) ;
		asus_pwm_write (pin, range / 2) ;
	}
}

void asus_set_gpioClockFreq(int pin, int freq)
{
	int divi;
	if(pin != 17)
	{
		printf("This pin cannot set as gpio clock\n");
		return;
	}
	divi = 297000000 / freq - 1;
	if (divi > 31)
		divi = 31 ;
	else if(divi < 0)
		divi = 0;
	*(cru+CRU_CLKSEL2_CON/4) = (*(cru+CRU_CLKSEL2_CON/4) & (~(0x1F<<8))) | 0x1f << (8+16) | (divi<<8);
}

int asus_get_pinAlt(int pin)
{
	int alt;
	int bank_clk_en;
	int bank, bank_pin;
	bank = gpioToBank(pin);
	bank_pin = gpioToBankPin(pin);
	bank_clk_en = gpio_clk_disable(pin);
	switch(pin)
	{
		//GPIO0
		case 17 : 
			alt =  ((*(pmu+PMU_GPIO0C_IOMUX/4))>>((pin%8)*2)) & 0x00000003;  
			break;			
		//GPIO5B
		case 160 : 
		case 161 :
		case 162 :
		case 163 :
			alt = ((*(grf+GRF_GPIO5B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		case 164 :
		case 165 :
		case 166 :
		case 167 :
			alt = ((*(grf+GRF_GPIO5B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		
		//GPIO5C
		case 168 : 
			alt = ((*(grf+GRF_GPIO5C_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		case 169 :
		case 170 :
		case 171 :
			alt = ((*(grf+GRF_GPIO5C_IOMUX/4))>>((pin%8)*2)) & 0x00000001;
			break;
		//GPIO6A
		case 184 : 
		case 185 :
		case 187 :
		case 188 :
			alt = ((*(grf+GRF_GPIO6A_IOMUX/4))>>((pin%8)*2)) & 0x00000001;
			break;
		//GPIO7A7
		case 223 : 
			alt = ((*(grf+GRF_GPIO7A_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;

		//GPIO7B
		case 224 : 
		case 225 : 
			alt = ((*(grf+GRF_GPIO7B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		case 226 : 
			alt = ((*(grf+GRF_GPIO7B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		//GPIO7C
		case 233 : 
		case 234 : 
			alt = ((*(grf+GRF_GPIO7CL_IOMUX/4))>>((pin%8)*2)) & 0x00000001;
			break;
		case 238 : 
			alt = ((*(grf+GRF_GPIO7CH_IOMUX/4))>>(((pin-4)%8)*4)) & 0x00000003;
			break;
		case 239 : 
			alt = ((*(grf+GRF_GPIO7CH_IOMUX/4))>>(((pin-4)%8)*4)) & 0x00000007;
			break;

		//GPIO8A
		case 251 : 
		case 254 :
		case 255 :
			alt = ((*(grf+GRF_GPIO8A_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		case 252 : 
		case 253 :
			alt = ((*(grf+GRF_GPIO8A_IOMUX/4))>>((pin%8)*2)) & 0x00000003;  
			break;
		//GPIO8B
		case 256 : 
		case 257 :
			alt = ((*(grf+GRF_GPIO8B_IOMUX/4))>>((pin%8)*2)) & 0x00000003;
			break;
		default:
			alt=-1; 
			break;
	}
	//RPi alt ("   GPIO  "), "ALT0", "ALT1", "ALT2", "ALT3", "ALT4", "ALT5"
	//          0      1        2      3        4      5       6       7
	//RPi alt ("IN", "OUT"), "ALT5", "ALT4", "ALT0", "ALT1", "ALT2", "ALT3"
	int alts[7] = {0, FSEL_ALT0, FSEL_ALT1, FSEL_ALT2, FSEL_ALT3, FSEL_ALT4, FSEL_ALT5};	
	if(alt < 7)
	{
		alt = alts[alt];
	}
    if (alt == 0)
    {
		if (*(gpio0[bank]+GPIO_SWPORTA_DDR_OFFSET/4) & (1<<bank_pin))
			alt = FSEL_OUTP;
		else
			alt = FSEL_INPT;
    }
	gpio_clk_recovery(pin, bank_clk_en);
	return alt;
}

void SetGpioMode(int pin, int alt)
{
	alt = ~alt & 0x3;
	switch(pin)
	{
		//GPIO0
		case 17 : 
			*(pmu+PMU_GPIO0C_IOMUX/4) = (*(pmu+PMU_GPIO0C_IOMUX/4) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2)));
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
			*(grf+GRF_GPIO5B_IOMUX/4) = (*(grf+GRF_GPIO5B_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2)));
			break;
		
		//GPIO5C
		case 168 : 
		case 169 :
		case 170 :
		case 171 :
			*(grf+GRF_GPIO5C_IOMUX/4) =  (*(grf+GRF_GPIO5C_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2)));
			break;

		//GPIO6A
		case 185 :
			#ifdef CONFIG_I2S_SHORT
			*(grf+GRF_GPIO6A_IOMUX/4) =  (*(grf+GRF_GPIO6A_IOMUX/4) | (0x0f<<((pin%8)*2+16)) | (0xf << ((pin % 8)*2))) & (~((alt | (alt<<2))<<((pin%8)*2)));
			#else
			*(grf+GRF_GPIO6A_IOMUX/4) =  (*(grf+GRF_GPIO6A_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2)));
			#endif
			break;
		case 184 :
		case 187 :
		case 188 :
			*(grf+GRF_GPIO6A_IOMUX/4) =  (*(grf+GRF_GPIO6A_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2)));
			break;

		//GPIO7A7
		case 223 : 
			*(grf+GRF_GPIO7A_IOMUX/4) =  (*(grf+GRF_GPIO7A_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2))); 
			break;

		//GPIO7B
		case 224 : 
		case 225 : 
		case 226 : 
			*(grf+GRF_GPIO7B_IOMUX/4) =  (*(grf+GRF_GPIO7B_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2))); 
			break;
		//GPIO7C
		case 233 : 
		case 234 : 
			*(grf+GRF_GPIO7CL_IOMUX/4) = (*(grf+GRF_GPIO7CL_IOMUX/4) | (0x0f<<(16+(pin%8)*4)) | (0x3 << ((pin % 8)*4))) & (~((alt)<<((pin%8)*4))); 
			break;
		case 238 : 			
		case 239 : 
			*(grf+GRF_GPIO7CH_IOMUX/4) =  (*(grf+GRF_GPIO7CH_IOMUX/4) | (0x0f<<(16+(pin%8-4)*4)) | (0x3 << ((pin%8-4)*4))) & (~(alt<<((pin%8-4)*4)));  
			break;

		//GPIO8A
		case 251 : 
		case 254 :
		case 255 :			
		case 252 : 
		case 253 :
			*(grf+GRF_GPIO8A_IOMUX/4) =  (*(grf+GRF_GPIO8A_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2))); 
			break;
		//GPIO8B
		case 256 : 
		case 257 :
			*(grf+GRF_GPIO8B_IOMUX/4) = (*(grf+GRF_GPIO8B_IOMUX/4) | (0x03<<((pin%8)*2+16)) | (0x3 << ((pin % 8)*2))) & (~(alt<<((pin%8)*2))); 
			break;
		default:
			printf("wrong gpio\n");
			break;
	}
}

void asus_set_pinAlt(int pin, int alt)
{
	int bank_clk_en;
	int bank, bank_pin;
	if(!gpio_is_valid(pin))
		return;
	bank = gpioToBank(pin);
	bank_pin = gpioToBankPin(pin);
	bank_clk_en = gpio_clk_disable(pin);
	switch(alt)
	{
		case FSEL_INPT:
			SetGpioMode(pin, 0x00);
			*(gpio0[bank]+GPIO_SWPORTA_DDR_OFFSET/4) &= ~(1<<bank_pin);
			break;
		case FSEL_OUTP:
			SetGpioMode(pin, 0x00);
			*(gpio0[bank]+GPIO_SWPORTA_DDR_OFFSET/4) |= (1<<bank_pin);
			break;
		case FSEL_ALT0:
			SetGpioMode(pin, 0x01);
			break;
		case FSEL_ALT1:
			SetGpioMode(pin, 0x02);
			break;
		case FSEL_ALT2:
			SetGpioMode(pin, 0x03);
			break;
		case FSEL_ALT3:
			SetGpioMode(pin, 0x04);
			break;
		case FSEL_ALT4:
			SetGpioMode(pin, 0x05);
			break;
		case FSEL_ALT5:
			SetGpioMode(pin, 0x06);
			break;
		default:
			break;
	}
	gpio_clk_recovery(pin, bank_clk_en);
}

void asus_cleanup(void)
{
	int i;
	for(i=0;i<GPIO_BANK;i++)
	{
    	munmap((caddr_t)gpio_map0[i], BLOCK_SIZE);
	}
	munmap((caddr_t)grf_map, BLOCK_SIZE);
	munmap((caddr_t)pwm_map, BLOCK_SIZE);
	munmap((caddr_t)pmu_map, BLOCK_SIZE);
	munmap((caddr_t)cru_map, BLOCK_SIZE);
}
