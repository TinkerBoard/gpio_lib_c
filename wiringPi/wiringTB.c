#include "wiringTB.h"
#include "wiringPi.h"
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

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

static int pwm_divisor = 124;
static int pwm_range = 1024;

void tinker_board_setup(void)
{
	int i;
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) 
	{
		printf("can't open /dev/mem \n");
		return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    }
	for(i=0;i<9;i++)
    {
		// mmap GPIO 
		gpio_map0[i] = mmap(
			NULL,             // Any adddress in our space will do 
			PAGE_SIZE,       // Map length 
			PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
			MAP_SHARED,       // Shared with other processes 
			mem_fd,           // File to map 
			RK3288_GPIO(i)         //Offset to GPIO peripheral 
		);
		if ((uint32_t)gpio_map0[i] < 0)
			return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
		gpio0[i] = (volatile unsigned *)gpio_map0[i];
   	}//for
	/////////////mmap grf////////////
	grf_map = mmap(
		NULL,             // Any adddress in our space will do 
		PAGE_SIZE,       // Map length 
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
		MAP_SHARED,       // Shared with other processes 
		mem_fd,           // File to map 
		RK3288_GRF_PHYS         //Offset to GPIO peripheral 
	);
	if ((uint32_t)grf_map < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    grf = (volatile unsigned *)grf_map;
	////////////////////////////  
	////////////mmap pwm////////
	pwm_map = mmap(
		NULL,             // Any adddress in our space will do 
		PAGE_SIZE,       // Map length 
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
		MAP_SHARED,       // Shared with other processes 
		mem_fd,           // File to map 
		RK3288_PWM         //Offset to GPIO peripheral 
	);
	if ((uint32_t)pwm_map < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    pwm = (volatile unsigned *)pwm_map;
	////////////////////////////
	////////////mmap pmu//////////
	pmu_map = mmap(
		NULL,             // Any adddress in our space will do 
		PAGE_SIZE,       // Map length 
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory 
		MAP_SHARED,       // Shared with other processes 
		mem_fd,           // File to map 
		RK3288_PMU         //Offset to GPIO peripheral 
	);
	if ((uint32_t)pmu_map < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
    pmu = (volatile unsigned *)pmu_map;
	///////////////////////////////
	////////////mmap cru//////////
	cru_map = mmap(
		NULL,             // Any adddress in our space will do
		PAGE_SIZE,       // Map length
		PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
		MAP_SHARED,       // Shared with other processes
		mem_fd,           // File to map
		RK3288_CRU         //Offset to GPIO peripheral
	);
	if ((uint32_t)cru_map < 0)
		return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem: %s\n", strerror (errno)) ;
	cru = (volatile unsigned *)cru_map;
	///////////////////////////////
	close(mem_fd); // No need to keep mem_fdcru open after mmap
}

int asus_get_pin_mode(int pin)
{
	int value,func;
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
				func = GPIOOUT;
			else
				func = GPIOIN;
		}
		else
		{
			if (*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) & 1<<(pin%32))
				func = GPIOOUT;
			else
				func = GPIOIN;
		}
	}
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
}

void asus_digitalWrite(int pin, int value)
{
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
}

void asus_set_pwmRange(unsigned int range)
{
	pwm_range = range;
}

void asus_set_pwmClock(int divisor)
{
	pwm_divisor = divisor;
}

void asus_pwm_write(int pin, int value)
{
	int pwm_ch;
	int mode = 0;
	unsigned int range = pwm_divisor*pwm_range;
	unsigned int duty = pwm_divisor*value;
	switch (pin)
	{
		case PWM2:pwm_ch=2;break;
		case PWM3:pwm_ch=3;break;
		default:pwm_ch=-1;break;
	}
	if(asus_get_pin_mode(pin)==PWM)
	{
		*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) &= ~(1<<0);	//Disable PWM
		*(pwm+RK3288_PWM0_PERIOD/4+pwm_ch*4) = range;	//Set period
		*(pwm+RK3288_PWM0_DUTY/4+pwm_ch*4) = range - duty; //Set duty
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
		*(pwm+RK3288_PWM0_CTR/4+pwm_ch*4) |= (1<<0); //Enable PWM
	}
	else
	{
		printf("please set this pin to pwmmode first\n");
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
	divi = 297000000 / freq ;
	if (divi > 31)
		divi = 31 ;
	*(cru+CRU_CLKSEL2_CON/4) = (*(cru+CRU_CLKSEL2_CON/4) & (~(0x1F<<8))) | 0x1f << (8+16) | (divi<<8);
}

int asus_get_pinAlt(int pin)
{
	int alt;
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
		if(pin>=24)
		{
			if (*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) & 1<<((pin+8)%32))
				alt = FSEL_OUTP;
			else
				alt = FSEL_INPT;
		}
		else
		{
			if (*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) & 1<<(pin%32))
				alt = FSEL_OUTP;
			else
				alt = FSEL_INPT;
		}
    }
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
		case 184 : 
		case 185 :
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
	}
}

void asus_set_pinAlt(int pin, int alt)
{
	switch(alt)
	{
		case FSEL_INPT:
			SetGpioMode(pin, 0x00);
			if(pin>=24)
			{
				*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) &= ~(1<<((pin+8)%32));
			}
			else
			{
				*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) &= ~(1<<(pin%32));
			}
			break;
		case FSEL_OUTP:
			SetGpioMode(pin, 0x00);
			if(pin>=24)
			{
				*(gpio0[(pin+8)/32]+GPIO_SWPORTA_DDR_OFFSET/4) |= (1<<((pin+8)%32));
			}
			else
			{
				*(gpio0[pin/32]+GPIO_SWPORTA_DDR_OFFSET/4) |= (1<<(pin%32));
			}
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
	}
}