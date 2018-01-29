#ifndef	__RKIO_H__
#define	__RKIO_H__

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
#define VOP0_PWM	 57
#define VOP1_PWM	 58


#define GPIO0_C1		17			//7----->17

#define GPIO5_B0		(8+152)		//8----->160
#define GPIO5_B1		(9+152)		//9----->161
#define GPIO5_B2		(10+152)	//10----->162
#define GPIO5_B3		(11+152)	//11----->163
#define GPIO5_B4		(12+152)	//12----->164
#define GPIO5_B5		(13+152)	//13----->165
#define GPIO5_B6		(14+152)	//14----->166
#define GPIO5_B7		(15+152)	//15----->167
#define GPIO5_C0		(16+152)	//16----->168
#define GPIO5_C1		(17+152)	//17----->169
#define GPIO5_C2		(18+152)	//18----->170
#define GPIO5_C3		(19+152)	//19----->171

#define GPIO6_A0		(184)		//0----->184
#define GPIO6_A1		(1+184)		//1----->185
#define GPIO6_A3		(3+184)		//3----->187
#define GPIO6_A4		(4+184)		//4----->188

#define GPIO7_A0		(0+216)		//0----->216
#define GPIO7_A7		(7+216)		//7----->223
#define GPIO7_B0		(8+216)		//8----->224
#define GPIO7_B1		(9+216)		//9----->225
#define GPIO7_B2		(10+216)	//10----->226
#define GPIO7_C1		(17+216)	//17----->233
#define GPIO7_C2		(18+216)	//18----->234
#define GPIO7_C6		(22+216)	//22----->238
#define GPIO7_C7		(23+216)	//23----->239

#define GPIO8_A3		(3+248)		//3----->251
#define GPIO8_A4		(4+248)		//4----->252
#define	GPIO8_A5		(5+248)		//5----->253
#define GPIO8_A6		(6+248)		//6----->254
#define GPIO8_A7		(7+248)		//7----->255
#define GPIO8_B0		(8+248)		//8----->256
#define GPIO8_B1		(9+248)		//9----->257

#define RK3288_PMU		0xff730000
#define PMU_GPIO0C_IOMUX	0x008c


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

/* Pull up / down / Z */
#define PMU_GPIO0C_P	0x006c
#define GRF_GPIO1D_P	0x014c
#define GRF_GPIO2A_P	0x0150
#define GRF_GPIO2B_P	0x0154
#define GRF_GPIO2C_P	0x0158
#define GRF_GPIO3A_P	0x0160
#define GRF_GPIO3B_P	0x0164
#define GRF_GPIO3C_P	0x0168
#define GRF_GPIO3D_P	0x016c
#define GRF_GPIO4A_P	0x0170
#define GRF_GPIO4B_P	0x0174
#define GRF_GPIO4C_P	0x0178
#define GRF_GPIO4D_P	0x017c
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

/* Drive Strength */
#define PMU_GPIO0C_E	0x0078
#define GRF_GPIO1D_E	0x01cc
#define GRF_GPIO2A_E	0x01d0
#define GRF_GPIO2B_E	0x01d4
#define GRF_GPIO2C_E	0x01d8
#define GRF_GPIO3A_E	0x01e0
#define GRF_GPIO3B_E	0x01e4
#define GRF_GPIO3C_E	0x01e8
#define GRF_GPIO3D_E	0x01ec
#define GRF_GPIO4A_E	0x01f0
#define GRF_GPIO4B_E	0x01f4
#define GRF_GPIO4C_E	0x01f8
#define GRF_GPIO4D_E	0x01fc
#define GRF_GPIO5B_E	0x0204
#define GRF_GPIO5C_E	0x0208
#define GRF_GPIO6A_E	0x0210
#define GRF_GPIO6B_E	0x0214
#define GRF_GPIO6C_E	0x0218
#define GRF_GPIO7A_E	0x0220
#define GRF_GPIO7B_E	0x0224
#define GRF_GPIO7C_E	0x0228
#define GRF_GPIO8A_E	0x0230
#define GRF_GPIO8B_E	0x0234


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
#define CRU_CLKSEL2_CON				0x0068
#define CRU_CLKGATE14_CON			0x0198
#define CRU_CLKGATE17_CON			0x01a4
//#define PWM0 26
//#define PWM1 27
#define PWM0 GPIO7_A0
#define PWM2 GPIO7_C6
#define PWM3 GPIO7_C7



#endif
