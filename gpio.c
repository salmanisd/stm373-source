#include <stm32f3xx.h>
#include <gpio.h>


int set_pin_AF(int p)
	{
		p=2<<(p*2);
		return p;
	}

int AF_sel(int p)
	{
		p=5<<(p*4); //5(0101) for AF=SPI
		return p;
	}

int set_ospeed(int p)
	{
		p=1<<(p*2);
		return p;
	}

	
int set_pulldir(int p) //Pin->Pulledup
	{
		p=1<<(p*2);
		return p;
	}

void init_gpio(void)
{
/*
        GPIOC->MODER |=set_pin_AF(6);  //NSS
	GPIOC->AFR[0]|=AF_sel(6);
	GPIOC->OTYPER |=0;
	GPIOC->OSPEEDR |=set_ospeed(6); 
//	GPIOC->PUPDR|=set_pulldir(6);
*/

	GPIOC->MODER |=set_pin_AF(7);  //SPICLK
	GPIOC->AFR[0]|=AF_sel(7);
	GPIOC->OTYPER |=0;
	GPIOC->OSPEEDR |=set_ospeed(7); 
//	GPIOC->PUPDR|=set_pulldir(7);

	GPIOC->MODER |=set_pin_AF(8);    //MISO
	GPIOC->AFR[1]|=5;
	GPIOC->OTYPER |=0;
	GPIOC->OSPEEDR |=set_ospeed(8); 
//	GPIOC->PUPDR|=set_pulldir(8);
//GPIOA->PUPDR|=0x1000; //pullup when Slave missing
//GPIOA->PUPDR|=0x2000;// pulldown


	GPIOC->MODER |=set_pin_AF(9);   //MOSI
	GPIOC->AFR[1]|=5<<4 ;
	GPIOC->OTYPER |=0;
	GPIOC->OSPEEDR |=set_ospeed(9); 
//	GPIOC->PUPDR|=set_pulldir(9);

	GPIOC->MODER |=(0x01<<12);
	GPIOC->OTYPER |=0;
	GPIOC->OSPEEDR |=set_ospeed(6); 
	GPIOC->BSRRL|=0x0040;                  //set portC pin6 as output=1,CS Disable (high)

  //      GPIOB->PUPDR|=0x1000; //pullup;
 //       GPIOB->PUPDR|=0x2000;//pulldown
        
	GPIOA->MODER |= 0X0000000F; //FOR ADC MCU pins PA0 and PA1 set to analog mode

 
        
    //GREEN LED GGPIOC Settings
        GPIOC->MODER |=GPIO_MODER_MODER11_0;
   //     GPIOC->OTYPER |=GPIO_OTYPER_OT_11;
   //     GPIOC->ODR |= GPIO_ODR_11;
        GPIOC->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR11_0; 
      GPIOC->BSRRL|= GPIO_BSRR_BS_11 ;
      
}