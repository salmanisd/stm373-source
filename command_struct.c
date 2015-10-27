#include <command_struct.h>
#include <stm32f3xx.h>

  unsigned short number_of_channels=0;
  unsigned short adc_config=0;
  unsigned short adc_sample_cycles=0;
  unsigned short adc_clk_div=0;

struct Command {
	unsigned short opcode;
	unsigned short len;
	unsigned short descriptor;
};

unsigned short createMask(unsigned short a, unsigned short b)
{
   unsigned short r = 0;
   unsigned int i;
   for ( i=a; i<=b; i++)
       r |= 1 << i;

   return r;
}


/*
These bits are set and cleared by software to select the frequency of the clock to
the ADC.
00: PCLK divided by 2 =18   M
01: PCLK divided by 4 =9    M
10: PCLK divided by 6 =4.5  M
11: PCLK divided by 8 =2.25 M
*/


void process_cmd(unsigned short* opcode)
{
  unsigned short r;


  r=createMask(0,3);
  number_of_channels=r & (*opcode); 
    
  r=createMask(4,7);
  adc_sample_cycles=(r & (*opcode))>>4;
   
  r=createMask(8,11);
  adc_clk_div=(r & (*opcode))>>8;
    
  r=createMask(12,15);
  adc_config=(r & (*opcode))>>12;   
  
if (adc_config)
{
  
  RCC->CFGR   &=~RCC_CFGR_ADCPRE;//reset to 00 
  ADC1->SMPR1 &= ~ADC_SMPR1_SMP16; //reset to 000<-for chan 16?
  ADC1->SQR1  &= ~ADC_SQR1_L;           //reset length of convserions to zero
  
  switch (adc_clk_div)
  {
      case  2:
      RCC->CFGR|=0<<14; //PCLK Div by 2
      ADC1->SMPR1 |= adc_sample_cycles<<18; 
      break;
  
      case 4:
      RCC->CFGR|=1<<14; //PCLK Div by 4
      ADC1->SMPR1 |= adc_sample_cycles<<18; 
      break;
      
      case 6:
      RCC->CFGR|=2<<14; //PCLK Div by 6
      ADC1->SMPR1 |= adc_sample_cycles<<18; 
      break;
      
      case 8:
      RCC->CFGR|=3<<14; //PCLK Div by 8
      ADC1->SMPR1 |= adc_sample_cycles<<18; 
      break;
      
      default:
        TIM3->PSC = 30000;  
        break;
        
  }
  
  
     ADC1->SQR1|=((number_of_channels-1)<<20 );
    
  
}


}