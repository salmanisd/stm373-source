#include <stm32f3xx.h>
#include <adc.h>
#include <main.h>

void init_adc_dma(void)
{
               
      	//DMA CONFIG FOR ADC
                DMA1_Channel1->CCR &= 0;
		while ( (DMA1_Channel1->CCR !=0));
   
                DMA1_Channel1->CPAR |= (uint32_t)&ADC1->DR;
                DMA1_Channel1->CMAR = (uint32_t)&adc_resultA[2];
                DMA1_Channel1->CNDTR =1395; //46 readings transfer
                
                DMA1_Channel1->CCR |=DMA_CCR_TCIE; //full transfer interrupt enabled
                DMA1_Channel1->CCR |=DMA_CCR_HTIE;//half transfer interrupt enabled
                
                DMA1_Channel1->CCR |=DMA_CCR_PSIZE_0;   //Set Peripheral data size to 16bits
                DMA1_Channel1->CCR |=DMA_CCR_MSIZE_0;
		DMA1_Channel1->CCR |=DMA_CCR_PL_1;     // High prority
          //      DMA1_Channel1->CCR |=DMA_SxCR_PL_0; 
                DMA1_Channel1->CCR |=DMA_CCR_CIRC; //circular mode set
                DMA1_Channel1->CCR |=DMA_CCR_MINC; //memory increment
                
		//	DMA1_Channel1->CCR |=(1<<6); //direction
                //DMA1_Channel1->CCR |= (1<<5) ; //[perh is flowcontroller
                 //Emable DMA Stream for ADC
               DMA1_Channel1->CCR |=DMA_CCR_EN;
        
}

void init_adc(void)
{
      unsigned int seq,i=0;

              
              ADC1->CR2 |= ADC_CR2_ADON;  //First ADC ON
              ADC1->CR1 |=  ADC_CR1_SCAN;
             ADC1->SQR1|=  (0<<20); //1conversions
//	ADC1->SQR1|=ADC_SQR1_L_0
  
  
            ADC1->CR2 |=ADC_CR2_TSVREFE; //internal temp sensor
                   
            //add channnels 0-15 in the sequence registers SQRx             
           seq=0;                                                            
           for (i=0;i<=5;i++)
            {
            ADC1->SQR3|=(i<<seq);
            seq=seq+5;
            }
            
           seq=0;      
           for (i=6;i<=11;i++)
           {  
           ADC1->SQR2|=(i<<seq);
           seq=seq+5;
           }
  
          seq=0;        
          for (i=12;i<=15;i++)
          {
           ADC1->SQR1|=(i<<seq);
           seq=seq+5;
          }
            
   //     ADC1->SMPR1 |=  ADC_SMPR1_SMP16_0|ADC_SMPR1_SMP16_1|ADC_SMPR1_SMP16_2 ; //Sample Time 239.5 cycles
	ADC1->CR2 |= ADC_CR2_ADON;  //Second time ADC ON to start conversion
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
       ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
//	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
	
  
}

void disable_adc(void)
{               
  ADC1->CR2&=0xFFFFFFFE;
  DMA1_Channel1->CCR &=0xFFFFFFFE;  
}

void enable_adc(void)
{
  /*
unsigned int num_chans,seq,i=0;

if (num_chans >= 1 && num_chans <= 6)
{
  seq=0;
  
  for (i=1;i<=num_chans;i++)
  {
   ADC1->SQR3|=(i<<seq);
   seq=seq+5;
  }
  
}
  

if (num_chans >= 7 && num_chans <= 12)
{
  seq=0;
  
  for (i=1;i<=num_chans;i++)
  {
   ADC1->SQR2|=(i<<seq);
   seq=seq+5;
  }
  
}

if (num_chans >= 13 && num_chans <= 16)
{
  seq=0;
  
   for (i=1;i<=6;i++)
  {
   ADC1->SQR3|=(i<<seq);
   seq=seq+5;
  }
  
  seq=0;
  
   for (i=7;i<=12;i++)
  {
   ADC1->SQR2|=(i<<seq);
   seq=seq+5;
  }
  
   seq=0;

  for (i=1;i<=num_chans;i++)
  {
   ADC1->SQR1|=(i<<seq);
   seq=seq+5;
  }
  
}
  */
                
     
  
        DMA1_Channel1->CMAR = (uint32_t)&adc_resultA[2];
        DMA1_Channel1->CNDTR =1396; //46 readings transfer
        //Emable DMA Stream for ADC
        DMA1_Channel1->CCR |=DMA_CCR_EN;
          
     	ADC1->CR2 |= ADC_CR2_ADON;  //ADC ON
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
        ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
//	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
        
}

