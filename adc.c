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
                DMA1_Channel1->CNDTR =1396; //46 readings transfer
                
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
  

              
        ADC1->CR2 |= ADC_CR2_ADON;  //First ADC ON
        
    //    ADC->CCR |= ADC_CCR_ADCPRE_0;
     //   ADC->CCR |=  ADC_CCR_ADCPRE_1;
	//ADC1->SQR3|=0x00000001;
               
	ADC1->SQR1|=0x00000001;
        ADC1->CR2 |=ADC_CR2_TSVREFE;
        ADC1->SMPR1 |=  ADC_SMPR1_SMP16_0|ADC_SMPR1_SMP16_1|ADC_SMPR1_SMP16_2 ; //Sample Time 239.5 cycles
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
  
//  ADC1->SR &=~(ADC_SR_OVR);

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

