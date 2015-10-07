#include <stm32f3xx.h>
#include <spi.h>
#include <main.h>

void init_spi_dma(void)
{
  	DMA1_Channel3->CCR &= 0;
		while ( (DMA1_Channel3->CCR !=0));
	
                //DMA CONFIG for SPI_TX
		DMA1_Channel3->CPAR |= (uint32_t)&SPI1->DR;
               DMA1_Channel3->CMAR |= (uint32_t)&adc_resultA[0]; 
		DMA1_Channel3->CNDTR =700;
               DMA1_Channel3->CCR |=DMA_CCR_TCIE; //FUll transfer interrupt enabled
       //       DMA1_Channel3->CCR |=DMA_CCR_HTIE;//half transfer interrupt enabled
		DMA1_Channel3->CCR |=DMA_CCR_PSIZE_0;   //Set Peripheral data size to 16bits
                DMA1_Channel3->CCR |=DMA_CCR_MSIZE_0;   //Set Memory data size to 16bits
                
		DMA1_Channel3->CCR |=DMA_CCR_PL_0;     //Very High prority
                DMA1_Channel3->CCR |=DMA_CCR_PL_1; 
                
		DMA1_Channel3->CCR |=DMA_CCR_MINC;
	//	DMA1_Channel3->CCR |=DMA_SxCR_CIRC; //circular mode set for SPI
		DMA1_Channel3->CCR |=DMA_CCR_DIR; //direction read from memory
		//      DMA1_Channel3->CCR |= (1<<5) ; //[perh is flowcontroller
		
         
                //Emable DMA Stream for SPI
       //     DMA1_Channel3->CCR |=DMA_SxCR_EN;
  
}

void enable_int_spi(void)
{
          SPI1->CR2|=SPI_CR2_TXDMAEN; //DMA request when TX empty flag set
  //    SPI1->CR2|=SPI_CR2_RXDMAEN; //Rx Buffer DMA Enable 
        
       SPI1->CR2|= SPI_CR2_RXNEIE;
}
void disable_int_spi(void)
{
   SPI1->CR2&=0xFFBF; //disable interrupt RXNEIE bit

}
void enable_spi(void)
{
  
        SPI1->CR1 &=0x00000000;
	SPI1->CR2 |=SPI_CR2_DS; //16 bit data frame
	SPI1->CR1 |=SPI_CR1_BR_0; // Baud Rate as  18MHz

	SPI1->CR1 |= SPI_CR1_SSM ;
	SPI1->CR1 |= SPI_CR1_SSI;        
  //      SPI1->CR2 |= SPI_CR2_SSOE; 
	SPI1->CR1 |=SPI_CR1_MSTR;
        

}



void disable_spi(void)
{
 // while(!(SPI1->SR & SPI_SR_RXNE));
  while(!(SPI1->SR & SPI_SR_TXE));

	while (SPI1->SR & SPI_SR_BSY);
          SPI1->CR1 &=0xFFFFFFFE;

}
void suspend_SPITX_DMA(void)
{
    
  //spdtxdma++;
//while(!(SPI1->SR & SPI_SR_TXE));
// while(!(SPI1->SR & SPI_SR_RXNE));

 //Emable DMA Stream for SPI
              DMA1_Channel3->CCR &=0xFFFFFFFE;  //toggle EN bit from 1 to 0
         //         DMA1_Channel3->CR ^=DMA_SxCR_EN;  //toggle EN bit from 1 to 0
while ( DMA1_Channel3->CCR & DMA_CCR_EN ); //break out when DMA_SxCR_EN==0
               
}

void spi_cs_enable(void)
{
    GPIOC->BSRRH|=0x0040 ; //CS Enable (low)
}

 void spi_cs_disable(void)
{
      GPIOC->BSRRL|=0x0040;                  //CS Disable (high) 
}