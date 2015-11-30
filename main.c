#include <stm32f3xx.h>
#include <command_struct.h>
#include <adc.h>
#include <spi.h>
#include <gpio.h>


//Prototypes
void ms_delay(int ms);
//uint8_t SPIsend(uint8_t data);
unsigned short SPIsend(unsigned short data);



static void SystemClock_Config(void);
void green_led(void);

__irq void DMA1_Channel1_IRQHandler(void);
__irq void DMA1_Channel3_IRQHandler(void);

 __irq void SPI1_IRQHandler(void);


void ms_delay(int ms) {
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0)
         __asm("nop");
   }
}


//GLOBAL VARIABLES/////
static short j=10;
unsigned short adc_resultA[1400];
unsigned short adc_resultB[350];


   
unsigned int saveNDTR;
static int flag=0;

unsigned int adc_ht_done=0;
unsigned int adc_tc_done=0;

unsigned int dma_cndtr;

unsigned short adc_htcnt=0;
unsigned short adc_tccnt=0;

unsigned short spi_cnt=0;

unsigned int spindtr=0;
unsigned int spi_rx_cnt=0;

unsigned int spdtxdma=0;
unsigned int restxdma=0;
unsigned int spirxINT=0;
unsigned int flaggg=0;

unsigned int ifhtd=0;
unsigned int iftcd=0;

volatile int h=0;
volatile int reset_flag=0;

 static short *ptr;
unsigned short recv_data[10];
unsigned short recv_cmd[20];

static void SystemClock_Config(void)

{

RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  //APB1=36Mhz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; //APB2=36Mhz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();

}

void green_led(void)
{
  
}



unsigned short SPIsend(unsigned short data)
{
  while(!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR=data;

	

	while(!(SPI1->SR & SPI_SR_RXNE));
//	while (SPI1->SR & SPI_SR_BSY);

	return (SPI1->DR);
//	return 0;
	
}


	
 
 


__irq void DMA1_Channel1_IRQHandler(void)
 {
   
   if (DMA1->ISR & DMA_ISR_HTIF1)
   {  
     adc_resultA[1]=adc_resultA[1]+2;
     adc_ht_done=1;
     adc_htcnt++;
     DMA1->IFCR=DMA_IFCR_CHTIF1;
   }
   
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
      adc_tc_done=1;
        adc_tccnt++;
        adc_resultA[1399]=adc_resultA[1399]+2;
      DMA1->IFCR=DMA_IFCR_CTCIF1;
    }
   
  }

 __irq void DMA1_Channel3_IRQHandler(void)
{

     if (DMA1->ISR & DMA_ISR_TCIF3)
     {
             spi_cnt++;

        DMA1->IFCR=DMA_IFCR_CTCIF3; //clear interrupt
  //      DMA1->IFCR=DMA_IFCR_CHTIF3;
        

     }
     
     
     if (DMA1->ISR & DMA_ISR_HTIF3)
   {  
               //   spi_cnt++;

     DMA1->IFCR=DMA_IFCR_CHTIF3;
   }
}


 __irq void SPI1_IRQHandler()
 {
  // spi_rx_cnt++;
 // SPI1->CR2&=0xFFBF;
   *ptr=SPI1->DR;
 
  // SPI1->CR2&=0xFFBF; //disable interrupt RXNEIE bit


if (*ptr!=0x0000)
{
  h=1;
}
else
{
  h=0;

}
          

 }
 
void main () {
 
  SystemClock_Config();
  
ptr=&recv_data[0];
int i=0;


  __GPIOF_CLK_ENABLE();
   adc_resultA[0]=0xA5A5;
   adc_resultA[1]=0;
   adc_resultA[1398]=0xB9B9;
   adc_resultA[1399]=1;


		//APB2=No predivisor=Max Clock=84Mhz
	//peripheral clock enable register ,enable SPI1 clock
	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN ; 
	
	//Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;                 
	
        //DMA1 Clock
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	
        //Enable TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//* Enbale GPIOC clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	//* Enbale GPIOB clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
       
        RCC->CFGR|=RCC_CFGR_ADCPRE_0|RCC_CFGR_ADCPRE_1; //PCLK Div by 8

NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


NVIC_SetPriority ( DMA1_Channel1_IRQn,4); //enable DMA for ADC int
NVIC_EnableIRQ (DMA1_Channel1_IRQn); 
//
NVIC_SetPriority ( DMA1_Channel3_IRQn,4); //enable DMA for SPI_Tx int
NVIC_EnableIRQ (DMA1_Channel3_IRQn);        

NVIC_SetPriority ( SPI1_IRQn,4); //enable SPI_RX int
NVIC_EnableIRQ (SPI1_IRQn); 

init_gpio();
     
     
init_adc_dma();      
     
init_adc();
	 
    
							
	 enable_spi();

init_spi_dma(); 
                  
   
                        
                enable_int_spi();
               spi_cs_enable();
        			


while(1)
{
 
    if (adc_ht_done==1)
     
    {
      DMA1->IFCR|=DMA_IFCR_CTCIF3;
      DMA1->IFCR|=DMA_IFCR_CHTIF3;
      DMA1_Channel3->CMAR = (uint32_t)&adc_resultA[0]; 
      DMA1_Channel3->CCR &=~(DMA_CCR_EN);
      DMA1_Channel3->CNDTR =700;
      DMA1_Channel3->CCR |=DMA_CCR_EN;
      SPI1->CR1|=SPI_CR1_SPE;
      adc_ht_done=0;
      ifhtd++;
    }
    
     if (adc_tc_done==1)
    {
      DMA1->IFCR|=DMA_IFCR_CTCIF3;
      DMA1->IFCR|=DMA_IFCR_CHTIF3;
      DMA1_Channel3->CMAR = (uint32_t)&adc_resultA[700]; 
      DMA1_Channel3->CCR &=~(DMA_CCR_EN);
      DMA1_Channel3->CNDTR =700;
      DMA1_Channel3->CCR |=DMA_CCR_EN;
      SPI1->CR1|=SPI_CR1_SPE;
      adc_tc_done=0;
      iftcd++;
    }
  
  
  
  if  (h==1) //(reset_flag==1)
  {
    h=0;
    flaggg++;
    unsigned int mosi_high=0;
    
  disable_adc();
    
    suspend_SPITX_DMA();
    //     suspend_SPIRX_DMA();
    spi_cs_disable();
    
    
    disable_spi();
    enable_spi();
    //        disable_int_spi();
    
    SPI1->CR1|=SPI_CR1_SPE;  
    spi_cs_enable();
    
    for(mosi_high=0;mosi_high<2;mosi_high++)
    {
      while(!(SPI1->SR & SPI_SR_TXE));
      SPI1->DR=0xFFFF;  
      

  }   
 ms_delay(10);


  
  
  for(mosi_high=0;mosi_high<5;mosi_high++)
  {
     while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR=0xFFFF;  
  //  while(!(SPI1->SR & SPI_SR_RXNE));
    recv_cmd[mosi_high]=SPI1->DR;
    
   ms_delay(1); //Giving 1ms for slave to prepare next CMD element
  }   
  ms_delay(100);
dma_cndtr= process_cmd(&recv_cmd[3]);
 
            spi_cs_disable();
      
            
              spi_cs_enable();
 while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR=recv_cmd[3];  
  
   
recv_data[0]=0x0000;

//DMA1->IFCR = 0xFFFFFFFF;

  adc_ht_done=0;
   adc_tc_done=0;
   adc_resultA[1]=0;
   adc_resultA[1399]=1;
enable_adc(dma_cndtr);

    
   
  }
    /*  //GREEN orange LEd GGPIOC Settings
        GPIOC->MODER |=GPIO_MODER_MODER12_0;
    GPIOC->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR10; 
      GPIOC->BSRRL|= GPIO_BSRR_BS_12 ;*/
h=0;

  
  
}
	






while(1);
}