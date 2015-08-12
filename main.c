#include <stm32f3xx.h>
#include <command_struct.h>

//Prototypes
void ms_delay(int ms);
//uint8_t SPIsend(uint8_t data);
unsigned short SPIsend(unsigned short data);

int set_pin_AF(int p);
int AF_sel(int p);
int set_ospeed(int p);
int set_pulldir(int p);


void suspend_SPITX_DMA(void);
void resume_SPITX_DMA(void);

void suspend_SPIRX_DMA(void);
void resume_SPIRX_DMA(void);

void spi_cs_enable(void);
void spi_cs_disable(void);

void enable_spi(void);
void disable_spi(void);

static void SystemClock_Config(void);

void enable_int_spi(void);
void disable_int_spi(void);

void enable_adc(void);
void disable_adc(void);

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


//GLOBAL VARIABLES
static short j=10;
unsigned short adc_resultA[350];
unsigned short adc_resultB[350];


   
unsigned int saveNDTR;
static int flag=0;

unsigned int adc_ht_done=0;
unsigned int adc_tc_done=0;
   
unsigned short adc_htcnt=0;
unsigned short adc_tccnt=0;

unsigned short spi_cnt=0;


unsigned int spdtxdma=0;
unsigned int restxdma=0;
unsigned int spirxINT=0;
unsigned int flaggg=0;

volatile int h=0;
volatile int reset_flag=0;

 static short *ptr;
unsigned short recv_data[10];
unsigned short recv_cmd[20];

static void SystemClock_Config(void)

{

 RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
  }



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
	SPI1->CR1 |=SPI_CR1_BR_0; // Baud Rate as  fpclk/4 (21 Mhz) where fpclk is APB2 clock=84Mhz

	SPI1->CR1 |= SPI_CR1_SSM ;
	SPI1->CR1 |= SPI_CR1_SSI;                       
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
    
  spdtxdma++;
//while(!(SPI1->SR & SPI_SR_TXE));
// while(!(SPI1->SR & SPI_SR_RXNE));

 //Emable DMA Stream for SPI
              DMA1_Channel3->CCR &=0xFFFFFFFE;  //toggle EN bit from 1 to 0
         //         DMA1_Channel3->CR ^=DMA_SxCR_EN;  //toggle EN bit from 1 to 0
while ( DMA1_Channel3->CCR & DMA_CCR_EN ); //break out when DMA_SxCR_EN==0
               
}
//void resume_SPITX_DMA(void)
//{
//  
//}


void enable_adc()
{
  
//  ADC1->SR &=~(ADC_SR_OVR);

        DMA1_Channel1->CMAR = (uint32_t)&adc_resultA[2];
        DMA1_Channel1->CNDTR =346; //46 readings transfer
        //Emable DMA Stream for ADC
        DMA1_Channel1->CCR |=DMA_CCR_EN;
          
     	ADC1->CR2 |= ADC_CR2_ADON;  //ADC ON
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
        ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
//	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
        
}

void disable_adc()
{               
  ADC1->CR2&=0xFFFFFFFE;
  DMA1_Channel1->CCR &=0xFFFFFFFE;  
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
 
 
void spi_cs_enable(void)
{
    GPIOB->BSRRH|=0x0040 ; //CS Enable (low)
}

 void spi_cs_disable(void)
{
      GPIOB->BSRRL|=0x0040;                  //CS Disable (high) 
}

__irq void DMA1_Channel1_IRQHandler()
 {
   
   if (DMA1->ISR & DMA_ISR_HTIF1)
   {  
     adc_ht_done=1;
     adc_htcnt++;
     DMA1->IFCR=DMA_IFCR_CHTIF1;
   }
   
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
      adc_tc_done=1;
        adc_tccnt++;
        adc_resultA[349]=adc_resultA[349]+1;
      DMA1->IFCR=DMA_IFCR_CTCIF1;
    }
   
  }

 __irq void DMA1_Channel3_IRQHandler()
{
//       if (DMA2->LISR & DMA_LISR_HTIF3)
//   {
//     adc_ht_done=0;
//     DMA2->LIFCR=DMA_LIFCR_CHTIF3;}
//   
//    if (DMA2->LISR & DMA_LISR_TCIF3)
//    {
//      adc_tc_done=0;
//      DMA2->LIFCR=DMA_LIFCR_CTCIF3;
//    }
        DMA1->IFCR|=DMA_IFCR_CTCIF3; //clear interrupt
        DMA1->IFCR|=DMA_IFCR_CHTIF3;
        
      spi_cnt++;


}


 __irq void SPI1_IRQHandler()
 {
   
 // SPI1->CR2&=0xFFBF;
   *ptr=SPI1->DR;
 
  // SPI1->CR2&=0xFFBF; //disable interrupt RXNEIE bit


if (*ptr!=0x0000)
{
   //disable interrupt RXNEIE bit
  h=1;
 // ptr++;
}
else
{
  h=0;
//SPI1->CR2|= SPI_CR2_RXNEIE;  
}
          
 //SPI1->CR2|= SPI_CR2_RXNEIE;
 }
 
void main () {
 
  SystemClock_Config();
  
ptr=&recv_data[0];
int i=0;

//  for(i=0;i<48;i++)
//  {
//    adc_resultA[i]=0xFF;
//  }
//  


  adc_resultA[0]=0xA5A5;
  adc_resultA[1]=0xA5A5;
  adc_resultA[348]=0xB9B9;
  adc_resultA[349]=0;


		//APB2=No predivisor=Max Clock=84Mhz
	//peripheral clock enable register ,enable SPI1 clock
	RCC->APB2ENR |=  RCC_APB2ENR_SPI1EN ; 
	
	//Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN ;                 
	
        //DMA1 Clock
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	
        //Enable TIM3
//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//* Enbale GPIOA clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	//* Enbale GPIOB clock */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
       


NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


NVIC_SetPriority ( DMA1_Channel1_IRQn,4); //enable DMA for ADC int
NVIC_EnableIRQ (DMA1_Channel1_IRQn); 
//
NVIC_SetPriority ( DMA1_Channel3_IRQn,4); //enable DMA for SPI_Tx int
NVIC_EnableIRQ (DMA1_Channel3_IRQn);        

NVIC_SetPriority ( SPI1_IRQn,4); //enable SPI_RX int
NVIC_EnableIRQ (SPI1_IRQn); 


        GPIOC->MODER |=set_pin_AF(6);  //NSS
	GPIOC->AFR[0]|=AF_sel(6);
	GPIOC->OTYPER |=0;
	GPIOC->OSPEEDR |=set_ospeed(6); 
//	GPIOC->PUPDR|=set_pulldir(6);


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

	GPIOB->MODER |=(0x01<<12);
	GPIOB->OTYPER |=0;
	GPIOB->OSPEEDR |=set_ospeed(6); 
	GPIOB->BSRRL|=0x0040;                  //set portB pin6 as output=1,CS Disable (high)

  //      GPIOB->PUPDR|=0x1000; //pullup;
 //       GPIOB->PUPDR|=0x2000;//pulldown
        
	GPIOA->MODER |= 0X0000000F; //FOR ADC MCU pins PA0 and PA1 set to analog mode


              
/****************************************************************************************************/
        /* DMA Config For ADC */
/****************************************************************************************************/            
             
      	//DMA CONFIG FOR ADC
                DMA1_Channel1->CCR &= 0;
		while ( (DMA1_Channel1->CCR !=0));
   
                DMA1_Channel1->CPAR |= (uint32_t)&ADC1->DR;
                DMA1_Channel1->CMAR = (uint32_t)&adc_resultA[2];
                DMA1_Channel1->CNDTR =346; //46 readings transfer
                
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
        
/****************************************************************************************************/
              
        ADC1->CR2 |= ADC_CR2_ADON;  //First ADC ON
        
    //    ADC->CCR |= ADC_CCR_ADCPRE_0;
     //   ADC->CCR |=  ADC_CCR_ADCPRE_1;
	//ADC1->SQR3|=0x00000001;
               
	ADC1->SQR1|=0x00000010;
        ADC1->CR2 |=ADC_CR2_TSVREFE;
	ADC1->CR2 |= ADC_CR2_ADON;  //Second time ADC ON to start conversion
        ADC1->CR2 |= ADC_CR2_CONT;   //continous conversion until bit cleared
       ADC1->CR2 |=ADC_CR2_DMA; //use DMA for data transfer
//	ADC1->CR2 |=ADC_CR2_DDS; //DMA requests are issued as long as data is converted and DMA=1
        ADC1->CR2 |=ADC_CR2_SWSTART;	//Start conversion
	
	 
    
							
	 enable_spi();


/****************************************************************************************************/
        /* DMA Config For SPI_TX */
/****************************************************************************************************/
		DMA1_Channel3->CCR &= 0;
		while ( (DMA1_Channel3->CCR !=0));
	
                //DMA CONFIG for SPI_TX
		DMA1_Channel3->CPAR |= (uint32_t)&SPI1->DR;
               DMA1_Channel3->CMAR |= (uint32_t)&adc_resultA[0]; 
		DMA1_Channel3->CNDTR =175;
               DMA1_Channel3->CCR |=DMA_CCR_TCIE; //FUll transfer interrupt enabled
      //      DMA1_Channel3->CCR |=DMA_CCR_HTIE;//half transfer interrupt enabled
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
                
/****************************************************************************************************/                   
   
                        
                enable_int_spi();
               spi_cs_enable();
        			
  SPI1->CR1 &=0x00000000;
	SPI1->CR2 |=SPI_CR2_DS; //16 bit data frame
	SPI1->CR1 |=SPI_CR1_BR_0; // Baud Rate as  fpclk/4 (21 Mhz) where fpclk is APB2 clock=84Mhz

	//SPI1->CR1 |= SPI_CR1_SSM ;
	SPI1->CR1 |= SPI_CR1_SSI;        
        SPI1->CR2 |= SPI_CR2_SSOE; 
	SPI1->CR1 |=SPI_CR1_MSTR;
 SPI1->CR1|=SPI_CR1_SPE;
while(1)
{
 /*
    if (adc_ht_done==1)
     
    {
      DMA1->IFCR|=DMA_IFCR_CTCIF3;
      DMA1->IFCR|=DMA_IFCR_CHTIF3;
      DMA1_Channel3->CMAR = (uint32_t)&adc_resultA[0]; 
      DMA1_Channel3->CNDTR =175;
      DMA1_Channel3->CCR |=DMA_CCR_EN;
      SPI1->CR1|=SPI_CR1_SPE;
      adc_ht_done=0;
    }
    
     if (adc_tc_done==1)
    {
      DMA1->IFCR|=DMA_IFCR_CTCIF3;
      DMA1->IFCR|=DMA_IFCR_CHTIF3;
      DMA1_Channel3->CMAR = (uint32_t)&adc_resultA[175]; 
      DMA1_Channel3->CNDTR =175;
      DMA1_Channel3->CCR |=DMA_CCR_EN;
      SPI1->CR1|=SPI_CR1_SPE;
      adc_tc_done=0;
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
   // while(!(SPI1->SR & SPI_SR_RXNE));
    recv_cmd[mosi_high]=SPI1->DR;
    
   ms_delay(1); //Giving 1ms for slave to prepare next CMD element
  }   
  ms_delay(100);
 process_cmd(&recv_cmd[1]);
 
            spi_cs_disable();
      
            
              spi_cs_enable();


recv_data[0]=0x0000;

DMA1->IFCR &= 0xFFFFFFFF;

  adc_ht_done=0;
   adc_tc_done=0;
enable_adc();

    
    
   
  }
h=0;

*/
 int mosi_high;
      for(mosi_high=0;mosi_high<10;mosi_high++)
    {
    //  while(!(SPI1->SR & SPI_SR_TXE));
      SPI1->DR=0xFFFF;  
      

  }   
  
}
	






while(1);
}