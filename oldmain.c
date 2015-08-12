#include <stm32f3xx.h>

void main ()
{
SPI1->CR1=SPI_CR1_CPHA;
}