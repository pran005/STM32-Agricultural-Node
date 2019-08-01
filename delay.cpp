#include <stm32f10x.h> 
#include "delay.h" 


void delayms(uint32_t ms)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 
	TIM3->PSC = 23999;	         										 			 //Set prescaler to 24 000 (PSC + 1)
  TIM3->ARR = ms;	            									  	    // Auto reload value 
	TIM3->CR1 = TIM_CR1_CEN;
  while((TIM3->SR & TIM_SR_UIF)==0) ; 
  TIM3->SR &= ~TIM_SR_UIF;	
}
