#include <stm32f10x.h> 
#include <string.h> 
#include "delay.h" 
/*
  
								 PC14 GND PB13 									PC6	PC7	PC8 PC9			 
                    |	 |	|					             |   |   |   |
										Rs RW EN  									DB4 DB5 DB6 DB7    
								_____________________________________________________________
							 |  _________________________________________________________	 |
							 | |																												 | |
							 | | 	STM32F100																							 | | 
							 | |	Mini Project	I																				 | | 
							 | |________________________________________________________ | |
							 |_____________________________________________________________|
							 
							 
*/


void cmd (char command) 
{
	
	uint8_t a = (command>>4) & 0x0f  ; 
	uint8_t b = (command & 0x0f) ; 
	
	GPIOB -> BSRR = (1<<30) ;  														//RESET RS  
	GPIOB -> BSRR = (1<<13) ;															//SET E 
  GPIOC -> ODR =  (a<<6) ;
  delayms(5)   ;               
  GPIOB -> BSRR = (1<<29) ; 													 //RESET E 
	
	GPIOB -> BSRR = (1<<13) ;
  GPIOC -> ODR  =  (b<<6) ;
 	delayms(5) ; 								
	GPIOB -> BSRR = (1<<29) ;

}	
				
void lcd_data(char data)
{
	uint8_t a = (data>>4) & 0x0f ; 
	uint8_t b = data & 0x0f ;
	
	  
	GPIOB -> BSRR = (1<<13) | (1<<14); 														 // SET E | SET RS 
	GPIOC -> ODR =  (a<<6) ;
  delayms(5) ;                     
  GPIOB -> BSRR=  (1<<29) ;													 					 //RESET E 
	
	GPIOB -> BSRR = (1<<13) ;
	GPIOC -> ODR =  (b<<6) ;
 	delayms(5) ;  								  
	GPIOB -> BSRR = (1<<29) ;

}
void lcd (char *string) 
{
	while(*string) 
		lcd_data(*string++) ; 
}
void lcd(int m) 
{
	uint8_t i=0, flag=0,r,p  ;
	char str[100] ;	
	if(m==0)
	{ str[i] = '0' ;
		i++ ;
		str[i]= '\0' ;
	}
	if(m<0)
	{ m=-m ;
		flag=1 ;
	}
	while(m!=0)
	{
		r=m%10 ;
		str[i++] = r+ '0' ;
		m=m/10 ;
		 
	}
	if(flag==1)
	str[i++] = '-' ;
	str[i++] = '\0' ;
	p=strlen(str) ; 
	for(int i=p-1 ; i>=0 ; i--)
	lcd_data(str[i]) ;
}
void lcd(double d)
{
	int i = int(d) ; 
	float f = d-i ; 
	lcd(i) ; 																									 // - 0 is not a thing 
  lcd_data('.') ;
  if(d>0)	
  lcd((int)(f*100000)) ; 
  else if(d<0) 
 	lcd((int)(-f*100000)) ;	
}
void init_lcd()
{
	RCC->APB2ENR |= (1<<4) | RCC_APB2ENR_IOPBEN ;							// IOPCEN | IOPDEN clock 
	
  
	GPIOC -> CRL &= ~(1<<26) & ~(1<<30)  ; 
	GPIOC -> CRL |= (1<<24) | (1<<28)  ;
    
  //GPIOC -> CRH = 0 ; 	
	GPIOC -> CRH &= ~(1<<2) & ~(1<<6) ;  
	GPIOC -> CRH |= (1<<0) | (1<<4) ; 
	
  //GPIOB -> CRH = 0 ; 
	GPIOB -> CRH &= ~(1<<26) & ~(1<<22) ;  ; 
  GPIOB -> CRH |= (1<<20) | (1<<24)  ;    
  
  cmd(0x02) ; 
	cmd(0x28) ; 
	cmd(0x0E) ; 
	cmd(0x06) ; 
	cmd(0x01) ; 
	cmd(0x80) ;
	
  delayms(10) ;  

}
