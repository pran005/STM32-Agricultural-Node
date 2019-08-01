#include <stm32f10x.h> 
#include "lcd.h" 
#include "delay.h" 
#include <stdio.h> 
#include <string.h>
#include <stdbool.h> 
#include <stdlib.h>
#include "lib.h" 
/*************************ALWAYS USE PULLUPS WITH HC05 FOR MAX. SUCCESS***************************************/  

/*****************************UART FUNCTION DEFINITIONS*****************************************************/
void init_UART()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN ; 
	
	GPIOA -> CRH |= (1<<4) | (1<<7)  ;
	
  USART1 -> BRR = 0x9C4 ; 	
	USART1 -> CR1 |= (1<<13) | (1<<3) | (1<<2) | (1<<5) ;

}	

void UART_tx(char data) 
{
  while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = data; 
}

void UART_tx(int data) 
{
  while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = data; 
}

void UART_tx(uint8_t data) 
{
  while(!(USART1->SR & USART_SR_TXE));
  USART1->DR = data; 
}

void UART_tx(char *string) 
{
	while(*string) 
		UART_tx(*string++) ; 
}

unsigned char UART_rx()
{
	while((USART1->SR & (1<<5))==0) ;
		return USART1->DR ; 
}

void UART_str(char* string)
{
	unsigned char i=0,j=0 ; 
	do
	{
		*(string+i) = UART_rx() ; 
		j = *(string+i) ; 
		i++ ; 
	
	}
	while((j != '\r') && (j!='\n')) ;
		i++ ; 
	  *(string+i) = '\0' ; 
}

/*********************************END OF UART DEFINITIONS*******************************/


/*********************************ESP8266 Interface*************************************/

#define MAX_BUFF_LEN 30
#define MAX_WAIT_TIME 500UL

/******WIFI MODES******/
#define Station 1 
#define SoftAP  2 
#define Both    3 

/***Connection Modes***/ 
#define Single 0 
#define Multi  1

/*****ESP MODE*****/
#define Normal 0 
#define UART_passthru 1

/*****THINGSPEAK*****/
#define DOMAIN "api.thingspeak.com"
#define PORT   "80"
#define WRITE_KEY "FU5I020P64SON640"
#define Channel_ID  "524595"
/********************/

enum responses{

	started,
	waiting,
	finished,
	timed_out,
	Error

} ; 

enum Wifi_status{
	
	WIFI_CONNECTED,
	CONNECTION_TIMEOUT,
	WRONG_PASSWORD,
	NOT_FOUND,
	CONNECTION_FAILED,
	UNKNOWN_ERROR	
	
	};
	
enum TCP_STATUS {
	
	CONNECTED_TO_AP,
	TRANSMISSION_ENGAGED,
	TRANSMISSION_DISCONNECTED,
	NOT_CONNECTED_TO_AP,
	TCP_UNKNOWN_ERROR
	
};	

/***************************GLOBAL VARIABLES********************************/ 

volatile int16_t buff_ptr = 0 , ptr_loc = 0 , bt_ptr=0 ; 
char Rx_buff[MAX_BUFF_LEN] ; 
char BT_buff[MAX_BUFF_LEN] ; 
uint32_t time=0 ; 
int8_t status ; 

/*************************ESP8266 COMMAND ROUTINES**********************/

void check_response(char* act_response)
{
  int act_response_len = strlen(act_response) ; 
	uint16_t rx_res_len , countms=0 ; 
	char Rx_response[20] ; 
	while(1)
	{
		clr_lcd();
		if(countms >= MAX_WAIT_TIME) 
		{
			countms = 0 ; 
			status= timed_out ; //waiting 
			return ; 
		}
		if(status==started)
		{   
			 
			status=waiting ;
			 
		}					
		rx_res_len = strlen(Rx_buff) ;
		//lcd(rx_res_len) ;  
		if(rx_res_len)
		{
			delayms(1) ; 
			countms++ ; 

			if(rx_res_len==strlen(Rx_buff))
			{
				for(uint16_t i=0; i<rx_res_len; i++)
				{
				     
					memmove(Rx_response,Rx_response+ 1,act_response_len-1) ; 
					Rx_response[act_response_len-1]= Rx_buff[i] ;  
					
					if(!(strncmp(Rx_response,act_response,act_response_len)))
					{
						 
						countms = 0 ; 
						status = finished ; 
						return ; 
					}
				}
			}
		}
		delayms(1);
		countms++ ;
	}	
} 
/*void check_response(char* expected_rx)
{
	uint8_t rx_buff_len ; 
	char rx_resp[100]  ;
	uint8_t exp_len = strlen(expected_rx) ; 
	uint32_t countms = 0 ; 
	
while(1)
{
	if(countms >= MAX_WAIT_TIME)
	{
			status = timed_out ;
			time=0 ; 
			return ; 
	}
	if(status == started) 
	{
			status = waiting ; 
	}
	rx_buff_len = strlen(Rx_buff) ; 
	if(rx_buff_len) 
	{
		delayms(1) ; 
		++countms ; 
		
		if(rx_buff_len==strlen(Rx_buff)) 
		{
			*if(strstr(Rx_buff,expected_rx)) 
			{
				status = finished ; 
				//clr_lcd() ; 
				//lcd(Rx_buff) ;
				//delayms(800) ; 
				countms = 0  ; 
				return  ; 
			}
					*****VARIANT 2 ******
			for(int i = 0 ; i<exp_len ; i++) 
			{
				memmove(rx_resp,rx_resp+1,exp_len-1) ; 
				rx_resp[exp_len-1] = Rx_buff[i] ; 
				if(!strncmp(rx_resp,expected_rx,exp_len))
				{
					countms = 0 ; 
					status = finished ; 
					return ; 
				} 
			}
		}
	
	delayms(1) ; 
	++countms ; 
}}
}*/

void clear_buff()
{
	memset(Rx_buff,0,MAX_BUFF_LEN) ; 
}

bool get_response(char* expected_rx)
{
	status = started ; 
	do
	{
		check_response(expected_rx) ; 
	}
	while(status==waiting) ; 

	if(status!=timed_out)
		return 1 ; 
	else 
		return 0 ; 
}

bool AT_Communicate(char* AT_cmd, char* expected_rx)
{
	clear_buff() ; 
	UART_tx(AT_cmd) ;  
	UART_tx((char*)"\r\n") ;
 	return get_response(expected_rx) ; 
}	

bool wake_ESP() 
{
	clear_buff() ; 
	for(uint8_t i=0;i<5;i++) 
	{
		if(AT_Communicate((char*)"AT",(char*)"\r\nOK\r\n"))  
				return 1 ; 
	}
	return 0 ; 
}

bool ESP_Mode(uint8_t Mode)
{
	char AT[50] ; 
	memset(AT,0,50) ; 
	sprintf(AT,"AT+CIPMODE=%d",Mode) ; 
	AT[49] = 0 ; 
	return AT_Communicate(AT,"\r\nOK\r\n") ; 
}

bool Wifi_mode(uint8_t Mode)
{
	char AT[50] ;
	memset(AT,0,50) ;
	sprintf(AT,"AT+CWMODE=%d",Mode) ;
	AT[49] = 0 ;
	return AT_Communicate(AT,"\r\nOK\r\n") ;
}

bool TCP_mode(uint8_t Mode)
{
	char AT[50] ;
	memset(AT,0,50) ;
	sprintf(AT,"AT+CIPMUX=%d",Mode) ;
	AT[49] = 0 ;
	return AT_Communicate(AT,"\r\nOK\r\n") ;
}

uint8_t JoinAP(char*SSID,char* passkey)
{
	char AT[50] ;
	memset(AT,0,50) ;
	sprintf(AT, "AT+CWJAP_CUR=\"%s\",\"%s\"",SSID,passkey) ;
	AT[49] = 0 ;
    if(AT_Communicate(AT,"\r\nWIFI CONNECTED\r\n")) 
		return	WIFI_CONNECTED ; 
	else if(strstr(Rx_buff,"+CWJAP:1"))
		return CONNECTION_TIMEOUT;	
	else if(strstr(Rx_buff,"+CWJAP:2"))
		return WRONG_PASSWORD ;
	else if(strstr(Rx_buff,"+CWJAP:3"))
		return NOT_FOUND ; 
	else if(strstr(Rx_buff, "+CWJAP:4"))
		return CONNECTION_FAILED;
	else
		return UNKNOWN_ERROR;			 
}

uint8_t TCP_stat()
{
	AT_Communicate("AT+CIPSTATUS", "\r\nOK\r\n");
	if(strstr(Rx_buff, "STATUS:2"))
	return CONNECTED_TO_AP;
	else if(strstr(Rx_buff, "STATUS:3"))
	return TRANSMISSION_ENGAGED;
	else if(strstr(Rx_buff, "STATUS:4"))
	return TRANSMISSION_DISCONNECTED;
	else if(strstr(Rx_buff, "STATUS:5"))
	return NOT_CONNECTED_TO_AP;
	else
	return TCP_UNKNOWN_ERROR;
}

int8_t TCP_start(uint8_t Connect_number,char* domain, char* port)
{
	bool response ; 
	char AT[70] ; 
	memset(AT,0,70) ; 
	AT[69]= 0 ; 
	if(AT_Communicate("AT+CIPMUX?", "CIPMUX:0"))														//check connection mode
		sprintf(AT,"AT+CIPSTART=\"TCP\",\"%s\",%s", domain, port); 
	else
		sprintf(AT,"AT+CIPSTART=\"%d\",\"TCP\",\"%s\",%s",Connect_number, domain, port);
	response = AT_Communicate(AT,"CONNECT\r\n") ; 
	if(!response)
	{	
		if(status==timed_out)
			return timed_out ; 
		return Error ; 	
	}
	return finished ;
}

uint8_t send_data(char* data)
{
	char AT[50] ; 
	memset(AT,0,50) ; 
	sprintf(AT,"AT+CIPSEND=%d",(strlen(data)+2));
	AT[49] = 0; 
	AT_Communicate(AT,"\r\nOK\r\n>") ; 
	if(!AT_Communicate(data,"\r\nSEND OK\r\n"))
	{
		if (status==timed_out)
			return timed_out ; 
		else 
			return Error ; 	
	}
	return finished ; 
}

/*************************************END OF ESP8266 ROUTINES*********************************************/

/*****************************UART 2 FUNCTION DEFINITIONS*****************************************************/
void init_UART2()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN ; 
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN ;
	
	GPIOA -> CRH |= (1<<4) | (1<<7)  ;
	GPIOA -> CRL |= (1<<8) | (1<<11) ; 
	
  USART2 -> BRR = 0x9C4 ; 	
	USART2 -> CR1 |= (1<<13) | (1<<3) | (1<<2) | (1<<5) ;

}	
void UART2_tx(char data) 
{
  while(!(USART2->SR & USART_SR_TXE));
  USART2->DR = data; 
}

void UART2_tx(int data) 
{
  while(!(USART2->SR & USART_SR_TXE));
  USART2->DR = data; 
}

void UART2_tx(uint8_t data) 
{
  while(!(USART2->SR & USART_SR_TXE));
  USART2->DR = data; 
}

void UART2_tx(char *string) 
{
	while(*string) 
		UART2_tx(*string++) ; 
}

unsigned char UART2_rx()
{
	while((USART2->SR & (1<<5))==0);
		return USART2->DR ; 
}

void UART2_str(char* string)
{
	unsigned char i=0,j=0 ; 
	do
	{
		*(string+i) = UART2_rx() ; 
		j = *(string+i) ; 
		i++ ;
		
	}
	while((j != '\r') && (j!='\n')) ;
		i++ ;
	  *(string+i) = '\0' ;
	  
		//lcd(string) ; 
}

/*********************************END OF UART 2 DEFINITIONS*******************************/


/*************************************UART INTERRUPT HANDLERS*********************************************/
extern "C" 
{
void USART1_IRQHandler()
{
	 
	//USART1->SR &= ~(1<<5) ;
	//DISABLE global interrupts here  
  // 
	//NVIC_DisableIRQ(USART1_IRQn) ; 
	Rx_buff[buff_ptr++] = USART1->DR ;
   if(buff_ptr == MAX_BUFF_LEN) 
	 {
			buff_ptr = 0 ; 
	 }
	 NVIC_ClearPendingIRQ(USART1_IRQn) ;
 //NVIC_EnableIRQ(USART1_IRQn) ; 	 
}
} 

extern "C" 
{
void USART2_IRQHandler()
{
	 
  	clr_lcd() ; 
		char strp[20] ;
		uint8_t count = 0; 
	  memset(strp,0,20) ; 
		lcd("Command Mode") ; 
		cmd(0xc0);
	  UART2_str(strp) ;
    if(strstr(strp,"Relay3"))		
		{
			lcd("PUMP ON") ;
			GPIOB -> ODR ^= (1<<0) ;
		}
		else if(strstr(strp,"Relay4"))		
		{
			lcd("Siren ON") ; 
			GPIOB -> ODR ^= (1<<1) ;
			delayms(1000) ; 
			GPIOB -> ODR ^= (1<<1) ;
		} 
		else if(strstr(strp,"Relay2"))		
		{lcd("Lights ON") ; } 
		UART2_tx("RX!") ; 
		cmd(0x80) ; 
		delayms(1000) ; 
		NVIC_ClearPendingIRQ(USART2_IRQn) ; 
	//NVIC_EnableIRQ(USART2_IRQn) ;
		clr_lcd() ; 
}
}
void  EXTI0_IRQHandler()
{
	GPIOB -> BSRR |= (1<<16) ;
}

/**************************************END OF UART HANDLER********************************************/


/**************************************ADC CONVERSION FUNCTIONS*************************************/
float convert_temp()
/*{
	
	ADC1 -> CR2 |= (1<<0) | (1<<23)   ;
	//ADC1 -> SMPR1 &=  ~(1<<18) ; 
	ADC1 -> SQR3 |= (1<<0) ;
	ADC1 -> CR2 |= (1<<0) ;
	while(!(ADC1->SR & ADC_SR_EOC)) {} ;
  return ADC1 -> DR ;
} */

{
		uint16_t data ; 
		ADC1 -> CR2 |= (1<<0) | (1<<23)   ;
	//	ADC1 -> SMPR1 &=  ~(1<<18) ; 
		ADC1 -> SQR3 |= (1<<4) ;
		ADC1 -> CR2 |= (1<<0) ;
		while(!(ADC1->SR & ADC_SR_EOC)) {} ;
		data = ADC1 -> DR ; 
    float v25 = 1.42 ; 
		float vsense = data*(3.3/4096) ; 
		float temp = ((vsense - v25)/4.6) + 25 ;
		return temp ; 


} 

uint16_t convert_moist()
{
	ADC1 -> CR2 |= (1<<0) | (1<<23)   ; 
	ADC1 -> SMPR1 &= ~(1<<0) ;
	ADC1 -> SQR3 = 0x00000000 ;
	//ADC1 -> SQR3 = ~(1<<0) & ~(1<<4) ;
	ADC1 -> CR2 |= (1<<0) ;
	while(!(ADC1->SR & ADC_SR_EOC)) {} ;
  return ADC1 -> DR ;
}

/************************************END OF ADC CONVERSIONS*****************************************/
/*************************************MOTOR DEFINITIONS********************************************/
#define LED_BLUE_GPIO	GPIOB
#define LED_BLUE_PIN	0

#define LED_GREEN_GPIO	GPIOB
#define LED_GREEN_PIN	1 

/******************************************MAIN************************************************/
int main()
{
	/****************SYSTEM INITIALIZATIONS*************/
	
	SystemInit() ;
	RCC->APB2ENR |=   RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN ;
  //GPIOA -> CRL = 0x00000000; 
	GPIOA ->CRL &= ~(1<<2) ; 
	GPIOA ->CRL &= ~(1<<3) ;
  GPIOA ->CRL &= ~(1<<6) ;
  GPIOA ->CRL &= ~(1<<7) ;  	
	
	#if (LED_BLUE_PIN > 7)
  LED_BLUE_GPIO->CRH = (LED_BLUE_GPIO->CRH & CONFMASKH(LED_BLUE_PIN)) |
		GPIOPINCONFH(LED_BLUE_PIN, GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_AFIO_PUSHPULL));
	#else
  LED_BLUE_GPIO->CRL = (LED_BLUE_GPIO->CRL & CONFMASKL(LED_BLUE_PIN)) | 
    GPIOPINCONFL(LED_BLUE_PIN, GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
	#endif

	#if (LED_GREEN_PIN > 7)
  LED_GREEN_GPIO->CRH = (LED_GREEN_GPIO->CRH & CONFMASKH(LED_GREEN_PIN)) |     GPIOPINCONFH(LED_GREEN_PIN, GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_AFIO_PUSHPULL));
	#else
  LED_GREEN_GPIO->CRL = (LED_GREEN_GPIO->CRL & CONFMASKL(LED_GREEN_PIN)) |     GPIOPINCONFL(LED_GREEN_PIN, GPIOCONF(GPIO_MODE_OUTPUT2MHz, GPIO_CNF_OUTPUT_PUSHPULL));
	#endif
	
	GPIOB -> BSRR |= (1<<16) ;
	GPIOB -> ODR |= (1<<1) ;
	
	/****************PERIPHERAL INITIALIZATIONS*****************/
	 init_UART() ; 
		init_lcd() ;
	init_UART2() ; 

	char str1[60],str2[60]  ;
	uint8_t connect_stat ; 
	//uint16_t d=20  ; ;
	NVIC_SetPriority(USART2_IRQn,44) ;
	NVIC_SetPriority(USART1_IRQn,45) ;
	NVIC_EnableIRQ(USART1_IRQn) ;
  NVIC_EnableIRQ(USART2_IRQn) ;	
	
	
	/****************ESP8266 INITIALIZATIONS********************/
		clear_buff() ; 	
		//ms++ ;
		 
	while(!wake_ESP()) ;   
		lcd("ESP8266 AWAKE") ;
		clear_buff() ; 

		AT_Communicate("AT+CIPMUX=1","\r\nOK\r\n") ; 
    TCP_mode(0); 
		clr_lcd();
		lcd("TCP CLIENT MODE") ; 
		
		Wifi_mode(3) ;
		clr_lcd(); 
		lcd("WIFI MODE : SET") ; 
		
		ESP_Mode(0);
		lcd("Trying to Connect") ; 
		//if(TCP_stat()==NOT_CONNECTED_TO_AP)
	  JoinAP("Wifi1","00001111") ; 
	  clr_lcd();
	  lcd((char *)"Initializing");
		cmd(0xC0) ; 
		lcd("Data logging") ; 
		
 /*****************IT's been a WHILE*******************/
 while (1) 
    {
		 
			clr_lcd() ; 
			lcd((char*)"Connecting") ;
			connect_stat=TCP_stat() ; 
			
			/*********CONNECT TO SERVER************/
			
			if(connect_stat == NOT_CONNECTED_TO_AP)
			JoinAP((char*)"Wifi1",(char*)"00001111") ;
			int n = 0 ;
			while(TCP_start(0,(char*)DOMAIN,(char *)PORT) != finished) 
			{
				TCP_start(0,(char *)DOMAIN,(char *)PORT);
				n++ ; 
				lcd(".") ; 
				if(n==5) 
				break ; 
			}
			
			clr_lcd() ; 
			lcd("SENDING DATA..") ; 
		
		
		
			/************TEMPERATURE*************/
			float	temp = convert_temp() ; 
		//	float temp = ((float)temp_data*330)/((float)4096)	; 
			//lcd(temp) ;
			//lcd("-C") ; 
			//lcd_newline() ;
		 
			
			/***********SOIL MOISTURE***********/
			uint16_t moist_data = convert_moist() ;
			float moist =  ((4095 - (float)moist_data)*100)/4096 ;
			//lcd(moist) ;
			//lcd("%") ; 
			clr_lcd() ; 
			if(moist<30)
			{		lcd("LOW MOISTURE ") ;
			//while(moist<30)
					GPIOB -> BSRR |= (1<<0) ; 
					delayms(1000) ; 	
					uint16_t moist_data = convert_moist() ;
					moist =  ((4095 - (float)moist_data)*100)/4096 ;
					if(moist>30) 
						GPIOB -> BSRR = (1<<16) ;
			}
			else 
				GPIOB -> BSRR = (1<<16) ;
			lcd((int)moist) ; 
			lcd("%") ;
			lcd_newline() ;
			lcd(temp) ;
			lcd_data(0xDF); 
			lcd("C") ; 
			
			
			/**********STRING CAST***********/
			memset(str1,0,50) ;
			memset(str2,0,50) ;		
			sprintf(str1, "GET /update?api_key=%s&field2=%f", WRITE_KEY, (float)temp);
			sprintf(str2, "GET /update?api_key=%s&field1=%f", WRITE_KEY, moist);
			
		
		/*********SEND TEMPERATURE*******/
			send_data(str1) ; 
			delayms(15000) ;
			
		 n=0; 
			while(TCP_start(0,(char*)DOMAIN,(char *)PORT) != finished) 
			{
				TCP_start(0,(char *)DOMAIN,(char *)PORT);
				n++ ; 
				if(n==5) 
				break ; 
			}
			
			
			moist_data = convert_moist() ;
		  moist =  ((4095 - (float)moist_data)*100)/4096 ;
			clr_lcd() ; 
			if(moist<30)
			{		lcd("LOW MOISTURE ") ;
					GPIOB -> BSRR |= (1<<0) ; 
					delayms(1000) ; 	
					uint16_t moist_data = convert_moist() ;
					moist =  ((4095 - (float)moist_data)*100)/4096 ;
					if(moist>30) 
						GPIOB -> BSRR |= (1<<16) ;
			}
			else 
				GPIOB -> BSRR = (1<<16) ;
			lcd((int)moist) ; 
			lcd("%") ;
			lcd_newline() ;
			lcd(temp) ;
			lcd_data(0xDF); 
			lcd("C") ;
    	
		 /********SEND MOISTURE*********/	
			send_data(str2) ; 
			delayms(15000) ; 
	 
    }
  	}		


