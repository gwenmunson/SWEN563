//Gwen Munson
//UART_Demo main.c
//Create a command line UI that can turn LEDs on and off and have them flash. 

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>

char RxComByte = 0;
uint8_t buffer[BufferSize];
char currLow[40];
char menu[] = "Please enter in a value for the lower limit of the pulse test (50 to 9950 microseconds, anything outside this range will run default values):\r\n";
char outputModifier[] = "\r\n";//used to display output on its own line
char errorOutput[] = "Incorrect command, please try again All commands are in caps.\n\r";//error message used for UI friendliness
int lowLimit = 950;//in microseconds
int highLimit = 1050;//in microseconds

void init_pa0( void )
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN ;		// enable clock for A group of GPIO
	GPIOA->MODER &= ~3 ;										// clear out bits 0 and 1 for PA0
																					// PA0 is now in input mode
	//GPIOA->MODER |= 2 ;									// Enable alternate function mode (binary 10) for PA0
}

// Read the state of PA0 (high or low)
int read_pa0( void )
{
	return GPIOA->IDR & 1 ;				// read the bottom bit of the GPIOA input lines which is PA0.
																// returns 1 if high, 0 if low.
}

// This is an infinite loop that will show if PA0 is changing state
void monitor_pa0( void )
{
	//while ( 1 )
		if ( read_pa0() ){
			Red_LED_Off() ;
		  Green_LED_On();
		}
		else{
			Red_LED_On() ;
			Green_LED_Off();
		}
}

void set_timer2(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; 
	TIM2->PSC = 79;//79 + 1 is 80, and this prescaler makes it so we're running at 1 mHz
	
	TIM2->EGR |= TIM_EGR_UG; //generates update event for TIM2
	TIM2->CCR1 &= ~TIM_CCER_CC1E; //disable input/output to configure TIM2CH1
	TIM2->CCMR1 &= ~TIM_CCMR1_IC1F; //disables input filters for noise
	//configure CH1 for input
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; //sets bit 0 of the CC1S to 1
	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S_1; //sets bit 1 of the CC1S to 0
	TIM2->CCR1 |= TIM_CCER_CC1E; //enables input for CH1
}

void post_test(){
	TIM2->CR1 |= TIM_CR1_CEN; //input for CH1 ia now being captured
	
}


int main(void){
	char  rxByte;
	
	System_Clock_Init(); // Switch System Clock = 80 MHz
	LED_Init();
	UART2_Init();
	set_timer2();

	
	init_pa0();
	//POST Test
		
	while (1){
		char input[10] = "";
		sprintf(currLow, "Current lower limit: %i", lowLimit);
		USART_Write(USART2, (uint8_t *)currLow, strlen(currLow));
		USART_Write(USART2, (uint8_t *)outputModifier, strlen(outputModifier));
		USART_Write(USART2, (uint8_t *)menu, strlen(menu));
		int i = 0;
		rxByte = USART_Read(USART2);
		if(rxByte == 8 || rxByte == 127){
			input[i-1] = '\0';
			//strcpy(input, input - 1);
		}
		else if(rxByte != '\r'){
			while (rxByte != '\r'){
				input[i] = rxByte;
				i++;
				USART_Write(USART2, (uint8_t *)input, strlen(input));
				USART_Write(USART2, (uint8_t *)outputModifier, strlen(outputModifier));
				rxByte = USART_Read(USART2);
			}
		}
		input[i+1] = '\0';

		
		
		
	}
}

