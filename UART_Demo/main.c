//Gwen Munson, Mason Jordan
//SWEN 563 Project 1: main.c
//Record the exact times for 1000 pulses and display all non-zero times. 

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_U32_DIGIT_SIZE 10
#define MAX_LOW_LIMIT 9950 //given in problem statment
#define DEFAULT_LOW_LIMIT 950 //given in problem statment
#define MIN_LOW_LIMIT 50 //given in problem statment
#define NUMBER_OF_PULSES 100 //used for array sizing
#define MAX_BUCKET_COUNT 5 

char currLow[40];//current low used for display purposes
char input[10];//used for grabbing user input
char bucketOutput[MAX_BUCKET_COUNT];
char bucketValueOutput[MAX_BUCKET_COUNT];
char menu[] = "Please enter in a value for the lower limit of the pulse test (50 to 950 microseconds, anything outside this range will run default values):\r\n";
char bucketPrintStart[] = "Program Results:\r\n";
char bucketPrintEnd[] = "End of results.\r\n";
char passedPost[] = "POST test passed!\r\n";
char failedPost[] = "POST test failed! Do you wish to run again? (N to exit, anything else to continue):\r\n";
char outputModifier[] = "\r\n";//used to display output on its own line
int recordedTimes[NUMBER_OF_PULSES];//these are the times we've recorded from the testing
int lowLimit = DEFAULT_LOW_LIMIT;//in microseconds
int highLimit = DEFAULT_LOW_LIMIT + 100;//in microseconds

//compare function used for C's qsort() function to sort the array of collected values
//I grabbed this from TutorialsPoint as this is a function that makes qsort sort in ascending order
int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

void printStringInt(char* text, int value){
	int bufSize=strlen(text)+MAX_U32_DIGIT_SIZE+1;
	char textBuffer[bufSize];
	sprintf(textBuffer,text, value);
	USART_Write(USART2, (uint8_t *)textBuffer, bufSize);
	memset(textBuffer,0,strlen(textBuffer));
}

void init_pa0( void )
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN ;		// enable clock for A group of GPIO
	GPIOA->MODER &= ~3 ;										// clear out bits 0 and 1 for PA0
																					// PA0 is now in input mode
	GPIOA->MODER |= 2 ;									    // Enable alternate function mode (binary 10) for PA0
	GPIOA->AFR[0] |= 0x1; 
}

//used for testing that we're receiving input, not needed in final implementation
//I kept it here so you can see the process taken to solve this
// Read the state of PA0 (high or low)
int read_pa0( void )
{
	return GPIOA->IDR & 1 ;				// read the bottom bit of the GPIOA input lines which is PA0.
																// returns 1 if high, 0 if low.
}

// This is an infinite loop that will show if PA0 is changing state
void monitor_pa0( void )
{
	while ( 1 )
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
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Timer 2 is now enabled
	TIM2->PSC = 79;//79 + 1 is 80, and this prescaler makes it so we're running at 1 mHz
	TIM2->EGR |= TIM_EGR_UG; //generates update event for Timer 2
	TIM2->CCR1 &= ~TIM_CCER_CC1E; //disable input/output to configure Timer 2 Channel 1
	TIM2->CCMR1 &= ~TIM_CCMR1_IC1F; //disables input filters for noise
	
	//configure Channel 1 for input, CC1 is configured for input & Timer Input 2 is used for
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; //sets bit 0 of the CC1S to 1
	TIM2->CCMR1 &= ~TIM_CCMR1_CC1S_1; //sets bit 1 of the CC1S to 0
	
	//We want the non-inverted, rising edge to be captured in input. Bits 3 and 1 need to be 0.
	TIM2->CCER &= ~(TIM_CCER_CC1P);//input polarity
	TIM2->CCER &= ~(TIM_CCER_CC1NP);//additional input polarity - used in conjunction with CC1P
	
	TIM2->CCER |= TIM_CCER_CC1E; //enables input for CH1
	TIM2->DIER |= TIM_DIER_CC1IE;
	
	//TIM2->CR1 |= TIM_CR1_CEN; //input for CH1 is now being captured
	//might need to enable interrupts
}

int post_test(){
	/*
		Commented code was used for debugging and understanding the status registers.
		Initially, POST would always fail even when input was being detected.
	*/
	
//	int testVal= TIM2->CNT & TIM_CNT_CNT;
//	short status = TIM2->SR;
	
	TIM2->CNT &= ~TIM_CNT_CNT;
	TIM2->CR1 |= TIM_CR1_CEN; //input for CH1 is now being captured
	
//	int val=TIM2->CCMR1;
//	int val2=TIM2->CCER;
//	printStringInt("CCMR1 Reg:%d\r\n",val);
//	printStringInt("CCER Reg:%d\r\n",val2);
//	printStringInt("Current Counter Val:%d\r\n",testVal);
//	printStringInt("Current Status Val:%x\r\n",status);
	while(1){
//		testVal= TIM2->CNT & TIM_CNT_CNT;
//		status = TIM2->SR;
		
		if((TIM2->SR & TIM_SR_CC1IF)){
			return 1;
		}
		else if(TIM2->CNT >= 100000){
			TIM2->CR1 &= ~TIM_CR1_CEN; //input for CH1 is now disabled
//			printStringInt("Current Counter Val:%d\r\n",testVal);
//	    printStringInt("Current Status Val:%x\r\n",status);
			return 0;
		}
	}
}

int capture_input(){
	/*
	PSEUDOCODE:
		Enable timer before busy wait
		while	CC1IF is 0, busy wait
		once while loop is exited, stop timer
		elapsed time = end time - start time
	  return time
	CC1IF - used to mark when input is received from the oscilliscope
	CCR1 contains the time (pg 915)
	*/
	while((TIM2->SR & TIM_SR_CC1IF) != 1) {}//busy wait until input is detected
	//input detected!!!! let's grab it 
	int pulseTime = TIM2->CCR1;
	if( !(pulseTime > highLimit || pulseTime < lowLimit) ){
		return pulseTime;//it's within range and should be counted!!!!
	}
	return 0;//this input is out of range!!!!
}

int main(void){
	char  rxByte;
	
	
	System_Clock_Init(); // Switch System Clock = 80 MHz
	LED_Init();
	UART2_Init();
	set_timer2();
	init_pa0();
	//monitor_pa0();
	
	//POST Test goes here
	int passPost = post_test();
	if(!passPost){
		while(!passPost){
			USART_Write(USART2, (uint8_t *)failedPost, strlen(failedPost));
			char response = USART_Read(USART2);
			if(response == 'N' || response == 'n'){
				break;//we break the while loop
			}//end if statement
			passPost = post_test();
		}//end while loop
	}//end if statement
	if(passPost){
		USART_Write(USART2, (uint8_t *)passedPost, strlen(passedPost));
	}
		
	while (1){
		//UI code
		sprintf(currLow, "Current lower limit: %i", lowLimit);
		USART_Write(USART2, (uint8_t *)currLow, strlen(currLow));
		USART_Write(USART2, (uint8_t *)outputModifier, strlen(outputModifier));
		USART_Write(USART2, (uint8_t *)menu, strlen(menu));
		int i = 0;
		rxByte = USART_Read(USART2);
		//backspace functionality
		if(rxByte == 8 || rxByte == 127){
			input[i-1] = '\0';
		}
		else if(rxByte != '\r'){
			while (rxByte != '\r'){
				input[i] = rxByte;
				i++;
				USART_Write(USART2, (uint8_t *)input, strlen(input));
				USART_Write(USART2, (uint8_t *)outputModifier, strlen(outputModifier));
				rxByte = USART_Read(USART2);
			}//end while loop
		}//end else if statement
		input[i+1] = '\0';
		int result = atoi(input);
		//Is the input within range? Change the low limit to this value if it is; otherwise, stick with default
		if( result > MAX_LOW_LIMIT || result < MIN_LOW_LIMIT ){
			lowLimit = DEFAULT_LOW_LIMIT;
			highLimit = DEFAULT_LOW_LIMIT + 100;
		}
		else{
			lowLimit = result;
			highLimit = lowLimit + 100;
		}

		TIM2->CR1 |= TIM_CR1_CEN; //input for CH1 is now being captured
		//Run actual program
		int totalTimes = 0;
		while (totalTimes != NUMBER_OF_PULSES){
			int currentTime = capture_input();
			if( currentTime > 0){
				recordedTimes[totalTimes] = currentTime;
			}//end if statement
			totalTimes++;
		}//end while loop
		
		//Display results
		//let's sort the recorded times to make actually displaying this easy
		qsort(recordedTimes, NUMBER_OF_PULSES, sizeof(int), cmpfunc );
		int currBucket = recordedTimes[0];//we're gonna start the first bucket with the first element in the array
		int bucketSize = 0;//number of values within the bucket
		USART_Write(USART2, (uint8_t *)bucketPrintStart, sizeof(bucketPrintStart));
		for(int i = 0; i < NUMBER_OF_PULSES; i++){
			if(currBucket == recordedTimes[i]){bucketSize++;}
			else{
				if(currBucket != 0){//zeroes may be in the array - this means the value was out of range.
					sprintf(bucketOutput, "%i\t", currBucket);
					sprintf(bucketValueOutput, "%i\t", bucketSize);
					USART_Write(USART2, (uint8_t *)bucketOutput, sizeof(bucketOutput));
					USART_Write(USART2, (uint8_t *)bucketValueOutput, sizeof(bucketValueOutput));
					USART_Write(USART2, (uint8_t *)outputModifier, strlen(outputModifier));
				}
				currBucket = recordedTimes[i];
				bucketSize = 0;
			}//end else statement
		}//end for loop
		USART_Write(USART2, (uint8_t *)bucketPrintEnd, sizeof(bucketPrintEnd));
				
		//Clear input string
		memset(input,0,strlen(input));
		TIM2->CR1 &= ~TIM_CR1_CEN; //input for CH1 is now disabled
		//add input disable here
	}
}
