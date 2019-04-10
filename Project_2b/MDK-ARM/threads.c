//#include "stm32l476xx.h"
//#include "SysClock.h"
//#include "stm32l4xx_hal_uart.h"
//#include "timers.h"
//#include "gpio.h"
//#include "servo.h"
#include "recipe.h"
//#include "LED.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


#define OPCODE_MASK 0xE0
#define ARGS_MASK 0x1F
#define SERVO_DELAY 2


int timeout = 0;
uint8_t command;
int program_counter[] = {0,0};
int wait_count[] = {0,0};
int in_loop[] = {0,0};
int loop_counter[] = {0,0};
int loop_instruction[] = {0,0};
enum user_events user_command[] = {pause_recipe,pause_recipe};
int new_command[] = {0,0};
enum states servo_state[] = {recipe_paused, recipe_paused};
int position[] = {3,3};

SemaphoreHandle_t  servo_mutex;
servo systemServos[2];
struct commands servo_commands = {{noop,noop},{false,false}};
systemServos[0] = servo1;
systemServos[1] = servo2;

typedef struct recipes{
  unsigned char* recipe;
}recipes;

typedef struct servos{
	enum user_events servo_command;
	bool command_flag;
}servo;


unsigned char test_recipe[] = {MOV|0, MOV|5, MOV|0, MOV|3, LOOP|0, MOV|1, MOV|4, END_LOOP, MOV|0, MOV|2, WAIT|0, MOV|3, WAIT|0, MOV|2, MOV|3, WAIT|31, WAIT|31, WAIT|31, MOV|4, RECIPE_END, MOV|3};
unsigned char move_recipe[] = {MOV|0, MOV|1, MOV|3, MOV|2, MOV|5, MOV|4, RECIPE_END, MOV|1};
unsigned char cmd_err_recipe[] = {MOV|1, MOV|4, WAIT|12, MOV|2, LOOP|2, MOV|1, MOV|2, END_LOOP, MOV|6, MOV|4, RECIPE_END};
unsigned char loop_err_recipe[] = {MOV|2, MOV|5, MOV|1, WAIT|31, LOOP|3, MOV|2, MOV|5, LOOP|1, MOV|4, MOV|1, END_LOOP, END_LOOP, RECIPE_END};

struct recipes rec_list[2] = {{0},{0}};

int count_commands = 0;
int cancel_command = 0;

uint8_t prompt[] = "\r\n>>";

void CaptureCommands(void);
void ParseCommand(uint8_t read_command);
void GetCommandsTimer(void);
void process_event(enum user_events one_event, enum states current_state, int servo_num);

void parse_recipe (int op){
	switch(op){
			case MOV:
				if(args >= 0 && args <= 5){
					wait_count[i] = SERVO_DELAY * (abs(position[i] - args));
					position[i] = args;
					SetPosition(i, args);
				}
				else{
					servo_state[i] = recipe_cmd_err;
				}
				break;
			case WAIT:
				wait_count[i] = args+1;
				break;
			case LOOP:
				if(in_loop[i] == 1){
					servo_state[i] = recipe_nested_err;
				}
				else{
					loop_counter[i] = args;
					in_loop[i] = 1;
					loop_instruction[i] = program_counter[i];
				}
				break;
			case END_LOOP:
				if(loop_counter[i] > 0){
					loop_counter[i]--;
					program_counter[i] = loop_instruction[i];
				}
				else{
					in_loop[i] = 0;
				}
				break;
			case RECIPE_END:
				servo_state[i] = recipe_done;
				break;
			default:
				break;
		}
	}
}

bool IsNewCommand(int i){
	xSemaphoreTake(servo_mutex, 10000);
	bool is_updated = servo_commands.updated[i];
	xSemaphoreGive(servo_mutex);
}

void servo_thread(void* argument){
	int i = *(int *) argument;
	for(;;){
		if(IsNewCommand(i)){
			user_events command = servo_commands.event[i];
			process_event(command, servo_state[i], i);
		}
		if(servo_state[i] == recipe_running){
		   if(wait_count[i] > 0){
		   	wait_count[i]--;
		   	continue;
		   }
		   int recipe_command = rec_list[i].recipe[program_counter[i]];
		   int op = recipe_command & OPCODE_MASK;
		   int args = recipe_command & ARGS_MASK;
		   parse_recipe(op);
		   program_counter[i]++;
		}
		if(i == 0){
			StateLEDs(servo_state[0]);
		}
	}
}

int main(void){
	System_Clock_Init();
	LED_Init();
	Timer2_init();
	Timer5_init();
	GPIO_Init();
	UART2_Init();

	rec_list[0].recipe = loop_err_recipe;
	rec_list[1].recipe = test_recipe;


	USART_Write(USART2, prompt, 4);
	TIM5->CR1 |= TIM_CR1_CEN;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void CaptureCommands(void){
	if(command == 0){
		return;
	}
	else{
		USART_Write(USART2, &command, 1);
		ParseCommand(command);
	}
}

void ParseCommand(uint8_t read_command){
	if(read_command == 0x58 || read_command == 0x78){
		user_command[0] = noop;
		user_command[1] = noop;
		cancel_command = 1;
	}
	else if(count_commands < 2 && cancel_command == 0){
		switch(command){
			case 0x50:
			case 0x70:
				user_command[count_commands] = pause_recipe;
				break;
			case 0x43:
			case 0x63:
				user_command[count_commands] = continue_recipe;
				break;
			case 0x52:
			case 0x72:
				user_command[count_commands] = move_right;
				break;
			case 0x4c:
			case 0x6c:
				user_command[count_commands] = move_left;
				break;
			case 0x42:
			case 0x62:
				user_command[count_commands] = begin_recipe;
				break;
			default:
				user_command[count_commands] = noop;
				break;
		}
        count_commands++;
	}
	else if(command == 0x0D){
		if(count_commands >= 2){
			new_command[0] = 1;
			new_command[1] = 1;
		}
		count_commands = 0;
		cancel_command = 0;
		USART_Write(USART2, prompt, 4);
	}
}


void TIM2_IRQHandler(){
	if(TIM2->SR & TIM_SR_UIF){
		TIM2->SR &= ~(TIM_SR_UIF);
		timeout = 1;
	}
	if(systemServos[0].command_flag || systemServos[1].command_flag){

	}
}

void GetCommandsTimer(){
	while(!(USART2->ISR & USART_ISR_RXNE)){
		if(timeout == 1){
			command = 0;
			return;
		}
	}
	uint8_t val = USART2->RDR & 0xFF;;
	command = val;
}

void process_event(enum user_events one_event, enum states current_state, int servo_num){
	switch (current_state)
	{
		case recipe_paused:
        case recipe_done:
			if (one_event == move_left && position[servo_num] < 5) // prevent moving too far left
			{
				position[servo_num]++;
				wait_count[servo_num] += SERVO_DELAY;
				SetPosition(servo_num, position[servo_num]) ;
			}
			else if(one_event == move_right && position[servo_num] > 0){
				position[servo_num]--;
				wait_count[servo_num] += SERVO_DELAY;
				SetPosition(servo_num, position[servo_num]);
			}
			else if(one_event == begin_recipe){
                program_counter[servo_num] = 0;
                wait_count[servo_num] = 0;
                in_loop[servo_num] = 0;
                loop_instruction[servo_num] = 0;
                servo_state[servo_num] = recipe_running;
			}
			else if(one_event == continue_recipe){
				servo_state[servo_num] = recipe_running;
			}
			break ;
		case recipe_running:
            if(one_event == pause_recipe){
				servo_state[servo_num] = recipe_paused;
			}
			break;
		default:
			break;
	}
}
