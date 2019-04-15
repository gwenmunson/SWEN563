/*
#include "threads_2.h"


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

// Struct for holding recipes - used because 2d arrays wouldn't work
typedef struct recipes{
    unsigned char* recipe;
}recipes;

SemaphoreHandle_t  servo_mutex;
struct commands servo_commands = {{noop,noop},{false,false}};

unsigned char test_recipe[] = {MOV|0, MOV|5, MOV|0, MOV|3, LOOP|0, MOV|1, MOV|4, END_LOOP, MOV|0, MOV|2, WAIT|0, MOV|3, WAIT|0, MOV|2, MOV|3, WAIT|31, WAIT|31, WAIT|31, MOV|4, RECIPE_END, MOV|3};
unsigned char move_recipe[] = {MOV|0, MOV|1, MOV|3, MOV|2, MOV|5, MOV|4, RECIPE_END, MOV|1};
unsigned char cmd_err_recipe[] = {MOV|1, MOV|4, WAIT|12, MOV|2, LOOP|2, MOV|1, MOV|2, END_LOOP, MOV|6, MOV|4, RECIPE_END};
unsigned char loop_err_recipe[] = {MOV|2, MOV|5, MOV|1, WAIT|31, LOOP|3, MOV|2, MOV|5, LOOP|1, MOV|4, MOV|1, END_LOOP, END_LOOP, RECIPE_END};

// list of recipes
struct recipes rec_list[2] = {{0},{0}};	
	
//int count_commands = 0;
//bool cancel_command = false;

uint8_t prompt[] = "\r\n>>";


UART_HandleTypeDef huart2_thread;
uint8_t rx_buffer[20];  // Shared buffer between foreground and UART RX
uint8_t rx_byte;        // the currently received byte
uint8_t rx_index = 0;   // pointer into the rx_buffer
SemaphoreHandle_t  transmit_mutex;  // protects UART transmitter resource
SemaphoreHandle_t  receive_mutex;   // protects UART receiveer resource

void UART_Init(void){//UART_HandleTypeDef *huart2){
	//huart2_thread = *huart2;
	transmit_mutex = xSemaphoreCreateMutex();     // create mutex to protect UART transmitter resource
	receive_mutex = xSemaphoreCreateMutex();      // create mutex to protect UART receiver resource
	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);    // one time, kick off receive interrupt (repeated from within rx callback)
}	


void vPrintString(char *message) {
  xSemaphoreTake(transmit_mutex, ~0);         // Wait forever until the USART is free, then take mutex
  HAL_UART_Transmit_IT(&huart2_thread, (uint8_t *)message, strlen(message));
  xSemaphoreGive(transmit_mutex);             // Give up mutex after printing
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xTaskWoken = pdFALSE;
  static BaseType_t i_have_receive_mutex = pdFALSE;
  if(huart->Instance == USART2) {

    // if received byte is a newline, give the received buffer to the forground
    if(rx_byte == '\r') {
      xSemaphoreGiveFromISR(receive_mutex, &xTaskWoken);
      i_have_receive_mutex = pdFALSE;     // We don't have the mutex anymore
      rx_index = 0;                       // Next time around, queue data from start of buffer
    }

    // buffer all characters
    else {
      // acquire receive_mutex once
      if(!i_have_receive_mutex) {
        xSemaphoreTakeFromISR(receive_mutex,  &xTaskWoken);
        i_have_receive_mutex = pdTRUE;    // don't need to ask to Take again
      }

      // buffer all other characters
      rx_buffer[rx_index++] = rx_byte;    // buffer the byte
      rx_buffer[rx_index] = 0;            // keep string NULL terminated
      if(rx_index >= sizeof(rx_buffer))
        rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart2_thread, &rx_byte, 1);  // one time, kick off receive interrupt (repeated from within callback)
  }
}

void thread_init(void){
	int id[2];
	servo_mutex = xSemaphoreCreateMutex();
	
	char init_print[128];
	sprintf(init_print, "Initializing Threads\r\n");
	vPrintString(init_print);
	
	xTaskCreate(input_thread, "input_thread", 128, NULL, osPriorityNormal, 0);

	for(int i = 0; i<2; i++){
		id[i] = i;
		xTaskCreate(servo_thread, "servo_thread", 128, (void *)&id[i], osPriorityNormal, 0);
	}
}

void parse_recipe (int op, int args, int i){
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


bool IsNewCommand(int i){
	xSemaphoreTake(servo_mutex, 10000);
	bool is_updated = servo_commands.updated[i];
	xSemaphoreGive(servo_mutex);
	return is_updated;
}

void servo_thread(void* argument){
	int i = *(int *) argument;
	for(;;){
		if(IsNewCommand(i)){
			enum user_events command = servo_commands.event[i];
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
		   parse_recipe(op, args, i);
		   program_counter[i]++;
		}
		if(i == 0){
			StateLEDs(servo_state[0]);
		}
	}
}

//void CaptureCommands(void){
//	if(command == 0){
//		return;
//	}
//	else{
//		USART_Write(USART2, &command, 1);
//		ParseCommand(command);
//	}
//}

void ParseCommand(){
	for(int i = 0; i < 2; i++){
		
		if(rx_buffer[i] == 0x58 || rx_buffer[i] == 0x78){
			servo_commands.event[0] = noop;
			servo_commands.event[1] = noop;
			break;
		}

		switch(rx_buffer[i]){
			case 0x50:
			case 0x70:
				servo_commands.event[i] = pause_recipe;
				servo_commands.updated[i] = true;
				break;
			case 0x43:
			case 0x63:
				servo_commands.event[i] = continue_recipe;
				servo_commands.updated[i] = true;
				break;
			case 0x52:
			case 0x72:
				servo_commands.event[i] = move_right;
				servo_commands.updated[i] = true;
				break;
			case 0x4c:
			case 0x6c:
				servo_commands.event[i] = move_left;
				servo_commands.updated[i] = true;
				break;
			case 0x42:
			case 0x62:
				servo_commands.event[i] = begin_recipe;
				servo_commands.updated[i] = true;
				break;
			default:
				servo_commands.event[i] = noop;
				break;
		}//end switch statement

	}//end for loop
}


//void GetCommandsTimer(){
//	while(!(USART2->ISR & USART_ISR_RXNE)){
//		if(timeout == 1){
//			command = 0;
//			return;
//		}
//	}
//	uint8_t val = USART2->RDR & 0xFF;;
//	command = val;
//}


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

void input_thread(void* argument){
	char *display_msg = "Input thread initialized\n\r";
	char *command_msg = "Enter in a one-letter command for each servo:\n\r";
	char *error_msg = "Invalid entry - last character entered was not a carriage return. Please try again.";
	
	//The system is turning on, send output to the user to notify them	
	vPrintString(display_msg);
	vPrintString(command_msg);
	
	for(;;){
		//let's grab the input from the user
		HAL_UART_Receive_IT(&huart2_thread, &rx_byte, 1);
		//if we got something from the user, it'll be a non-zero value
		if(rx_byte != 0){
			//backspace/delete key detected
			if( rx_index > 0 && (rx_byte == 8 || rx_byte == 127) ){
				rx_buffer[rx_index--] = '\0';
			}
			else if( rx_byte == '\r' ){
				xSemaphoreTake(servo_mutex, 10000);
				ParseCommand();
				xSemaphoreGive(servo_mutex);
				//reset values
				memset(rx_buffer, 0, sizeof(rx_buffer));
				rx_index = 0;
				vPrintString(command_msg);
			}
		}//end if statement
		//error handling - if we got here, it means the last key entered wasn't a carriage return
		else if(rx_index == 2){
			vPrintString(error_msg);
			//reset values
			memset(rx_buffer, 0, sizeof(rx_buffer));
			rx_index = 0;
		}//end else if statement	
	}//end for loop
		
}
*/
