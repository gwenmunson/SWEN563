#ifndef __STM32L476G_DISCOVERY_THREAD_H
#define __STM32L476G_DISCOVERY_THREAD_H

//#include "stm32l476xx.h"
#include "cmsis_os.h"
#include "servo.h"
#include "recipe.h"
#include "LED.h"
#include "UART_1.h"
#include "string.h"
//#include "semphr.h"
//#include "task.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "timers.h"

#define OPCODE_MASK 0xE0
#define ARGS_MASK 0x1F
#define SERVO_DELAY 2

//Prototypes
void thread_init(void);
void parse_recipe (int op, int args, int i);
bool IsNewCommand(int i);
void servo_thread(void* argument);
void ParseCommand(char rx_byte[2]);
void servo_thread(void* argument);
void process_event(enum user_events one_event, enum states current_state, int servo_num);
void input_thread(void* argument);

#endif
