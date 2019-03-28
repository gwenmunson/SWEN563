/**
  ******************************************************************************
  * @file           : bank_manager.c
  * @brief          : all things bank_manager-related
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifndef BANK_TELLER_H
#define BANK_TELLER_H

#include "string.h"
#include "usart.h"
#include "task.h"
#include "bank_manager.h"

#define NUM_TELLERS 3

/* Private variables ---------------------------------------------------------*/

typedef struct {
	TaskHandle_t handle;
	char name[10];
	int instance;
	// statistics to tally per teller
	int customers_served;
	uint32_t customer_arrival_time;
	uint32_t total_transaction_time;
	uint32_t max_transaction_time;
	uint32_t total_customer_wait_time;
	uint32_t max_customer_wait_time;
	uint32_t total_teller_wait_time;
	uint32_t max_teller_wait_time;
} TELLER_t;

extern TELLER_t tellers[NUM_TELLERS];
	
/* Private function prototypes -----------------------------------------------*/

void UART2_Init(void);
void UART2_GPIO_Init(void);
void teller_thread(void * argument);
void tellers_init(int num_tellers);

#endif // BANK_TELLER_H
