/**
  ******************************************************************************
  * @file           : bank_manager.c
  * @brief          : all things bank_manager-related
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#ifndef BANK_MANAGER_H
#define BANK_MANAGER_H

#include "freertos.h"
#include "task.h"
#include "bank_teller.h"

#define SCALE_FACTOR (100.0/60)	// real msec / bank time second
#define MIN_ARRIVAL_TIME (int)(60 * SCALE_FACTOR)
#define MAX_ARRIVAL_TIME (int)(240 * SCALE_FACTOR)
#define	BANK_OPEN_MSEC (int)(25200 * SCALE_FACTOR)	// how many msec is the bank open?
#define MIN_TRANS_TIME (int)(30 * SCALE_FACTOR)
#define MAX_TRANS_TIME (int)(480 * SCALE_FACTOR)

/* Private variables ---------------------------------------------------------*/
extern unsigned tellers_running;
extern QueueHandle_t customer_queue;
extern SemaphoreHandle_t tellers_running_mutex;

/* Private function prototypes -----------------------------------------------*/

void UART2_Init(void);
void UART2_GPIO_Init(void);
void bank_manager_thread(void * argument);
void bank_manager_init(void);


#endif // BANK_MANAGER_H
