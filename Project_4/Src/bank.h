#ifndef __BANK_MAN_H
#define __BANK_MAN_H
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os.h"
#include "uart.h"
#include "rng.h"

#define NUM_TELLERS 3

struct customer{
	int id;
	int time_entered_queue;
	int time_left_queue;
	int time_left_teller;
};

enum status{
	IDLE,
	BUSY,
	BREAK
};

struct teller{
	status teller_status;
	int num_customers;
	int num_breaks;
	int total_wait_time;
	int total_transaction_time;
	int total_break_time;
};


struct total_metrics{
	//times in system ms
	int customers_served;
	int customers_served_per_teller[3];
	float avg_customer_waiting_time;
	float avg_teller_time;
	float avg_teller_waiting_time;
	int max_customer_wait_time;
	int max_teller_wait_time;
	int max_transaction_time;
	int max_queue_depth;
	int total_num_breaks;
	float avg_break_time;
	int max_break_time;
	int min_break_time;
};

struct bank{
	QueueHandle_t customers;

	TaskHandle_t customer_entering_queue;
	TaskHandle_t teller_thread;

	teller tellers[3];
};

// Function Prototypes


#endif
