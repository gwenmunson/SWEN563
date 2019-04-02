#ifndef __BANK_MAN_H
#define __BANK_MAN_H
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os.h"
//#include "uart.h"
#include "rng.h"

#define NUM_TELLERS 3

/*
 * Customer struct that's used to keep track of customers within threads.
 *
 * int id: unique ID for customer, used for keeping track of current customer + metrics for total customers
 * int time_entered_queue: tick count of when customer entered queue, used for calculating metrics
 * int time_left_queue: tick count of when customer left queue, used for calculating metrics
 * int time_left_teller: tick count of when teller is finished with customer, used for calculating metrics
 *     and is different from the time the customer leaves the queue. Inside customer struct for extenability. 
 * int transaction_time: the randomly generated time between 30 seconds and 8 minutes (system time) used for 
			 servicing a customer.
 */
struct customer{
	int id;
	int time_entered_queue;
	int time_left_queue;
	int time_left_teller;
	int transaction_time;
};

/*
 * Status enum used for the status of the tellers.
 *
 * enum IDLE: status used for when a teller is not currently working with a customer or on break,
 *						waiting to take a customer.
 * enum BUSY: status used for when a teller is currently working with a customer.
 * enum BREAK: status used for when a teller is currently on a break from working and is unavailable to take customers.
 *             Breaks randomly occur every 30 - 60 minutes and take effect once the teller is out of BUSY status.
 *             Breaks last for 1 - 4 minutes and take 30 - 60 minutes for another break to be taken.
 */
enum status{
	IDLE,
	BUSY,
	BREAK
};

/*
 * Teller struct used to handle the processing of customer threads alongside metrics for the bank.
 *
 * enum status teller_status: a status enum used to signify what state the teller is currently in. Starts off as IDLE.
 * int num_customers: number of customers that this teller has served (only counts already served customers).
 * int num_breaks: number of breaks that this teller has taken (only counts completed breaks).
 * int total_wait_time: total amount of ticks that this teller has been in the IDLE status.
 * int total_transaction_time: total amount of ticks that this teller has been in the BUSY status.
 * int total_break_time: total amount of ticks that this teller has been in the BREAK status.
 */
struct teller{
	enum status teller_status;
	int num_customers;
	int num_breaks;
	int total_wait_time;
	int total_transaction_time;
	int total_break_time;
};

/*
 * Total metrics struct used for easily access of the total metrics of the bank system, formatted for printing.
 * All times are in system milliseconds (ms) instead of ticks.
 *
 * int customers_served: the total amount of customers served by all bank tellers.
 * int customers_served_per_teller: an array listing the total customers served by each respective teller.
 * float avg_customer_wait_time: the average amount of time that a customer spent in the queue awaiting service.
 * float avg_teller_time: the average amount of time needed for a teller to service a customer.
 * float avg_teller_waiting_time: the average amount of time a teller was in the IDLE status.
 * int max_customer_wait_time: the observed maximum time that a customer spent waiting in the queue.
 * int max_teller_wait_time: the observed maximum time that any teller was in the IDLE status.
 * int max_transaction_time: the observed maximum time that a teller needed to service a customer.
 * int max_queue_depth: the observed maximum size of the customer waiting queue.
 * int total_num_breaks: an array of the total amount of breaks for each respective teller.
 * float avg_break_time: the average break time between all tellers.
 * int max_break_time: the observed maxiumum break time for all tellers.
 * int min_break_time: the observed minimum break time for all tellers.
 */
struct total_metrics{
	int customers_served;
	int customers_served_per_teller[NUM_TELLERS];
	float avg_customer_waiting_time;
	float avg_teller_time;
	float avg_teller_waiting_time;
	int max_customer_wait_time;
	int max_teller_wait_time;
	int max_transaction_time;
	int max_queue_depth;
	int total_num_breaks[NUM_TELLERS];
	float avg_break_time[NUM_TELLERS];
	int max_break_time[NUM_TELLERS];
	int min_break_time[NUM_TELLERS];
	int total_break_time[NUM_TELLERS];
	int total_customer_queue_time;
	int total_customer_teller_time;
	int total_teller_wait_time;
};
/*
 * Bank struct used to represent the bank system as a whole, used within the main.c file.
 *
 * QueueHandle_t customers: a freeRTOS QueueHandle_t object used to represent the queue of customers waiting to be serviced.
 * TaskHandle_t customer_entereing_queue: a freeRTOS TaskHandle_t object used as an alert thread for whenever a customer
 *                                        is entering the queue.
 * TaskHandle_t teller_thread: a freeRTOS TaskHandle_t object used as a thread for when a teller is working with a customer.
 * struct teller tellers: an array of tellers of the struct teller type used as the bank's tellers to service customers.
 */
struct bank{
	QueueHandle_t customers;

	TaskHandle_t customer_entering_queue;
	TaskHandle_t teller_thread;

	struct teller tellers[NUM_TELLERS];
};

// Function Prototypes
void setSimTime(int new_sim);
int getSimTime(void);
void thread_init(void);
void bank_managing_thread(void* argument);
void teller_thread(void* argument);
int rand(int max);
void calculate_total_metrics(void);
void display_total_metrics(void);
char* teller_status_to_string(enum status teller_status);
void display_continuous_metrics(int sim_time);
void print(char* text, ...);
void println(char* text, ...);

#endif
