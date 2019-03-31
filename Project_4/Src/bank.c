#include "bank.h"

total_metrics m;
bank b;

void bank_managing_thread(void* argument){


	b.customers = xQueueCreate(256, sizeof(customer));

	TickType_t start = xTaskGetTickCount();
	TickType_t last_thread_wake = start;

	float simTime = 0;
	bool open = true;

	int customers_entered = 0;

	while(simTime < 420){
		int customer_interval = rand(300) + 100;
		simTime += customer_interval*600;
		if(simTime >= 420){
			break;
		}
		vTaskDelayUntil(&last_thread_wake, pdMS_TO_TICKS(customer_interval));
		customer c = {customers_entered+1, last_thread_wake, 0,0};
		xQueueSend(b.customers,&c,0);
		if(m.max_queue_depth < uxQueueMessagesWaiting(b.customers)){
			m.max_queue_depth = uxQueueMessagesWaiting(b.customers);
		}
		m.customers_served++;
		// PRINT CONTINUOUS METRICS
	}
	TickType_t end = xTaskGetTickCount();

	int customers_left_in_queue = uxQueueMessagesWaiting(b.customers);

	// Total Metrics?
}

void teller_thread(void* argument){
	teller t = {IDLE,0,0,0,0};
	TickType_t last_thread_wake = xTaskGetTickCount();

	int break_interval = rand(3000)+3000;
	for(;;){
		customer c;
		BaseType_t rec_customers = xQueueReceive(b.customers, &c, pdMS_TO_TICKS(break_interval-last_thread_wake));
	}
}

int rand(int max){
	uint32_t random_number;
	HAL_RNG_GenerateRandomNumber(&hrng, &random_number);
	random_number = random_number%max;

	return (random_number>=0 ? (int) random_number : ((int) random_number)*-1);
}

void display_total_metrics(void){
	char customers_served[128];
	char customers_served_teller_1[128];
	char customers_served_teller_2[128];
	char customers_served_teller_3[128];
	char avg_customer_waiting_time[128];
	char avg_teller_time[128];
	char avg_teller_waiting_time[128];
	char max_customer_wait_time[128];
	char max_teller_wait_time[128];
	char max_transaction_time[128];
	char max_queue_depth[128];
	char total_num_breaks[128];
	char avg_break_time[128];
	char max_break_time[128];
	char min_break_time[128];
	sprintf(customers_served, "Total Customers Served: %d\r\n", m.customers_served);
	sprintf(customers_served_teller_1, "\tTeller 1 Served %d Customers\r\n", m.customers_served_per_teller[0]);
	sprintf(customers_served_teller_2, "\tTeller 2 Served %d Customers\r\n", m.customers_served_per_teller[1]);
	sprintf(customers_served_teller_3, "\tTeller 3 Served %d Customers\r\n", m.customers_served_per_teller[2]);
	sprintf(avg_customer_waiting_time, "Average Customer Waiting Time: %f\r\n", m.avg_customer_waiting_time);
	sprintf(avg_teller_time, "Average Time Tellers Spent Helping Customers: %f\r\n", m.avg_teller_time);
	sprintf(avg_teller_waiting_time, "Average Time Tellers Spent Waiting: %f\r\n", m.avg_teller_waiting_time);
	sprintf(max_customer_wait_time, "Max Customer Wait Time: %f\r\n", m.max_customer_wait_time);
	sprintf(max_teller_wait_time, "Max Time Tellers Were Waiting: %f\r\n", m.max_teller_wait_time);
	sprintf(max_transaction_time, "Max Transaction Time: %f\r\n", m.max_transaction_time);
	sprintf(max_queue_depth, "Max Queue Depth: %d\r\n", m.max_queue_depth);
	sprintf(total_num_breaks, "Total Number of Teller Breaks: %d\r\n", m.total_num_breaks);
	sprintf(avg_break_time, "Average Teller Break Time: %f\r\n", m.avg_break_time);
	sprintf(max_break_time, "Max Teller Break Time: %f\r\n", m.max_break_time);
	sprintf(min_break_time, "Minimum Teller Break Time: %f\r\n", m.min_break_time);
}
