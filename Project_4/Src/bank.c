#include "bank_1.h"

struct total_metrics m; //Total metrics struct used as a global variable in order to keep track of metrics
SemaphoreHandle_t metric_mutex; //metrics need a mutex so that multiple threads can access it
struct bank b;//Bank struct used as a global variable to represent the bank as a whole
int simTime;
SemaphoreHandle_t sim_time_mutex;
SemaphoreHandle_t HAL_mutex;
int id[NUM_TELLERS];
TickType_t start;

void setSimTime(int new_sim){
	//xSemaphoreTake(sim_time_mutex, 10000);
	simTime = new_sim;
	//xSemaphoreGive(sim_time_mutex);
}

int getSimTime(void){
	int sim;
	//xSemaphoreTake(sim_time_mutex, 10000);
	sim = simTime;
	//xSemaphoreGive(sim_time_mutex);
	return sim;
}
struct teller teller_init(void){
	struct teller t = {IDLE, 0, 0, 0, 0, 0};
	return t;
}

void bank_init(void){
	//Creating customers queue
	b.customers = xQueueCreate(50, sizeof(struct customer));

	for(int i = 0; i<NUM_TELLERS; i++){
		b.tellers[i] = teller_init();
	}
}

struct total_metrics metric_init(void){
	struct total_metrics met;
	met.customers_served = 0;
	for(int i = 0; i<NUM_TELLERS; i++){
		met.customers_served_per_teller[i] = 0;
		met.total_num_breaks[i] = 0;
		met.avg_break_time[i] = 0;
		met.max_break_time[i] = 0;
		met.min_break_time[i] = 400;
		met.total_break_time[i] = 0;
	}
	met.avg_customer_waiting_time = 0;
	met.avg_teller_time = 0;
	met.avg_teller_waiting_time = 0;
	met.max_customer_wait_time = 0;
	met.max_teller_wait_time = 0;
	met.max_transaction_time = 0;
	met.max_queue_depth = 0;
	met.total_customer_queue_time = 0;
	met.total_customer_teller_time = 0;
	met.total_teller_wait_time = 0;

	return met;
}

void thread_init_1(void){

	HAL_mutex = xSemaphoreCreateMutex();
	metric_mutex = xSemaphoreCreateMutex();
	sim_time_mutex = xSemaphoreCreateMutex();

	char init_print[128];
	sprintf(init_print, "Initializing Threads\r\n");
	print(init_print);

	bank_init();

	m = metric_init();

	xTaskCreate(bank_managing_thread, "bank_thread", 128, 0, osPriorityNormal, 0);
	// create teller threads
	for(int i = 0; i < NUM_TELLERS; i++){
		id[i] = i;
		xTaskCreate(teller_thread, "teller_thread", 128, (void*) &id[i], osPriorityNormal, 0);
	}
}


/*
 * Handles the operation of the bank, starting up the bank and creating customers and handling customers
 * until the bank closes at 4:00 PM (7 hours system runtime). The bank time always starts at 9:00 AM.
 * All metrics are recorded using the total_metrics global variable.
 *
 * param argument: A void pointer that takes in any additional arguments sent in from the main.c file.
 * return: none
 */
void bank_managing_thread(void* argument){
	char enter_bank[32];
	sprintf(enter_bank, "Entering bank thread\r\n");
	print(enter_bank);

	//Opening bank
	start = xTaskGetTickCount();
	TickType_t last_thread_wake = start;

	setSimTime(0);//Time: 9:00 AM
	bool open = true;//bank is now open!
	int customers_entered = 0;


	//Begin bank operation
	int localSim = getSimTime();
	do{
		int customer_interval = random(300) + 100;
		//osDelay(customer_interval);
		vTaskDelayUntil(&last_thread_wake, customer_interval);
		setSimTime(localSim += customer_interval);
		if(getSimTime() >= 42000){
			open = false;
		}
		int customer_transaction_time = random(450) + 30;
		struct customer c = {customers_entered, last_thread_wake, 0,0, customer_transaction_time};
		xQueueSend(b.customers,&c,0);
		char test[20];
		sprintf(test, "Customers in queue: %d\r\n", (int)uxQueueMessagesWaiting(b.customers));
		print(test);
		xSemaphoreTake(metric_mutex, 10000);
		if(m.max_queue_depth < uxQueueMessagesWaiting(b.customers)){
			m.max_queue_depth = uxQueueMessagesWaiting(b.customers);
		}
		xSemaphoreGive(metric_mutex);
		customers_entered++;
		display_continuous_metrics(getSimTime());
	}while(open);
	TickType_t end = xTaskGetTickCount();
	xSemaphoreTake(metric_mutex, 10000);
	m.customers_served = customers_entered;
	xSemaphoreGive(metric_mutex);

	int customers_left_in_queue = uxQueueMessagesWaiting(b.customers);
	while(1){
		int current_customers = (int)uxQueueMessagesWaiting(b.customers);
		if(current_customers < customers_left_in_queue - 3){
			display_continuous_metrics(getSimTime());
			customers_left_in_queue = current_customers;
		}
	}

	calculate_total_metrics();
	display_total_metrics();

	while(1){}
}
/*
 * Creates a new teller thread for servicing a customer. The amount of time needed to serve a customer is
 * generated here instead of customer creation itself.
 *
 * param argument: A void pointer that takes in any additional arguments sent in from the main.c file.
 * return: none
 */
void teller_thread(void* argument){
	int i = *(int*) argument; // teller number for referencing in the bank struct

	//char enter_teller[32];
	//sprintf(enter_teller, "Entering teller thread %d\r\n", i+1);
	//print(enter_teller);

	TickType_t last_thread_wake = xTaskGetTickCount();

	int last_break = 0;
	int break_interval = random(3000)+3000;
	int break_len = random(300) + 100;
	int break_time_at = last_thread_wake + break_interval;
	for(;;){
		//int teller_wait_start = xTaskGetTickCount();
		b.tellers[i].teller_status = IDLE;
		struct customer c;//blank customer to be replaced with popped off customer from queue
		BaseType_t rec = pdTRUE;
		for(;;){
			rec = xQueueReceive(b.customers, &c, break_time_at - last_thread_wake);
			if(rec != pdTRUE){
				b.tellers[i].teller_status = BREAK;
				xSemaphoreTake(metric_mutex, 10000);
				m.total_num_breaks[i]++;
				xSemaphoreGive(metric_mutex);
				vTaskDelayUntil(&last_thread_wake,break_len);
				b.tellers[i].teller_status = IDLE;
				xSemaphoreTake(metric_mutex, 10000);
				m.total_break_time[i] += break_len;
				if(break_len > m.max_break_time[i]){
					m.max_break_time[i] = break_len;
				}
				if(break_len < m.min_break_time[i]){
					m.min_break_time[i] = break_len;
				}
				xSemaphoreGive(metric_mutex);
				break_interval = random(3000)+3000;
				break_len = random(300)+100;
				break_time_at = last_thread_wake + break_interval;
			}
			else{
				break;
			}
		}

		int teller_wait_time = xTaskGetTickCount() - last_thread_wake;
		b.tellers[i].num_customers += 1;
		b.tellers[i].teller_status = BUSY;//working on a customer right now

		int customer_time_in_queue = xTaskGetTickCount() - c.time_entered_queue;
		int transaction_time = random(450) + 30;
		b.tellers[i].total_transaction_time += transaction_time;
		//osDelay(transaction_time);
		vTaskDelayUntil(&last_thread_wake, transaction_time); //sleeps thread for length of transaction

		xSemaphoreTake(metric_mutex, 10000);
		m.total_customer_queue_time += customer_time_in_queue;
		if(customer_time_in_queue > m.max_customer_wait_time){
			m.max_customer_wait_time = customer_time_in_queue;
		}
		m.total_customer_teller_time += c.transaction_time;
		if(c.transaction_time > m.max_transaction_time){
			m.max_transaction_time = c.transaction_time;
		}
		m.customers_served_per_teller[i]++;
		m.total_teller_wait_time = teller_wait_time;
		if(teller_wait_time > m.max_teller_wait_time){
			m.max_teller_wait_time = teller_wait_time;
		}
		xSemaphoreGive(metric_mutex);
		//if(getSimTime() - last_break >= break_interval){
		//	xSemaphoreTake(metric_mutex,10000);
		//	m.total_num_breaks[i]++;
		//	m.total_break_time[i] += break_len;
		//	if(break_len > m.max_break_time[i]){
		//		m.max_break_time[i] = break_len;
		//	}
		//	if(break_len < m.min_break_time[i]){
		//		m.min_break_time[i] = break_len;
		//	}
		//	xSemaphoreGive(metric_mutex);
		//	break_interval = random(3000) + 3000;
		//	break_len = random(300) + 100;
		//	vTaskDelayUntil(&last_thread_wake, break_len);
		//}

		//if(getSimTime() >= 42000 && uxQueueMessagesWaiting(b.customers) == 0){
		//	//The bank is closed, so no new customers will be created/serviced
		//	break;
		//}
	}//end for loop
	//while(1){}
}


/*
 * Random Number Generator using freeRTOS's provided RNG code. Starts at 0.
 *
 * param max: the max value that a random number can be (range is 0 to max).
 * return: random number generated
 */
int random(int max){
	uint32_t random_number;
	HAL_RNG_GenerateRandomNumber(&hrng, &random_number);
	random_number = random_number%max;

	return (int)random_number;
}

void calculate_total_metrics(void){
	char display_message[32];
	sprintf(display_message, "Calculating Total Metrics\r\n");
	print(display_message);

	xSemaphoreTake(metric_mutex, 10000000);
	sprintf(display_message, "Got mutex\r\n");
	//print(display_message);

	sprintf(display_message, "Customers served: %d\r\n", m.customers_served);
	//print(display_message);
	m.avg_customer_waiting_time = m.total_customer_queue_time/m.customers_served;
	sprintf(display_message, "Avg cust waiting time: %f\r\n", m.avg_customer_waiting_time);
	//print(display_message);
	m.avg_teller_time = m.total_customer_teller_time/m.customers_served;
	sprintf(display_message, "Avg teller time: %f\r\n", m.avg_teller_time);
	//print(display_message);
	m.avg_teller_waiting_time = m.total_teller_wait_time/m.customers_served;
	sprintf(display_message, "Avg teller waiting time: %f\r\n", m.avg_teller_waiting_time);
	//print(display_message);
	//for(int i = 0; i < NUM_TELLERS; i++){
	//	m.avg_break_time[i] = m.total_break_time[i]/m.total_num_breaks[i];
	//}
	sprintf(display_message, "Giving back mutex\r\n");
	//print(display_message);
	xSemaphoreGive(metric_mutex);
	sprintf(display_message, "Gave back mutex\r\n");
	//print(display_message);
}

/*
 * Handles printing out total metrics. May be reworked in the future, so this is more placeholder code.
 *
 * param: none
 * return: none
 */
void display_total_metrics(void){
	char display_message[32];
	sprintf(display_message, "Printing Total Metrics\r\n");
	print(display_message);

	char customers_served[64];
	char customers_served_teller_1[64];
	char customers_served_teller_2[64];
	char customers_served_teller_3[64];
	char avg_customer_waiting_time[64];
	char avg_teller_time[64];
	char avg_teller_waiting_time[64];
	char max_customer_wait_time[64];
	char max_teller_wait_time[64];
	char max_transaction_time[64];
	char max_queue_depth[64];
	char total_num_breaks[64];
	char avg_break_time[64];
	char max_break_time[64];
	char min_break_time[64];
	xSemaphoreTake(metric_mutex,10000000);
	sprintf(display_message, "Got mutex\r\n");
	//print(display_message);
	sprintf(customers_served, "Total Customers Served: %d\r\n", m.customers_served);
	print(customers_served);
	sprintf(customers_served_teller_1, "\tTeller 1 Served %d Customers\r\n", m.customers_served_per_teller[0]);
	print(customers_served_teller_1);
	sprintf(customers_served_teller_2, "\tTeller 2 Served %d Customers\r\n", m.customers_served_per_teller[1]);
	print(customers_served_teller_2);
	sprintf(customers_served_teller_3, "\tTeller 3 Served %d Customers\r\n", m.customers_served_per_teller[2]);
	print(customers_served_teller_3);
	sprintf(avg_customer_waiting_time, "Average Customer Waiting Time: %f\r\n", m.avg_customer_waiting_time);
	print(avg_customer_waiting_time);
	sprintf(avg_teller_time, "Average Time Tellers Spent Helping Customers: %f\r\n", m.avg_teller_time);
	print(avg_teller_time);
	sprintf(avg_teller_waiting_time, "Average Time Tellers Spent Waiting: %f\r\n", m.avg_teller_waiting_time);
	print(avg_teller_waiting_time);
	sprintf(max_customer_wait_time, "Max Customer Wait Time: %d\r\n", m.max_customer_wait_time);
	print(max_customer_wait_time);
	sprintf(max_teller_wait_time, "Max Time Tellers Were Waiting: %d\r\n", m.max_teller_wait_time);
	print(max_teller_wait_time);
	sprintf(max_transaction_time, "Max Transaction Time: %d\r\n", m.max_transaction_time);
	print(max_transaction_time);
	sprintf(max_queue_depth, "Max Queue Depth: %d\r\n", m.max_queue_depth);
	print(max_queue_depth);
	//sprintf(total_num_breaks, "Total Number of Teller Breaks: %d\r\n", m.total_num_breaks);
	//sprintf(avg_break_time, "Average Teller Break Time: %f\r\n", m.avg_break_time);
	//sprintf(max_break_time, "Max Teller Break Time: %f\r\n", m.max_break_time);
	//sprintf(min_break_time, "Minimum Teller Break Time: %f\r\n", m.min_break_time);
	xSemaphoreGive(metric_mutex);

}

char* teller_status_to_string(enum status teller_status){
	if(teller_status == IDLE){
		return "Idle";
	}
	else if(teller_status == BUSY){
		return "Busy";
	}
	else if(teller_status == BREAK){
		return "Break";
	}
	else{
		return "Unknown";
	}
}

void display_continuous_metrics(int sim_time){
	char current_sim_time[128];
	char num_customers_in_queue[128];
	char teller_status_1[128];
	char teller_status_2[128];
	char teller_status_3[128];

	int simHours = sim_time/6000;
	int simMin = (sim_time/100)%60;


	sprintf(current_sim_time, "Current Simulation Time: %d:%02d \r\n", simHours+9, simMin);
	sprintf(teller_status_1, "Teller 1 Status: %s\r\n\tCustomers Served: %d\r\n", teller_status_to_string(b.tellers[0].teller_status), b.tellers[0].num_customers);
	sprintf(teller_status_2, "Teller 2 Status: %s\r\n\tCustomers Served: %d\r\n", teller_status_to_string(b.tellers[1].teller_status), b.tellers[1].num_customers);
	sprintf(teller_status_3, "Teller 3 Status: %s\r\n\tCustomers Served: %d\r\n", teller_status_to_string(b.tellers[2].teller_status), b.tellers[2].num_customers);

	print(current_sim_time);
	print(teller_status_1);
	print(teller_status_2);
	print(teller_status_3);
}

void print(char* text){
	xSemaphoreTake(HAL_mutex, 1000000);
	HAL_UART_Transmit(&huart2, (uint8_t *)text, strlen(text), 1000000);
	xSemaphoreGive(HAL_mutex);
}
