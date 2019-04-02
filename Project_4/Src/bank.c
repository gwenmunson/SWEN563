#include "bank.h"

total_metrics m; //Total metrics struct used as a global variable in order to keep track of metrics
SemaphoreHandle_t metric_mutex; //metrics need a mutex so that multiple threads can access it
bank b;//Bank struct used as a global variable to represent the bank as a whole
int simTime;
SemaphoreHandle_t sim_time_mutex;

void setSimTime(int new_sim){
	xSemaphoreTake(sim_time_mutex, 10000);
	simTime = new_sim;
	xSemaphoreGive(sim_time_mutex, 10000);
}

int getSimTime(void){
	int sim;
	xSemaphoreTake(sim_time_mutex, 10000);
	sim = simTime;
	xSemaphoreGive(sim_time_mutex, 10000);
	return sim;
}

void thread_init(void){

	metric_mutex = xSemaphoreCreateBinary();

	//Creating customers queue
	b.customers = xQueueCreate(256, sizeof(customer));

	xTaskCreate(bank_managing_thread, "bank_thread", 256, 0, osPriorityNormal, 0);
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

	//Opening bank
	TickType_t start = xTaskGetTickCount();
	TickType_t last_thread_wake = start;

	setSimTime(0);//Time: 9:00 AM
	bool open = true;//bank is now open!
	int customers_entered = 0;

	// create teller threads
	for(int i = 0; i < NUM_TELLERS; i++){
		xTaskCreate(teller_thread, "teller_thread", 128, i, osPriorityNormal, 0);
	}

	//Begin bank operation
	int localSim = getSimTime();
	while(localSim < 42000){
		int customer_interval = rand(300) + 100;
		setSimTime(localSim += customer_interval);
		if(getSimTime() >= 42000){
			//The bank is closed, so no new customers will be created/serviced
			break;
		}
		vTaskDelayUntil(&last_thread_wake, customer_interval);
		int customer_transaction_time = rand(450) + 30;
		customer c = {customers_entered, last_thread_wake, 0,0, customer_transaction_time};
		xQueueSend(b.customers,&c,0);
		xSemaphoreTake(metric_mutex, 10000);
		if(m.max_queue_depth < uxQueueMessagesWaiting(b.customers)){
			m.max_queue_depth = uxQueueMessagesWaiting(b.customers);
		}
		m.customers_served++;
		xSemaphoreGive(metric_mutex, 10000);
		customers_entered++;
		display_continuous_metrics();
		// !!!!USE vTaskDelete(NULL) FOR PRINTING THREADS!!!!
	}
	TickType_t end = xTaskGetTickCount();
	int customers_left_in_queue = uxQueueMessagesWaiting(b.customers);
	while(uxQueueMessagesWaiting(b.customers) > 0){
		continue;
	}

	calculate_total_metrics();
	display_total_metrics();

	vTaskDelete(NULL);
}

/*
 * Creates a new teller thread for servicing a customer. The amount of time needed to serve a customer is
 * generated here instead of customer creation itself.
 *
 * param argument: A void pointer that takes in any additional arguments sent in from the main.c file.
 * return: none
 */
void teller_thread(void* argument){
	TickType_t last_thread_wake = xTaskGetTickCount();

	int i = *(int*) argument; // teller number for referencing in the bank struct

	int last_break = 0;
	int break_interval = rand(3000)+3000;
	int break_len = rand(300) + 100;
	for(;;){
		int teller_wait_start = xTaskGetTickCount();
		b.tellers[i].teller_status = IDLE;
		customer c;//blank customer to be replaced with popped off customer from queue
		xQueueReceive(b.customers, &c, break_interval-last_thread_wake);
		int teller_wait_time = xTaskGetTickCount - teller_wait_start;
		c.time_left_queue = xTaskGetTickCount() - c.time_entered_queue;
		b.tellers[i].teller_status = BUSY;//working on a customer right now
		vTaskDelayUntil(&last_thread_wake, c.transaction_time); //sleeps thread for length of transaction
		b.tellers[i].teller_status = IDLE;//customer is done being worked on
		b.tellers[i].num_customers += 1;
		b.tellers[i].total_transaction_time += c.transaction_time;
		int customer_time_in_queue = c.time_left_queue - c.time_entered_queue;

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
		xSemaphoreGive(metric_mutex, 10000);
		if(getSimTime() - last_break >= break_interval){
			xSemaphoreTake(metric_mutex,10000);
			m.total_num_breaks[i]++;
			m.total_break_time[i] += break_len;
			if(break_len > m.max_break_time[i]){
				m.max_break_time[i] = break_len;
			}
			if(break_len < m.min_break_time[i]){
				m.min_break_time[i] = break_len;
			}
			xSemaphoreGive(metric_mutex,10000);
			break_interval = rand(3000) + 3000;
			break_len = rand(300) + 100;
			vTaskDelayUntil(&last_thread_wake, break_len);
		}

	}//end for loop
}


/*
 * Random Number Generator using freeRTOS's provided RNG code. Starts at 0.
 *
 * param max: the max value that a random number can be (range is 0 to max).
 * return: random number generated
 */
int rand(int max){
	uint32_t random_number;
	HAL_RNG_GenerateRandomNumber(&hrng, &random_number);
	random_number = random_number%max;

	return (random_number>=0 ? (int) random_number : ((int) random_number)*-1);
}

void calculate_total_metrics(void){
	xSemaphoreTake(metric_mutex, 10000000);
	m.avg_customer_waiting_time = m.total_customer_queue_time/m.customers_served;
	m.avg_teller_time = m.total_customer_teller_time/m.customers_served;
	m.avg_teller_waiting_time = m.total_teller_wait_time/m.customers_served;
	for(int i = 0; i < NUM_TELLERS; i++){
		m.avg_break_time[i] = m.total_break_time[i]/m.total_num_breaks[i];
	}
	xSemaphoreGive(metric_mutex, 10000000);
}

/*
 * Handles printing out total metrics. May be reworked in the future, so this is more placeholder code.
 *
 * param: none
 * return: none
 */
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
	xSemaphoreTake(metric_mutex,10000000);
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
	xSemaphoreGive(metric_mutex,10000000);
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

	int simHours = sim_time/60;
	int simMin = sim_time%60;


	sprintf(current_sim_time, "Current Simulation Time: %d:%d \r\n", simHours, simMin);
	sprintf(teller_status_1, "Teller 1 Status: %s\r\n\tCustomers Served: %d\r\n", b.tellers[0].teller_status, b.tellers[0].num_customers);
	sprintf(teller_status_2, "Teller 2 Status: %s\r\n\tCustomers Served: %d\r\n", b.tellers[1].teller_status, b.tellers[1].num_customers);
	sprintf(teller_status_3, "Teller 3 Status: %s\r\n\tCustomers Served: %d\r\n", b.tellers[2].teller_status, b.tellers[2].num_customers);
}

/*
 * Prints out formatted text to the USART
 * param text: the formatted string
 * param ...: The variable amount of values to pass along
 */
void print(char *text, ...) {
   va_list args;
	 int bufSize;
	 char testBuffer[1]; //now needed apparently
	
   va_start(args, text);
	 bufSize=vsnprintf(testBuffer,1,text,args); //need to write to the testbuffer. vsnprintf wont work with null and 0 arguments anymore for some reason
	 testBuffer[0]='\0';
	
	 char textBuffer[bufSize+1];
	
   vsprintf(textBuffer, text, args);
	 textBuffer[bufSize]='\0';
   va_end(args);
	
	 HAL_UART_Transmit(&huart2, (uint8_t *)textBuffer, bufSize+1, 1000000);
	 memset(textBuffer,0,strlen(textBuffer));
}


/*
 * Prints out formatted text to the USART, along with a newline at the end
 * Note: Unfortunately, it is no where near so simple to pass along a va_list to another
 * variable function, so the code is just  repeated with the newline at the end
 *  
 * param text: The formatted String
 * param ...: The variable amount of values to pass along
 */
void println(char *text, ...) {
	 va_list args;
   int bufSize;
	 char testBuffer[1]; //now needed apparently
	
   va_start(args, text);
	 bufSize=vsnprintf(testBuffer,1,text,args); //need to write to the testbuffer. vsnprintf wont work with null and 0 arguments anymore for some reason
	 testBuffer[0]='\0';
	
	 char textBuffer[bufSize+1];
	
   vsprintf(textBuffer, text, args);
	 textBuffer[bufSize]='\0';
   va_end(args);
	
	 HAL_UART_Transmit(&huart2, (uint8_t *)textBuffer, bufSize+1, 1000000);
	 memset(textBuffer,0,strlen(textBuffer));
	 
	 HAL_UART_Transmit(&huart2, (uint8_t *)NEWLINE, strlen(NEWLINE), 1000000);
}


