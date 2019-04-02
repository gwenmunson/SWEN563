#include "bank.h"

total_metrics m; //Total metrics struct used as a global variable in order to keep track of metrics
bank b;//Bank struct used as a global variable to represent the bank as a whole

/*
 * Handles the operation of the bank, starting up the bank and creating customers and handling customers
 * until the bank closes at 4:00 PM (7 hours system runtime). The bank time always starts at 9:00 AM.
 * All metrics are recorded using the total_metrics global variable.
 *
 * param argument: A void pointer that takes in any additional arguments sent in from the main.c file. 
 * return: none 
 */
void bank_managing_thread(void* argument){

	//Creating customers
	b.customers = xQueueCreate(256, sizeof(customer));

	//Opening bank
	TickType_t start = xTaskGetTickCount();
	TickType_t last_thread_wake = start;

	float simTime = 0;//Time: 9:00 AM
	bool open = true;//bank is now open!
	int customers_entered = 0;

	//Begin bank operation
	while(simTime < 420){
		int customer_interval = rand(450) + 30;
		simTime += customer_interval;
		if(simTime >= 420){
			//The bank is closed, so no new customers will be created/serviced
			break;
		}
		vTaskDelayUntil(&last_thread_wake, pdMS_TO_TICKS(customer_interval));
		customer c = {customers_entered+1, last_thread_wake, 0,0, customer_interval};
		xQueueSend(b.customers,&c,0);
		if(m.max_queue_depth < uxQueueMessagesWaiting(b.customers)){
			m.max_queue_depth = uxQueueMessagesWaiting(b.customers);
		}
		m.customers_served++;
		// TODO: PRINT CONTINUOUS METRICS
		// !!!!USE vTaskDelete(NULL) FOR PRINTING THREADS!!!!
	}
	TickType_t end = xTaskGetTickCount();

	int customers_left_in_queue = uxQueueMessagesWaiting(b.customers);

	// TODO: Total Metrics?
}

/*
 * Creates a new teller thread for servicing a customer. The amount of time needed to serve a customer is
 * generated here instead of customer creation itself. 
 *
 * param argument: A void pointer that takes in any additional arguments sent in from the main.c file. 
 * return: none 
 */
void teller_thread(void* argument){
	//teller t = {IDLE,0,0,0,0};
	TickType_t last_thread_wake = xTaskGetTickCount();

	int break_interval = rand(3000)+3000;
	//add break functionality roll here (idk the random chance of a break happening?)
	for(int i = 0; i < NUM_TELLERS; i++){
		if(b.tellers[i].teller_status == IDLE){
			for(;;){
				customer c;//blank customer to be replaced with popped off customer from queue
				BaseType_t rec_customers = xQueueReceive(b.customers, &c, pdMS_TO_TICKS(break_interval-last_thread_wake));
				b.tellers[i].teller_status = BUSY;//working on a customer right now
				//ADD SLEEP CODE HERE I'M DUMB
				b.tellers[i].teller_status = IDLE;//customer is done being worked on
				b.tellers[i].num_customers += 1;
				b.tellers[i].total_transaction_time += c.interval_time;
			}//end for loop
		}//end if statement
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


