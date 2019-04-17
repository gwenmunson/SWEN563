/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "UART_1.h"
#include "threads_2.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim5;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

// Struct for holding recipes - used because 2d arrays wouldn't work
typedef struct recipes{
    unsigned char* recipe;
}recipes;

SemaphoreHandle_t  servo_mutex;
SemaphoreHandle_t wait_mutex;
struct commands servo_commands = {{noop,noop},{false,false}};

unsigned char test_recipe[] = {MOV|0, MOV|5, MOV|0, MOV|3, LOOP|0, MOV|1, MOV|4, END_LOOP, MOV|0, MOV|2, WAIT|0, MOV|3, WAIT|0, MOV|2, MOV|3, WAIT|31, WAIT|31, WAIT|31, MOV|4, RECIPE_END, MOV|3};
unsigned char move_recipe[] = {MOV|0, MOV|1, MOV|3, MOV|2, MOV|5, MOV|4, RECIPE_END, MOV|1};
unsigned char wait_recipe[] = {MOV|0, MOV|3, WAIT|31,WAIT|31,WAIT|31, MOV|5};
unsigned char cmd_err_recipe[] = {MOV|1, MOV|4, WAIT|12, MOV|2, LOOP|2, MOV|1, MOV|2, END_LOOP, MOV|6, MOV|4, RECIPE_END};
unsigned char loop_err_recipe[] = {MOV|2, MOV|5, MOV|1, WAIT|31, LOOP|3, MOV|2, MOV|5, LOOP|1, MOV|4, MOV|1, END_LOOP, END_LOOP, RECIPE_END};

// list of recipes
struct recipes rec_list[2] = {{0},{0}};	

//int timeout = 0;
//uint8_t command;
int program_counter[] = {0,0};
int wait_count[] = {0,0};
int in_loop[] = {0,0};
int loop_counter[] = {0,0};
int loop_instruction[] = {0,0};
enum user_events user_command[] = {pause_recipe,pause_recipe};
int new_command[] = {0,0};
enum states servo_state[] = {recipe_paused, recipe_paused};
int position[] = {3,3};
int id[2] = {0,1};
bool cancel = false;


UART_HandleTypeDef huart2;

uint8_t rx_buffer[10];  // Shared buffer between foreground and UART RX
uint8_t rx_byte;        // the currently received byte
uint8_t rx_index = 0;   // pointer into the rx_buffer
SemaphoreHandle_t  transmit_mutex;  // protects UART transmitter resource
SemaphoreHandle_t  receive_mutex;   // protects UART receiveer resource


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RNG_Init(void);
//static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RNG_Init();
  //MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	LED_Init();
  UART_Init();//&huart2);
	Timer5_init();
	GPIO_Init();

	TIM5->CR1 |= TIM_CR1_CEN;
	char init_print[20];
	sprintf(init_print, "Initialization done\r\n");
	vPrintString(init_print);
	
	rec_list[0].recipe = move_recipe;
  rec_list[1].recipe = test_recipe;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  thread_init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 79;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1000;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/









void UART_Init(void){//UART_HandleTypeDef *huart2){
	//huart2_thread = *huart2;
	transmit_mutex = xSemaphoreCreateMutex();     // create mutex to protect UART transmitter resource
	receive_mutex = xSemaphoreCreateMutex();      // create mutex to protect UART receiver resource
	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);    // one time, kick off receive interrupt (repeated from within rx callback)
}	

/*
 * prints the string, blocks if UART busy, thus safe from multiple threads
 */
void vPrintString(char *message) {
  xSemaphoreTake(transmit_mutex, ~0);         // Wait forever until the USART is free, then take mutex
  HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 10000);
  xSemaphoreGive(transmit_mutex);             // Give up mutex after printing
	//HAL_Delay(100);
}

/*
 * overrides _weak HAL receiver callback, called when byte received
 */
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
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // one time, kick off receive interrupt (repeated from within callback)
  }
}

void thread_init(void){
	servo_mutex = xSemaphoreCreateMutex();
	
	char init_print[32];
	sprintf(init_print, "Initializing Threads\r\n");
	vPrintString(init_print);

	xTaskCreate(input_thread, "input_thread", 128, NULL, osPriorityNormal, 0);
	
	xTaskCreate(servo_thread, "servo_thread", 128, (void *)&id[0], osPriorityNormal, 0);
	xTaskCreate(servo_thread, "servo_thread", 128, (void *)&id[1], osPriorityNormal, 0);
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
				wait_count[i] = (args+1) * (6000);
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
	//char new_comm[20];
	//sprintf(new_comm,"IsNewCommand = %d\r\n", is_updated);
	//vPrintString(new_comm);
	return is_updated;
}

void servo_thread(void* argument){
	int i = *(int*) argument;
	for(;;){
		if(IsNewCommand(i)){
			xSemaphoreTake(servo_mutex, 10000);
			enum user_events command = servo_commands.event[i];
			xSemaphoreGive(servo_mutex);
			process_event(command, servo_state[i], i);
			xSemaphoreTake(servo_mutex, 10000);
			servo_commands.updated[i] = false;
			xSemaphoreGive(servo_mutex);
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
/*
void CaptureCommands(void){
	if(command == 0){
		return;
	}
	else{
		USART_Write(USART2, &command, 1);
		ParseCommand(command);
	}
}
*/
void ParseCommand(char rx_byte[2]){
	for(int i = 0; i<2; i++){
		if(rx_byte[i] == 0x58 || rx_byte[i] == 0x78){
			servo_commands.event[0] = noop;
			servo_commands.event[1] = noop;
			servo_commands.updated[0] = false;
			servo_commands.updated[1] = false;
			
			break;
		}
		switch(rx_byte[i]){
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
			case 0x53:
			case 0x73:
				servo_commands.event[i] = sweep;
				servo_commands.updated[i] = true;
				break;
			default:
				servo_commands.event[i] = noop;
				break;
		}//end switch statement
	}
}

/*
void GetCommandsTimer(){
	while(!(USART2->ISR & USART_ISR_RXNE)){
		if(timeout == 1){
			command = 0;
			return;
		}
	}
	uint8_t val = USART2->RDR & 0xFF;;
	command = val;
}
*/

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
			else if(one_event == sweep){
				int current_pos = position[servo_num];
				SetPosition(servo_num, 0);
				for(int i = 0; i<((500*SERVO_DELAY)); i++);
				SetPosition(servo_num, 5);
				for(int i = 0; i<((500*SERVO_DELAY)); i++);
				SetPosition(servo_num, current_pos);
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
	char display_msg[32];
	sprintf(display_msg, "Input thread initialized\n\r");
	vPrintString(display_msg);
	char rx_bytes[2];
	int i = 0;
	
	sprintf(display_msg, "\r\n>> ");
	vPrintString(display_msg);
	
	for(;;){
		//for(int i = 0; i<3; i++){
		while(i < 3){
			//let's grab the input from the user
			HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
			if(rx_byte != 0){
				char print_byte[1];
				sprintf(print_byte, "%c", rx_byte);
				vPrintString(print_byte);
				if(i != 2){
					rx_bytes[i] = rx_byte;
				}			
				else if(i == 2){
					xSemaphoreTake(servo_mutex, 10000);
					ParseCommand(rx_bytes);
					xSemaphoreGive(servo_mutex);
				}
				i++;
			}
			rx_byte = 0;
		}//end if statement
		i = 0;
		vPrintString(display_msg);
	}//end for loop	
}

