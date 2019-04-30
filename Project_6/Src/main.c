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
#include "../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"
#include "servo.h"
#include "UART.h"
#include "gpio.h"
#include "recipe.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_THRESHOLD_RANGE 75

#define OPCODE_MASK 0xE0
#define ARGS_MASK 0x1F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
UART_HandleTypeDef huart2;

uint8_t rx_buffer[10];  // Shared buffer between foreground and UART RX
uint8_t rx_byte;        // the currently received byte
uint8_t rx_index = 0;   // pointer into the rx_buffer
SemaphoreHandle_t  transmit_mutex;  // protects UART transmitter resource
SemaphoreHandle_t  receive_mutex;   // protects UART receiveer resource

char print_buffer[128];

float gyro_val[] = {0.0, 0.0, 0.0};
uint32_t gyro_angle[] = {0, 0, 0};

int player_pos = 0;

int display_player_pos = 0;


int program_counter = 0;

unsigned char Level_1[] = {MOV|0, MOV|1, MOV|3, MOV|2, MOV|5};
unsigned char Level_2[] = {MOV|1, MOV|4, MOV|1, MOV|3, MOV|4};
unsigned char Level_3[] = {MOV|0, MOV|1, MOV|2, MOV|0, MOV|5};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void thread_init(void);
void gyro_thread(void* argument);
int random(int max, int min);
void servo_thread(void* argument);
void Timer5_init(void);
int FindCurrentPos(int arg);
void parse_recipe (int op, int args);
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
	//BSP_GYRO_Init();
	UART_Init();
	//Timer5_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RNG_Init();
  MX_TIM5_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	//UART_Init();
	//GPIO_Init();
	//Timer5_init();
	BSP_GYRO_Init();
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	SetPosition(0, 500);
	SetPosition(1, 1000);

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
 * Timer5_init() - initializes timer registers for Timer5 PWM generation
 */
void Timer5_init(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

	TIM5->PSC |= 0x1F3F;
	//TIM5->EGR |= TIM_EGR_UG;

	TIM5->CCMR1 &= ~(0x03 << 8);
    TIM5->CCMR2 &= ~(0x03 << 0);
    TIM5->CCMR1 &= ~(0x07 << 12);
	TIM5->CCMR2 &= ~(0x07 << 12);
    TIM5->CCMR1 |= (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
	TIM5->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);

	TIM5->CR1 |= TIM_CR1_ARPE;
	TIM5->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E);

	TIM5->ARR = 200;

	TIM5->CCR2 = 10;
	TIM5->CCR3 = 10;

	TIM5->EGR |= TIM_EGR_UG;

	//TIM5->CR1 |= TIM_CR1_CEN;
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

/*
 * Initializes the servo thread and the gyro thread.
 * @param: void
 * @return: void
 */
void thread_init(void){

	xTaskCreate (servo_thread,	"servo_thread", 256, NULL, osPriorityNormal, NULL);
	xTaskCreate (gyro_thread,	"gyro_thread", 256, NULL, osPriorityNormal, NULL);
}

/*
 * Gyroscopic thread creation. This code works by grabbing the raw gyro value and getting
 * the angle of the gyro (relative to the Z-axis). This angle is used in the calcuation
 * of the actual position of the board.
 *
 * @param argument: void pointer used for thread instantiation, currently not used within
 *                  the code.
 * @return: void
 */
void gyro_thread(void* argument) {
  char buffer[128];
	int gyro_pos = 0;
	int num_display = 0;
  for(;;){

		BSP_GYRO_GetXYZ(gyro_val);   // get raw values from gyro device

    // integrate angular velocity to get angle
    gyro_angle[2] += (int32_t)(gyro_val[2] / 10000);

		//Player position setting functionality
		gyro_pos = ((gyro_angle[2]+150)*5) + (MIN_SERVO_PWM);
		if(gyro_pos < MIN_SERVO_PWM){
			SetPosition(PLAYER_SERVO, MIN_SERVO_PWM);
			HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
			player_pos = MIN_SERVO_PWM;
		}
		else if(gyro_pos > MAX_SERVO_PWM){
			SetPosition(PLAYER_SERVO, MAX_SERVO_PWM);
			player_pos = MAX_SERVO_PWM;
		}
		else{
			SetPosition(PLAYER_SERVO, gyro_pos);
			player_pos = gyro_pos;
		}
		/*
		if(display_player_pos){
			if(num_display == 0){
				sprintf(buffer, "player_pos: %4d",player_pos);
				vPrintString(buffer);
			}
			else{
				sprintf(buffer, "\b\b\b\b%4d", player_pos);
				vPrintString(buffer);
			}
			num_display++;
		}
		if(!display_player_pos){
			num_display = 0;
		}
		*/
    osDelay(50);//delay as to not run too fast
  }
}
/*
 * Servo thread creation. This code handles the actual game and the user interface. The user
 * decides whether to run the base game (mirror game) or the levels (recipes). The base game
 * runs through 10 rounds, delaying for 5 seconds each round. If the player does not get within
 * 5% of the duty cycle, no win is recorded; otherwise, a win is recorded. The levels are
 * recipes already initialized that the computer servo will use to move around and that the
 * player is expected to get within 5% of the servo in one round. Feedback is displayed after
 * every round, and, at the end of the levels or 10 rounds for the base game, the overall
 * score is displayed, and the game ends. The cycle is then repeated again.
 *
 * @param argument: void pointer used for thread instantiation, currently not used within
 *                  the code.
 * @return: void
 */
void servo_thread(void* argument){
	int current_pos = 0, next_pos = 0;
	int score = 0;//total amount of wins
	uint8_t rx_byte = 0;//character from user input

	//Thread Startup
	sprintf(print_buffer,"Starting game thread!\r\n");
	vPrintString(print_buffer);
	sprintf(print_buffer, "Would you like to:\r\n\t1) Play base game\r\n\t2) Play game levels\r\n");
	vPrintString(print_buffer);
	//Waiting for user input to begin...
	while(rx_byte == 0){
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
	}
	//Base Game functionality
	if(rx_byte == '1'){
		sprintf(print_buffer, "Starting base game!\r\n");
		vPrintString(print_buffer);
		//Loop for rounds 1-10
		for(int i = 1; i<11; i++){
			next_pos = random(MAX_SERVO_PWM, MIN_SERVO_PWM);
			//handling for different position than last round
			while(next_pos > current_pos-200 && next_pos < current_pos+200){
				next_pos = random(MAX_SERVO_PWM, MIN_SERVO_PWM);
			}
			//Beginning of round
			sprintf(print_buffer, "Starting Round %d", i);
			vPrintString(print_buffer);
			SetPosition(COMPUTER_SERVO, next_pos);
			HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
			current_pos = next_pos;

			//sprintf(print_buffer, "servo_pos: %d\t", current_pos);
			//vPrintString(print_buffer);
			//display_player_pos = 1;

			osDelay(5000);//Simulating round time
			//display_player_pos = 0;
			//Successful win!
			if(player_pos >= (current_pos - SERVO_THRESHOLD_RANGE) && player_pos <= (current_pos + SERVO_THRESHOLD_RANGE)){
				score++;
				sprintf(print_buffer, ": PLAYER SUCCESSFUL!\r\n");
				vPrintString(print_buffer);
			}
			//Failure
			else{
				sprintf(print_buffer, ": PLAYER FAIL!\r\n");
				vPrintString(print_buffer);
			}//end else statement
		}//end for loop
		//End of game
		sprintf(print_buffer, "Score: %d\r\nGAME OVER\r\n", score);
		vPrintString(print_buffer);
	}

 //Level Game functionality
	if(rx_byte == '2'){
		//Initialization for Level 1
		sprintf(print_buffer,"Starting Level 1\r\n");
		vPrintString(print_buffer);
		//Rounds 1-5
		for(int i = 0; i<5; i++){
			sprintf(print_buffer, "Starting Round %d", i+1);
			vPrintString(print_buffer);
			int recipe_command = Level_1[i];
			int op = recipe_command & OPCODE_MASK;
			int args = recipe_command & ARGS_MASK;
			parse_recipe(op, args);

			current_pos = FindCurrentPos(args);

			osDelay(5000);//simulate 5 seconds real time

			if(player_pos >= (current_pos - SERVO_THRESHOLD_RANGE) && player_pos <= (current_pos + SERVO_THRESHOLD_RANGE)){
				score++;
				sprintf(print_buffer, ": PLAYER SUCCESSFUL!\r\n");
				vPrintString(print_buffer);
			}
			else{
				sprintf(print_buffer, ": PLAYER FAIL!\r\n");
				vPrintString(print_buffer);
			}//end else statement
		}//end for loop
		//Level 1 Results
		sprintf(print_buffer, "Score after level 1: %d\r\n", score);
		vPrintString(print_buffer);
		osDelay(5000);//5 seconds delay for a break

		//Level 2 Inintialization
		sprintf(print_buffer,"Starting Level 2\r\n");
		vPrintString(print_buffer);
		//Rounds 1-5 for Level 2
		for(int i = 0; i<5; i++){
			sprintf(print_buffer, "Starting Round %d", i+1);
			vPrintString(print_buffer);
			int recipe_command = Level_1[i];
			int op = recipe_command & OPCODE_MASK;
			int args = recipe_command & ARGS_MASK;
			parse_recipe(op, args);
			current_pos = FindCurrentPos(args);

			//Level 2 only provides 3 seconds to win
			osDelay(3000);
			//Success!
			if(player_pos >= (current_pos - SERVO_THRESHOLD_RANGE) && player_pos <= (current_pos + SERVO_THRESHOLD_RANGE)){
				score++;
				sprintf(print_buffer, ": PLAYER SUCCESSFUL!\r\n");
				vPrintString(print_buffer);
			}
			//Failure
			else{
				sprintf(print_buffer, ": PLAYER FAIL!\r\n");
				vPrintString(print_buffer);
			}//end else statement
		}//end for loop
		//End of Level 2
		sprintf(print_buffer, "Score after level 2: %d\r\n", score);
		vPrintString(print_buffer);
		osDelay(5000);//5 seconds delay for a break

		//Level 3 Initialization
		sprintf(print_buffer,"Starting Level 3\r\n");
		vPrintString(print_buffer);
		//Rounds 1-5 for Level 3
		for(int i = 0; i<5; i++){
			sprintf(print_buffer, "Starting Round %d", i+1);
			vPrintString(print_buffer);
			int recipe_command = Level_1[i];
			int op = recipe_command & OPCODE_MASK;
			int args = recipe_command & ARGS_MASK;
			parse_recipe(op, args);
			current_pos = FindCurrentPos(args);

			//Level 3 only allows for 1 second to complete
			osDelay(1000);
			//Success!
			if(player_pos >= (current_pos - SERVO_THRESHOLD_RANGE) && player_pos <= (current_pos + SERVO_THRESHOLD_RANGE)){
				score++;
				sprintf(print_buffer, ": PLAYER SUCCESSFUL!\r\n");
				vPrintString(print_buffer);
			}
			//Failure
			else{
				sprintf(print_buffer, ": PLAYER FAIL!\r\n");
				vPrintString(print_buffer);
			}//end else statement
		}//end for loop
		//End of Level 3
		sprintf(print_buffer, "Final Score: %d\r\nGAME OVER\r\n", score);
		vPrintString(print_buffer);
	}

}

/*
 * Random Number Generator using freeRTOS's provided RNG code. Starts at 0.
 *
 * @param max: the max value that a random number can be (range is 0 to max).
 * @param min: the min value that a random number can be (range is 0 to max).
               Used to find true max value for range of numbers.
 * @return: random number generated
 */
int random(int max, int min){
	max = max-min;
	uint32_t random_number;
	HAL_RNG_GenerateRandomNumber(&hrng, &random_number);
	random_number = (random_number%max) + min;

	return (int)random_number;
}
/*
 * Recipe parsing code borrowed from previous projects used for level recipe parsing.
 *
 * @param op: The command provided
 * @param args: Arguments for the command provided
 * @return: void
 */
void parse_recipe (int op, int args){
	switch(op){
			case MOV:
				if(args >= 0 && args <= 5){
					switch(args){
						case 1:
							SetPosition(COMPUTER_SERVO, POS1);
							break;
						case 2:
							SetPosition(COMPUTER_SERVO, POS2);
							break;
						case 3:
							SetPosition(COMPUTER_SERVO, POS3);
							break;
						case 4:
							SetPosition(COMPUTER_SERVO, POS4);
							break;
						case 5:
							SetPosition(COMPUTER_SERVO, POS5);
							break;
						default:
							SetPosition(COMPUTER_SERVO, POS0);
					}
				}
				break;
			default:
				break;
	}
}

/*
 * Used to reference positions.
 * @param arg: position being passed in
 * @return: position being referenced
 */
int FindCurrentPos(int arg){
	switch(arg){
		case 1:
			return POS1;
		case 2:
			return POS2;
		case 3:
			return POS3;
		case 4:
			return POS4;
		case 5:
			return POS5;
		default:
			return POS0;
	}
}

