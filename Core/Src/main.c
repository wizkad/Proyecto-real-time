/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void delay_us(uint16_t us);
uint32_t TiempoaFrecuencia(uint32_t us);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UARTCommand(char *comando);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LecturaPin */
osThreadId_t LecturaPinHandle;
const osThreadAttr_t LecturaPin_attributes = {
  .name = "LecturaPin",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ultrasound */
osThreadId_t ultrasoundHandle;
const osThreadAttr_t ultrasound_attributes = {
  .name = "ultrasound",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for RecibirUART */
osThreadId_t RecibirUARTHandle;
const osThreadAttr_t RecibirUART_attributes = {
  .name = "RecibirUART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for frecuenciaUltrasonido */
osMessageQueueId_t frecuenciaUltrasonidoHandle;
const osMessageQueueAttr_t frecuenciaUltrasonido_attributes = {
  .name = "frecuenciaUltrasonido"
};
/* Definitions for frecuenciaConstante */
osMessageQueueId_t frecuenciaConstanteHandle;
const osMessageQueueAttr_t frecuenciaConstante_attributes = {
  .name = "frecuenciaConstante"
};
/* Definitions for frecuenciaLog */
osMessageQueueId_t frecuenciaLogHandle;
const osMessageQueueAttr_t frecuenciaLog_attributes = {
  .name = "frecuenciaLog"
};
/* Definitions for Mutex */
osMutexId_t MutexHandle;
const osMutexAttr_t Mutex_attributes = {
  .name = "Mutex"
};
/* Definitions for PinPolling */
osEventFlagsId_t PinPollingHandle;
const osEventFlagsAttr_t PinPolling_attributes = {
  .name = "PinPolling"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void LecturaPines(void *argument);
void Ultrasound(void *argument);
void BuzzerA(void *argument);
void Recibir_UART(void *argument);

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Mutex */
  MutexHandle = osMutexNew(&Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of frecuenciaUltrasonido */
  frecuenciaUltrasonidoHandle = osMessageQueueNew (16, sizeof(uint16_t), &frecuenciaUltrasonido_attributes);

  /* creation of frecuenciaConstante */
  frecuenciaConstanteHandle = osMessageQueueNew (16, sizeof(uint16_t), &frecuenciaConstante_attributes);

  /* creation of frecuenciaLog */
  frecuenciaLogHandle = osMessageQueueNew (16, sizeof(uint16_t), &frecuenciaLog_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LecturaPin */
  LecturaPinHandle = osThreadNew(LecturaPines, NULL, &LecturaPin_attributes);

  /* creation of ultrasound */
  ultrasoundHandle = osThreadNew(Ultrasound, NULL, &ultrasound_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(BuzzerA, NULL, &Buzzer_attributes);

  /* creation of RecibirUART */
  RecibirUARTHandle = osThreadNew(Recibir_UART, NULL, &RecibirUART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of PinPolling */
  PinPollingHandle = osEventFlagsNew(&PinPolling_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  osEventFlagsSet(PinPollingHandle,0x0002U);
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Trigger_ultrasonido_GPIO_Port, Trigger_ultrasonido_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo_ultrasonido_Pin */
  GPIO_InitStruct.Pin = Echo_ultrasonido_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo_ultrasonido_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_ultrasonido_Pin */
  GPIO_InitStruct.Pin = Trigger_ultrasonido_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Trigger_ultrasonido_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t timcounter;
#define TIME_MIN_US 66    // Tiempo mínimo (2cm)
#define TIME_MAX_US 200  // Tiempo máximo (4m)
#define FREQ_MIN_HZ 2000    // Frecuencia mínima (2 kHz)
#define FREQ_MAX_HZ 5000    // Frecuencia máxima (5 kHz)

//Variables para la transmision UART
#define BUFFER_SIZE 20   // Maximum size of the command buffer
char rxBuffer[1];           // Buffer for single-byte UART reception
char Buffer[20];  // Buffer to store the command
uint16_t cmdIndice = 0;      // Indice para controlar los datos
uint16_t FrecuenciaBuzzer = 2000;  // Frecuencia(Hz)
uint8_t EntradaBuzzer = 1;         // Controla la entrada del buzzer
uint16_t FrecuenciaMandar;  // Frecuencia(Hz)


//Funcion de generacion de frecuencias para el buzzer
uint32_t TiempoaFrecuencia(uint32_t us) {
    // Limitar el tiempo a los valores mínimos y máximos
    if (us < TIME_MIN_US) us = TIME_MIN_US;
    if (us > TIME_MAX_US) us = TIME_MAX_US;

    // Aplicar la fórmula de interpolación lineal
    uint32_t frecuencia = FREQ_MIN_HZ + (us - TIME_MIN_US) * (FREQ_MAX_HZ - FREQ_MIN_HZ) / (TIME_MAX_US - TIME_MIN_US);

    return frecuencia;
}


void delay_us(uint16_t us)
{
	HAL_TIM_Base_Start_IT(&htim1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while(timcounter<us);

    HAL_TIM_Base_Stop_IT(&htim1);
    timcounter = 0;
}

/*Callback TIM*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM1){
	timcounter++;
	}

	else {
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
	}
}

//Callback USART2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
	        char rxChar = rxBuffer[0];

	        if (rxChar == '\r') {
	            //Añade el fin de string
	            Buffer[cmdIndice] = '\0';

	            // Procesa el comando
	            UARTCommand(Buffer);

	            // Reinicia el indice
	            cmdIndice = 0;
	            memset(Buffer, 0, BUFFER_SIZE);
	        } else if (cmdIndice < BUFFER_SIZE - 1) {
	            // Añade el char al siguiente espacio
	            Buffer[cmdIndice++] = rxChar;
	        } else {
	            // Buffer lleno
	            cmdIndice = 0;  // Reset the buffer
	        }

	        // Reinicia la conexion
	        HAL_UART_Receive_IT(&huart2, (uint8_t *)rxBuffer, 1);
	    }
}

void UARTCommand(char *comando)
{
	char buffer[30];
	uint16_t frecuencia;
	 if (strncmp(comando, "SRC", 3) == 0) {
		 //Comando recibido: SRC
		 EntradaBuzzer = atoi(comando + 4);
		 memset(buffer, 0, sizeof(buffer));
		 sprintf(buffer,"Entrada cambiada: %d\n",EntradaBuzzer);
		 HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer), 10);

	 } else if (strncmp(comando, "FREQ", 4) == 0) {
		 frecuencia = atoi(comando + 5);  // Extrae la frecuencia
		 EntradaBuzzer = 2;
		 if((frecuencia>=2000) && (frecuencia<=5000)){
			 FrecuenciaBuzzer = frecuencia;
			 memset(buffer, 0, sizeof(buffer));
             sprintf(buffer,"Frecuencia: %d\n", frecuencia);
			 HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer), 10);
			 osMessageQueuePut(frecuenciaConstanteHandle,&frecuencia,0,0);
		 } else {
			 memset(buffer, 0, sizeof(buffer));
			 sprintf(buffer,"Frecuencia no valida\n");
			 HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer), 10);
		 }
	 }

		 else if (strncmp(comando, "frecuencia", 10) == 0) {
			 memset(buffer, 0, sizeof(buffer));
			 sprintf(buffer,"#%d#", FrecuenciaMandar);
			 HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer), 10);
	 } else {
		 memset(buffer, 0, sizeof(buffer));
		 sprintf(buffer,"Comando no valido\n");
		 HAL_UART_Transmit(&huart2,(uint8_t*)buffer,sizeof(buffer), 10);
	 }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LecturaPines */
/**
* @brief Function implementing the LecturaPulsador thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LecturaPines */
void LecturaPines(void *argument)
{
  /* USER CODE BEGIN LecturaPines */
	uint8_t pulso;
	uint8_t PinEstado = 0;
  /* Infinite loop */
  for(;;)
  {
	PinEstado = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
	  if(PinEstado == 1){
		  __HAL_TIM_SET_COUNTER(&htim1, 0);
		  HAL_TIM_Base_Start_IT(&htim1);
	  }else{
	  	  HAL_TIM_Base_Stop_IT(&htim1);
	  	  pulso = timcounter;
	  	  osEventFlagsSet(PinPollingHandle,0x0002U);
	  }
	  if(pulso != 0){
		  uint16_t frecuencia = TiempoaFrecuencia(pulso);
		  FrecuenciaMandar = frecuencia;
		  osMessageQueuePut(frecuenciaUltrasonidoHandle,&frecuencia,0,0);
	  }
	  osDelay(1);
  }
  /* USER CODE END LecturaPines */
}

/* USER CODE BEGIN Header_Ultrasound */
/**
* @brief Function implementing the ultrasound thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ultrasound */
void Ultrasound(void *argument)
{
  /* USER CODE BEGIN Ultrasound */
	/* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(PinPollingHandle,0x0002U,0,osWaitForever);
	  /*Reconfigura el pin como output */
	  timcounter = 0;

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
	  /* funcion de delay de 5us*/
	  delay_us(10);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);

    osDelay(1);
  }
  /* USER CODE END Ultrasound */
}

/* USER CODE BEGIN Header_BuzzerA */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BuzzerA */
void BuzzerA(void *argument)
{
  /* USER CODE BEGIN BuzzerA */
	uint16_t frecuencia;
  /* Infinite loop */
  for(;;)
  {
	  if(EntradaBuzzer == 1){
	  		  osMessageQueueGet(frecuenciaUltrasonidoHandle,&frecuencia,NULL,osWaitForever);}
	  if(EntradaBuzzer == 2){
	  		  osMessageQueueGet(frecuenciaConstanteHandle,&frecuencia,NULL,osWaitForever);}
	  //if(EntradaBuzzer == 3){
	  	//osMessageQueueGet(frecuenciaLogHandle,&frecuencia,NULL,osWaitForever);}

	  HAL_TIM_Base_Stop_IT(&htim3);
	  __HAL_TIM_SET_COUNTER(&htim3, 0);

	  uint32_t timerPeripherals = HAL_RCC_GetPCLK1Freq(); // Frecuencia del reloj del temporizador
	  uint32_t timerClock = timerPeripherals*2;
	  uint32_t arr = 1000; // Mantén ARR fijo en este caso para un duty cycle del 0-100%
	  uint32_t prescaler = (timerClock / (frecuencia * arr)) - 1; // Ajustar prescaler


	  __HAL_TIM_SET_PRESCALER(&htim3, prescaler); // Cambia el prescaler
	  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);     // Cambia el ARR

	      HAL_TIM_Base_Start_IT(&htim3);        // Reinicia el Timer3

    osDelay(1);
  }
  /* USER CODE END BuzzerA */
}

/* USER CODE BEGIN Header_Recibir_UART */
/**
* @brief Function implementing the RecibirUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Recibir_UART */
void Recibir_UART(void *argument)
{
  /* USER CODE BEGIN Recibir_UART */
  /* Infinite loop */
  for(;;)
  {
	HAL_UART_Receive_IT(&huart2, (uint8_t*)rxBuffer, 1);
    osDelay(1);
  }
  /* USER CODE END Recibir_UART */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
