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
#include <stdio.h>
#include <string.h>
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef enum { false, true } bool;

typedef struct RgbValue
{
	uint8_t Red;
	uint8_t Green;
	uint8_t Blue;
} RgbValue;

typedef struct HsvValue
{
	uint8_t Hue;
	uint8_t Saturation;
	uint8_t Value;
} HsvValue;

typedef struct ColorData
{
	uint8_t Average;
	uint32_t Sum;
	uint32_t Count;
} ColorData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for analyzeRed */
osThreadId_t analyzeRedHandle;
const osThreadAttr_t analyzeRed_attributes = {
  .name = "analyzeRed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for analyzeGreen */
osThreadId_t analyzeGreenHandle;
const osThreadAttr_t analyzeGreen_attributes = {
  .name = "analyzeGreen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for analyzeBlue */
osThreadId_t analyzeBlueHandle;
const osThreadAttr_t analyzeBlue_attributes = {
  .name = "analyzeBlue",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for displayColor */
osThreadId_t displayColorHandle;
const osThreadAttr_t displayColor_attributes = {
  .name = "displayColor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for redQueue */
osMessageQueueId_t redQueueHandle;
uint8_t redQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t redQueueControlBlock;
const osMessageQueueAttr_t redQueue_attributes = {
  .name = "redQueue",
  .cb_mem = &redQueueControlBlock,
  .cb_size = sizeof(redQueueControlBlock),
  .mq_mem = &redQueueBuffer,
  .mq_size = sizeof(redQueueBuffer)
};
/* Definitions for blueQueue */
osMessageQueueId_t blueQueueHandle;
uint8_t blueQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t blueQueueControlBlock;
const osMessageQueueAttr_t blueQueue_attributes = {
  .name = "blueQueue",
  .cb_mem = &blueQueueControlBlock,
  .cb_size = sizeof(blueQueueControlBlock),
  .mq_mem = &blueQueueBuffer,
  .mq_size = sizeof(blueQueueBuffer)
};
/* Definitions for greenQueue */
osMessageQueueId_t greenQueueHandle;
uint8_t greenQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t greenQueueControlBlock;
const osMessageQueueAttr_t greenQueue_attributes = {
  .name = "greenQueue",
  .cb_mem = &greenQueueControlBlock,
  .cb_size = sizeof(greenQueueControlBlock),
  .mq_mem = &greenQueueBuffer,
  .mq_size = sizeof(greenQueueBuffer)
};
/* USER CODE BEGIN PV */
#define ADC_BUFFER_SIZE (30)
const uint8_t QUEUE_SIZE = 16;
const uint16_t MAX_ADC_INPUT = 4095;
volatile uint32_t adc_values[ADC_BUFFER_SIZE];
bool IsRgb = true;
RgbValue CurrentColor;
ColorData BlueColorData;
ColorData RedColorData;
ColorData GreenColorData;
StaticTimer_t PrintStatsTimerControlBlock;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void AnalyzeRedTask(void *argument);
void AnalyzeGreenTask(void *argument);
void AnalyzeBlueTask(void *argument);
void DisplayColorTask(void *argument);

/* USER CODE BEGIN PFP */
uint8_t ConvertToByte(int16_t value);
void AnalyzeColor(ColorData *data, osMessageQueueId_t *queue, void (*mapper)(uint8_t), char *colorName);
void SetPWMDutyCycle(uint8_t dutyCycle, TIM_HandleTypeDef *timerHandle, uint16_t channel);
void SafeQueuePut(osMessageQueueId_t *queue, const osMessageQueueAttr_t *queueAttrs, uint16_t *msg);
void AssignRed(uint8_t value);
void AssignGreen(uint8_t value);
void AssignBlue(uint8_t value);
HsvValue RgbToHsv(RgbValue *color);
void PrintStatsCallback(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t ConvertToByte(int16_t value)
{
	return (value * UINT8_MAX) / MAX_ADC_INPUT;
}

void AnalyzeColor(ColorData *data, osMessageQueueId_t *queue, void (*mapper)(uint8_t), char *colorName)
{
	uint32_t value;
  	uint32_t messageCount = osMessageQueueGetCount(*queue);
  	uint8_t convertedValue;
  	char messageBuffer[32] = {};
	if (messageCount > 0 && messageCount <= QUEUE_SIZE)
	{
		  for (uint8_t i = 0; i < messageCount; i++)
		  {
			  osMessageQueueGet(*queue, &value, NULL, 0);
			  convertedValue = ConvertToByte(value);
			  data -> Sum += convertedValue;
			  data -> Count++;
			  data -> Average = data -> Sum / data -> Count;
		  }

		  mapper(convertedValue);
		  if (sprintf(messageBuffer, "Value of %s: %i -> %i \r\n", colorName, (int)value, convertedValue) >= 0)
			  HAL_UART_Transmit(&huart2, (uint8_t*)messageBuffer, strlen((char*)messageBuffer), HAL_MAX_DELAY);

	}
}

void SetPWMDutyCycle(uint8_t dutyCycle, TIM_HandleTypeDef *timerHandle, uint16_t channel)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = dutyCycle;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(timerHandle, &sConfigOC, channel);
    HAL_TIM_PWM_Start(timerHandle, channel);
}

void SafeQueuePut(osMessageQueueId_t *queue, const osMessageQueueAttr_t *queueAttrs, uint16_t *msg)
{
  	uint32_t capacity = osMessageQueueGetCapacity(*queue);
  	if (capacity == QUEUE_SIZE)
  	{
  		if (osMessageQueueGetSpace(*queue) > 0)
  			osMessageQueuePut(*queue, msg, 0, 0);
  	}
  	else
  	{
  		osMessageQueueDelete(*queue);
  		*queue = osMessageQueueNew(QUEUE_SIZE, sizeof(uint16_t), queueAttrs);
  	}
}

void AssignRed(uint8_t value)
{
	CurrentColor.Red = value;
}

void AssignGreen(uint8_t value)
{
	CurrentColor.Green = value;
}

void AssignBlue(uint8_t value)
{
	CurrentColor.Blue = value;
}

HsvValue RgbToHsv(RgbValue *color) //this function is chat gpt genrated
{
    uint8_t max = color -> Red > color -> Green ? (color -> Red > color -> Blue ? color -> Red : color -> Blue) : (color -> Green > color -> Blue ? color -> Green : color -> Blue);
    uint8_t min = color -> Red < color -> Green ? (color -> Red < color -> Blue ? color -> Red : color -> Blue) : (color -> Green < color -> Blue ? color -> Green : color -> Blue);
    uint8_t delta = max - min;

    uint8_t v = max;
    uint8_t h, s;
    if (max != 0)
        s = 255 * (long) delta / max;
    else {
        s = 0;
        h = 0;
        return (HsvValue){ h, s, v };
    }
    if (delta == 0) {
        h = 0;
        return (HsvValue){ h, s, v };
    }

    if (color -> Red == max)
        h = 43 * (color -> Green - color -> Blue) / delta;
    else if (color -> Green == max)
        h = 85 + 43 * (color -> Blue - color -> Red) / delta;
    else
        h = 171 + 43 * (color -> Red - color -> Green) / delta;

    if (h < 0)
        h += 255;

    return (HsvValue){ h, s, v };
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  NVIC_SetPriorityGrouping(0);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  TimerHandle_t hTimer = xTimerCreateStatic("printStatsTimer", 5000, pdTRUE, (void*)0, PrintStatsCallback, &PrintStatsTimerControlBlock);
  xTimerStart(hTimer, portMAX_DELAY);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of redQueue */
  redQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &redQueue_attributes);

  /* creation of blueQueue */
  blueQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &blueQueue_attributes);

  /* creation of greenQueue */
  greenQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &greenQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of analyzeRed */
  analyzeRedHandle = osThreadNew(AnalyzeRedTask, NULL, &analyzeRed_attributes);

  /* creation of analyzeGreen */
  analyzeGreenHandle = osThreadNew(AnalyzeGreenTask, NULL, &analyzeGreen_attributes);

  /* creation of analyzeBlue */
  analyzeBlueHandle = osThreadNew(AnalyzeBlueTask, NULL, &analyzeBlue_attributes);

  /* creation of displayColor */
  displayColorHandle = osThreadNew(DisplayColorTask, NULL, &displayColor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_BUFFER_SIZE);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		IsRgb = !IsRgb;
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint16_t r = adc_values[0];
	uint16_t g = adc_values[2];
	uint16_t b = adc_values[1];
	if (r > MAX_ADC_INPUT || g > MAX_ADC_INPUT || b > MAX_ADC_INPUT)
		return;

	SafeQueuePut(&redQueueHandle, &redQueue_attributes, &r);
	SafeQueuePut(&greenQueueHandle, &greenQueue_attributes, &g);
	SafeQueuePut(&blueQueueHandle, &blueQueue_attributes, &b);
}

void PrintStatsCallback(void *argument)
{
  char messageBuffer[64] = {};
  if (sprintf(messageBuffer, "Avg - red: %u, green: %u, blue: %u \r\n", RedColorData.Average, GreenColorData.Average, BlueColorData.Average) >= 0)
	  HAL_UART_Transmit(&huart2, (uint8_t*)messageBuffer, strlen((char*)messageBuffer), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_AnalyzeRedTask */
/**
  * @brief  Function implementing the analyzeRed thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AnalyzeRedTask */
void AnalyzeRedTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  AnalyzeColor(&RedColorData, &redQueueHandle, &AssignRed, "red");
	  osDelay(1);
  }
  osThreadTerminate(analyzeRedHandle);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_AnalyzeGreenTask */
/**
* @brief Function implementing the analyzeGreen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AnalyzeGreenTask */
void AnalyzeGreenTask(void *argument)
{
  /* USER CODE BEGIN AnalyzeGreenTask */
  /* Infinite loop */
  for(;;)
  {
	  AnalyzeColor(&GreenColorData, &greenQueueHandle, &AssignGreen, "green");
	  osDelay(1);
  }

  osThreadTerminate(analyzeGreenHandle);
  /* USER CODE END AnalyzeGreenTask */
}

/* USER CODE BEGIN Header_AnalyzeBlueTask */
/**
* @brief Function implementing the analyzeBlue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AnalyzeBlueTask */
void AnalyzeBlueTask(void *argument)
{
  /* USER CODE BEGIN AnalyzeBlueTask */
  /* Infinite loop */
  for(;;)
  {
	  AnalyzeColor(&BlueColorData, &blueQueueHandle, &AssignBlue, "blue");
	  osDelay(1);
  }
  osThreadTerminate(analyzeBlueHandle);
  /* USER CODE END AnalyzeBlueTask */
}

/* USER CODE BEGIN Header_DisplayColorTask */
/**
* @brief Function implementing the displayColor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DisplayColorTask */
void DisplayColorTask(void *argument)
{
  /* USER CODE BEGIN DisplayColorTask */
  /* Infinite loop */
  uint8_t r, g, b;
  for(;;)
  {
	r = CurrentColor.Red;
	g = CurrentColor.Green;
	b = CurrentColor.Blue;
	if (!IsRgb)
	{
		HsvValue hsvColor = RgbToHsv(&CurrentColor);
		r = hsvColor.Hue;
		g = hsvColor.Saturation;
		b = hsvColor.Value;
	}
	SetPWMDutyCycle(r, &htim4, TIM_CHANNEL_1);
	SetPWMDutyCycle(g, &htim2, TIM_CHANNEL_3);
	SetPWMDutyCycle(b, &htim3, TIM_CHANNEL_1);
    osDelay(1);
  }
  osThreadTerminate(displayColorHandle);
  /* USER CODE END DisplayColorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
