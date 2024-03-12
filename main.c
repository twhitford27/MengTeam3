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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#define NS 250


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t PushState = 0;
volatile uint16_t FSimCount = 0;
volatile uint16_t FSIMFLAG = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
uint16_t FrequencySimARR(uint16_t input);
uint16_t get_Freq_ARR_Value_Large(uint16_t input);
void FrequencySim(uint16_t t_step);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint32_t AD_RES = 0;
	uint16_t Default_ARR = 6695;
	uint32_t Wave_LUT[NS] = {
			2048, 2099, 2151, 2202, 2254, 2305, 2356, 2407, 2458, 2509, 2559, 2609, 2658, 2707, 2756,
			2804, 2852, 2899, 2946, 2992, 3038, 3082, 3127, 3170, 3213, 3255, 3296, 3337, 3377, 3416,
			3454, 3491, 3527, 3562, 3596, 3630, 3662, 3693, 3723, 3753, 3781, 3808, 3833, 3858, 3882,
			3904, 3925, 3945, 3964, 3982, 3998, 4013, 4027, 4039, 4051, 4061, 4070, 4077, 4083, 4088,
			4092, 4094, 4095, 4095, 4093, 4090, 4086, 4080, 4073, 4065, 4056, 4045, 4033, 4020, 4006,
			3990, 3973, 3955, 3935, 3915, 3893, 3870, 3846, 3821, 3794, 3767, 3738, 3709, 3678, 3646,
			3613, 3579, 3545, 3509, 3472, 3435, 3396, 3357, 3317, 3276, 3234, 3192, 3149, 3105, 3060,
			3015, 2969, 2923, 2876, 2828, 2780, 2732, 2683, 2633, 2584, 2534, 2483, 2433, 2382, 2331,
			2279, 2228, 2177, 2125, 2073, 2022, 1970, 1918, 1867, 1816, 1764, 1713, 1662, 1612, 1561,
			1511, 1462, 1412, 1363, 1315, 1267, 1219, 1172, 1126, 1080, 1035, 990, 946, 903, 861,
			819, 778, 738, 699, 660, 623, 586, 550, 516, 482, 449, 417, 386, 357, 328,
			301, 274, 249, 225, 202, 180, 160, 140, 122, 105, 89, 75, 62, 50, 39,
			30, 22, 15, 9, 5, 2, 0, 0, 1, 3, 7, 12, 18, 25, 34,
			44, 56, 68, 82, 97, 113, 131, 150, 170, 191, 213, 237, 262, 287, 314,
			342, 372, 402, 433, 465, 499, 533, 568, 604, 641, 679, 718, 758, 799, 840,
			882, 925, 968, 1013, 1057, 1103, 1149, 1196, 1243, 1291, 1339, 1388, 1437, 1486, 1536,
			1586, 1637, 1688, 1739, 1790, 1841, 1893, 1944, 1996, 2047
	};
	uint32_t SINE_LUT[NS] = {
			2048, 2099, 2151, 2202, 2254, 2305, 2356, 2407, 2458, 2509, 2559, 2609, 2658, 2707, 2756,
			2804, 2852, 2899, 2946, 2992, 3038, 3082, 3127, 3170, 3213, 3255, 3296, 3337, 3377, 3416,
			3454, 3491, 3527, 3562, 3596, 3630, 3662, 3693, 3723, 3753, 3781, 3808, 3833, 3858, 3882,
			3904, 3925, 3945, 3964, 3982, 3998, 4013, 4027, 4039, 4051, 4061, 4070, 4077, 4083, 4088,
			4092, 4094, 4095, 4095, 4093, 4090, 4086, 4080, 4073, 4065, 4056, 4045, 4033, 4020, 4006,
			3990, 3973, 3955, 3935, 3915, 3893, 3870, 3846, 3821, 3794, 3767, 3738, 3709, 3678, 3646,
			3613, 3579, 3545, 3509, 3472, 3435, 3396, 3357, 3317, 3276, 3234, 3192, 3149, 3105, 3060,
			3015, 2969, 2923, 2876, 2828, 2780, 2732, 2683, 2633, 2584, 2534, 2483, 2433, 2382, 2331,
			2279, 2228, 2177, 2125, 2073, 2022, 1970, 1918, 1867, 1816, 1764, 1713, 1662, 1612, 1561,
			1511, 1462, 1412, 1363, 1315, 1267, 1219, 1172, 1126, 1080, 1035, 990, 946, 903, 861,
			819, 778, 738, 699, 660, 623, 586, 550, 516, 482, 449, 417, 386, 357, 328,
			301, 274, 249, 225, 202, 180, 160, 140, 122, 105, 89, 75, 62, 50, 39,
			30, 22, 15, 9, 5, 2, 0, 0, 1, 3, 7, 12, 18, 25, 34,
			44, 56, 68, 82, 97, 113, 131, 150, 170, 191, 213, 237, 262, 287, 314,
			342, 372, 402, 433, 465, 499, 533, 568, 604, 641, 679, 718, 758, 799, 840,
			882, 925, 968, 1013, 1057, 1103, 1149, 1196, 1243, 1291, 1339, 1388, 1437, 1486, 1536,
			1586, 1637, 1688, 1739, 1790, 1841, 1893, 1944, 1996, 2047
	};
	uint32_t Noisy_LUT[NS] = {
			2061, 2085, 2143, 2189, 2266, 2289, 2335, 2419, 2449, 2516, 2551, 2614, 2606, 2717, 2794,
			2825, 2871, 2894, 2950, 2997, 3039, 3065, 3119, 3167, 3203, 3272, 3348, 3310, 3379, 3386,
			3480, 3520, 3493, 3602, 3574, 3634, 3685, 3696, 3770, 3809, 3784, 3769, 3826, 3841, 3866,
			3881, 3927, 3988, 3949, 3976, 4022, 4038, 4037, 4060, 4069, 4053, 4078, 4071, 4065, 4071,
			4093, 4090, 4092, 4091, 4095, 4094, 4092, 4081, 4058, 4081, 4085, 4034, 4073, 4017, 4001,
			3972, 3957, 3935, 3943, 3948, 3904, 3888, 3873, 3770, 3791, 3774, 3753, 3682, 3657, 3662,
			3611, 3591, 3525, 3475, 3488, 3459, 3430, 3326, 3289, 3246, 3233, 3179, 3175, 3075, 3024,
			3019, 2993, 2906, 2850, 2825, 2747, 2732, 2700, 2638, 2545, 2523, 2477, 2470, 2401, 2330,
			2306, 2207, 2169, 2154, 2104, 2037, 1980, 1906, 1882, 1799, 1776, 1719, 1686, 1603, 1574,
			1528, 1443, 1416, 1396, 1317, 1261, 1229, 1167, 1135, 1077, 1035, 1000, 974, 913, 895,
			778, 769, 743, 682, 634, 635, 599, 556, 524, 464, 439, 415, 372, 364, 377,
			291, 288, 228, 252, 182, 184, 147, 151, 122, 104, 150, 62, 61, 105, 16,
			41, 34, 36, 16, 18, 4, 5, 15, 46, 30, 15, 2, 12, 5, 16,
			36, 41, 50, 73, 78, 141, 111, 187, 162, 161, 201, 256, 283, 291, 320,
			355, 342, 390, 396, 456, 518, 548, 615, 608, 597, 714, 745, 746, 803, 856,
			890, 939, 966, 1012, 1059, 1087, 1178, 1196, 1257, 1273, 1317, 1386, 1432, 1511, 1549,
			1598, 1607, 1668, 1743, 1783, 1868, 1920, 1932, 2014, 2061
	};
	uint32_t Phase_LUT[NS] = {
			2048, 2099, 2151, 2202, 2254, 2305, 2356, 2407, 2458, 2509, 2559, 2609, 2658, 2707, 2756,
			2804, 2852, 2899, 2946, 2992, 3038, 3082, 3127, 3170, 3213, 3255, 3296, 3337, 3377, 3416,
			3454, 3491, 3527, 3562, 3596, 3630, 3662, 3693, 3723, 3980, 3996, 4012, 4026, 4038, 4050,
			4060, 4069, 4076, 4083, 4088, 4091, 4094, 4095, 4095, 4093, 4090, 4086, 4081, 4074, 4066,
			4057, 4046, 4035, 4021, 4007, 3991, 3975, 3957, 3937, 3917, 3895, 3872, 3848, 3823, 3797,
			3769, 3741, 3711, 3681, 3649, 3616, 3583, 3548, 3512, 3476, 3438, 3400, 3361, 3321, 3280,
			3238, 3196, 3153, 3109, 3064, 3019, 2973, 2927, 2880, 2832, 2785, 2736, 2687, 2638, 2588,
			2538, 2488, 2437, 2387, 2335, 2284, 2233, 2181, 2130, 2078, 2026, 1975, 1923, 1872, 1820,
			1769, 1718, 1667, 1616, 1566, 1516, 1466, 1417, 1368, 1319, 1271, 1224, 1177, 1130, 1084,
			1039, 994, 951, 907, 865, 823, 782, 742, 702, 664, 626, 589, 554, 519, 485,
			452, 420, 389, 360, 331, 303, 277, 251, 227, 204, 182, 161, 142, 124, 107,
			91, 76, 63, 51, 40, 30, 22, 15, 10, 5, 2, 0, 0, 1, 3,
			6, 11, 17, 25, 33, 43, 54, 67, 81, 96, 112, 129, 148, 168, 189,
			211, 235, 259, 285, 312, 340, 369, 399, 430, 462, 495, 530, 565, 601, 638,
			676, 715, 754, 795, 836, 878, 921, 964, 1008, 1053, 1099, 1145, 1192, 1239, 1286,
			1335, 1383, 1432, 1482, 1532, 1582, 1632, 1683, 1734, 1785, 1836, 1888, 1939, 1991, 2043,
			2094, 2146, 2198, 2249, 2300, 2352, 2403, 2453, 2504, 2554,
	};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t old_state = PushState;
	uint8_t txbuf[64];
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
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Wave_LUT, 250, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (old_state != PushState){
		  HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);
		  //HAL_TIM_Base_Stop_IT(&htim11);
		  TIM2->ARR = 6720-1;
		  FSimCount = 0;
		  old_state = PushState;
		  switch(PushState){

		  case 0://Clean signal
			  TIM2->ARR = 6720-1; //sets the frequency to 50Hz
			  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1, (uint32_t*)SINE_LUT,250,DAC_ALIGN_12B_R); //outputs the sine wave period
			  break;

		  case 1:
			  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1, (uint32_t*)SINE_LUT,250,DAC_ALIGN_12B_R); // output sine wave
			  FrequencySim(1000);
			  PushState = 0;

		  case 2:
			  TIM2->ARR = 6720-1;
			  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)Noisy_LUT,250,DAC_ALIGN_12B_R);// output noisy Waveform

		  case 3:
			  TIM2->ARR = 6720-1;
			  HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)Phase_LUT,250, DAC_ALIGN_12B_R);
		  }
	  }
	  //HAL_ADC_Start_DMA(&hadc1,&AD_RES,1);
	  //HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	  HAL_Delay(100);
	  sprintf((char*)txbuf, "Returned: %d \r\n", PushState );
	  HAL_UART_Transmit(&huart2, txbuf, strlen((char*)txbuf), HAL_MAX_DELAY);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6720 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 84 - 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000 - 1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin PC6 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = B1_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_13){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		PushState = 0;

	}else if (GPIO_Pin == GPIO_PIN_9){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		PushState = 1;
	}else if (GPIO_Pin == GPIO_PIN_8){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		PushState = 2;
	}else if (GPIO_Pin == GPIO_PIN_7){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		PushState = 3;
	}else if (GPIO_Pin == GPIO_PIN_6){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		PushState = 4;
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Conversion Complete & DMA Transfer Complete As Well
	if(FSIMFLAG == 0){
		uint16_t raw = AD_RES;
		uint16_t outmsg = get_Freq_ARR_Value_Large(raw);
		char msg[10];
		sprintf(msg, "%d\r\n", outmsg);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		TIM2->ARR = outmsg;
	}

}
uint16_t get_Freq_ARR_Value_Large(uint16_t input){
	uint16_t i = 0;
	uint16_t output = 0;
	struct{
		uint16_t inputvalue;
		uint16_t outputvalue;
	}list[]={
			{512,7636 - 1}, //44Hz
			{1024,7466 - 1}, //45Hz
			{1536,7304 - 1}, //46Hz
			{2048,7149 - 1},//47Hz
			{2560,7000 -1},//48Hz
			{3072,6857 - 1},//49Hz
			{3584,6695 - 1},//50Hz
			{4096,6588 - 1}//51Hz

	};
	for (i = 0; i<7 && input >= list[i].inputvalue;i++);
	output = list[i].outputvalue;
	return output;
}
void FrequencySim(uint16_t t_step){
	uint16_t i = 0;

	uint16_t F[] = {6746 - 1,
			6774 - 1,
			6733 - 1,
			7000 - 1,
			6695
	};
	uint16_t size = sizeof(F)/sizeof(F[0]);
	for (i = 0; i < size - 1;i++){
		TIM2->ARR = F[i];
		HAL_Delay(t_step);
	}

}

/* USER CODE END 4 */

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
