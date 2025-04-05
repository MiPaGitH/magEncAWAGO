/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OTPDATA_bits31_16_ZeroPosition 0x00u /*(ZeroHI and LO - bits 31..16; )*/

//Inc Resolution Bits = 1024 CPR (bits 0,1,2)
//Absolute resolution = 12 bits (bit 8)
//Incremental select bit 10 remains 0 - ABI mode
//Incremental enable (bit 11)
#define OTPDATA_bits_15_0_ModeHI_and_LO (1u<<11) + (1u<<8) + 0x07u

#define NB_OF_CLK_EDGES 71

enum
{
	firstRead,
	write,
	secondRead,
	maxState
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
//values for activating the ABI interface
uint8_t otpData[3][2][NB_OF_CLK_EDGES+2] =
 {
  /*read*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          0,0, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,        1, 1, 1, 0, 0 }},

   /*write*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          1,1, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1, 0,0,0,0,0,0,0,0, 0,0,1,1,1,1,1,1,        1, 1, 0, 0, 0 }},

   /*second read*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          0,0, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,        1, 1, 1, 0, 0 }},
 };
uint32_t inputData;
uint32_t inputDataFirstRead;
uint32_t inputDataSecondRead;


uint32_t evenParBit = 1u; //because there are 5 bits with value 1 in otpData; even parity means that total number of bits with value 1 must be even

uint32_t cntClkRisingEdge;

uint8_t mData[2];
uint16_t mData16;

uint32_t tim1Tick;
uint32_t mSysTick;
uint32_t oldTick;
uint32_t oldBtn;

uint8_t csPinState;
uint8_t dataPinState;
uint8_t clkPinState;
uint8_t pState;
uint8_t oldpState;

uint8_t globState;
uint8_t doInputMode;

uint8_t inputParBit;
uint8_t inputParBitFirstRead;
uint8_t inputParBitSecondRead;
uint8_t evTim1Tick;

uint8_t pStBuf[255];
uint8_t cPStateBuf;

uint8_t *cOtpClk;
uint8_t *cOtpData;


volatile uint8_t btnPressed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
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
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
	cOtpClk = &otpData[0][0][0];
	cOtpData = &otpData[0][1][0];

//	uint8_t fSetPin2InputMode = 0u;
	oldTick = 0u;
	oldBtn = 10000u;
	csPinState = GPIO_PIN_SET;
	dataPinState = GPIO_PIN_SET;
	clkPinState = GPIO_PIN_SET;
	cntClkRisingEdge = 0u;
	btnPressed = 0u;
	doInputMode = 0u;
	evTim1Tick = 0u;

	pState = 0u;
	oldpState = 0u;
	globState = firstRead;

	cPStateBuf = 0u;

	mData[0]=0u;
	mData[1]=0u;
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
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin|ssiClk_Pin|ssiDO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t nBtn = GPIO_PIN_RESET;
	  nBtn = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
	  mSysTick = HAL_GetTick();
	  if ( (oldTick!=mSysTick) && (0u == (mSysTick%25)) )
	  {
		  mData16 = ((((uint16_t)mData[1]&0x3F)<<8) + mData[0])>>4;
		  //HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_SET);
		  oldTick = mSysTick;
		  //start data transfer
		  HAL_SPI_Receive(&hspi2, &mData[0], 1u, 1000u);
	  }


	  if ( ( GPIO_PIN_RESET == oldBtn) && (GPIO_PIN_SET== nBtn ) )
	  {
		  oldBtn = 100000u;
		  if ( globState < maxState )
		  {
			  cOtpClk = &otpData[globState][0][0];
			  cOtpData = &otpData[globState][1][0];

			  globState++;
			  HAL_TIM_Base_Start_IT(&htim1);
		  }

	  }
	  if ( GPIO_PIN_RESET == nBtn ) //deBounce
	  {
		  if ( oldBtn>0u) oldBtn--;
	  }

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin|ssiClk_Pin|ssiDO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|testPin_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ssiNCS_Pin */
  GPIO_InitStruct.Pin = ssiNCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ssiNCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ssiClk_Pin */
  GPIO_InitStruct.Pin = ssiClk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ssiClk_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ssiDO_Pin */
  GPIO_InitStruct.Pin = ssiDO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ssiDO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : testPin_Pin */
  GPIO_InitStruct.Pin = testPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(testPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(testPin_GPIO_Port, testPin_Pin);
//	if ( pState != 0u )
//	{
	  HAL_GPIO_WritePin(GPIOA, ssiClk_Pin, cOtpClk[tim1Tick]);
	  HAL_GPIO_WritePin(GPIOA, ssiDO_Pin, cOtpData[tim1Tick]);
	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, dataPinState);
//	}

	if (tim1Tick< NB_OF_CLK_EDGES)
	{
		tim1Tick++;
	}
	else
	{
		tim1Tick=0u;
		HAL_TIM_Base_Stop_IT(&htim1);
//		HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_SET);
	}
//	evTim1Tick = 1u;
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
