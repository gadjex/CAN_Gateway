/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

uint32_t allowedIds[] = { 0x12F8BFA7, 0x12F8BE9F };
int allowedIdsSize;

//char uartBuffer[20] = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  allowedIdsSize = sizeof allowedIds / sizeof allowedIds[0];
  char uartBuffer[] = "INIT\n";
  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  CAN_RX_STATUS rxStatus = recvCanMsg(hcan1);
	  if (rxStatus == RX_SUCCESSFUL)
	  {
		  CAN_TX_STATUS txStatus = relayCanMsg(hcan2, false);
		  if (txStatus == TX_BUFFER_FULL)
		  {
			  char uartBuffer[] = "TX2_BUFFER_FULL\n";
			  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
		  }
		  else if (txStatus == TX_ERROR)
		  {
			  char uartBuffer[] = "TX2_ERROR\n";
			  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
		  }
		  else if (txStatus == TX_SUCCESSFUL)
		  {
			  char uartBuffer[] = "TX2\n";
			  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
		  }
	  }
	  else if (rxStatus == RX_ERROR)
	  {
		  char uartBuffer[] = "RX1_ERROR\n";
		  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
	  }

	  // Uncomment for two-way gateway
	  /*
	  rxStatus = recvCanMsg(hcan2);
	  if (rxStatus == RX_SUCCESSFUL)
	  {
		  CAN_TX_STATUS txStatus = relayCanMsg(hcan1, false);
		  if (txStatus == TX_BUFFER_FULL)
		  {
			  char uartBuffer[] = "TX2_BUFFER_FULL\n";
			  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
		  }
		  else if (txStatus == TX_ERROR)
		  {
			  char uartBuffer[] = "TX2_ERROR\n";
			  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
		  }
		  else if (txStatus == TX_SUCCESSFUL)
		  {
			  char uartBuffer[] = "TX2\n";
			  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
		  }
	  }
	  else if (rxStatus == RX_ERROR)
	  {
		  char uartBuffer[] = "RX1_ERROR\n";
		  sendSerialMsg(uartBuffer, sizeof(uartBuffer));
	  }
	  */

	  //HAL_Delay(100);
	  //sendTestMsg(hcan2, 0xAA);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.SlaveStartFilterBank = 14; // 14 to 27 are assigned to slave CAN (CAN2), 0 to 13 are assigned to CAN1
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 18;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	/*CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.FilterIdHigh = 0;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);*/

	if (HAL_CAN_Start(&hcan2) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
CAN_TX_STATUS sendTestMsg(CAN_HandleTypeDef can, uint8_t data1)
{
	TxHeader.DLC = 1;
	TxHeader.StdId = 0x123;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxData[0] = data1;
	return sendCanMsg(can);
}

CAN_TX_STATUS sendCanMsg(CAN_HandleTypeDef can)
{
	uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&can);
	if (freeLevel == 0)
		return TX_BUFFER_FULL;

	HAL_StatusTypeDef txStatus = HAL_CAN_AddTxMessage(&can, &TxHeader, TxData, &TxMailbox);
	if (txStatus != HAL_OK)
	{
		/* Transmission request Error */
		//Error_Handler();
		HAL_CAN_ResetError(&can);
		return TX_ERROR;
	}
	return TX_SUCCESSFUL;
}

CAN_RX_STATUS recvCanMsg(CAN_HandleTypeDef can)
{
	uint32_t fillLevel = HAL_CAN_GetRxFifoFillLevel(&can, CAN_RX_FIFO0);

	if (fillLevel == 0)
		return RX_BUFFER_EMPTY;

	HAL_StatusTypeDef rxStatus = HAL_CAN_GetRxMessage(&can, CAN_RX_FIFO0, &RxHeader, RxData);
	if (rxStatus != HAL_OK)
	{
		/* Reception Error */
		//Error_Handler();
		HAL_CAN_ResetError(&can);
		return RX_ERROR;
	}
	return RX_SUCCESSFUL;

}

CAN_TX_STATUS relayCanMsg(CAN_HandleTypeDef can, bool force)
{
	// Filter messages by id and relay only allowed ids
	if (!force)
	{
		bool relayMsg = false;
		if (RxHeader.IDE == CAN_ID_STD)
		{
			for(int i = 0; i < allowedIdsSize; i++)
			{
				if (allowedIds[i] == RxHeader.StdId)
				{
					relayMsg = true;
					break;
				}
			}
		}
		else if (RxHeader.IDE == CAN_ID_EXT)
		{
			for(int i = 0; i < allowedIdsSize; i++)
			{
				if (allowedIds[i] == RxHeader.ExtId)
				{
					relayMsg = true;
					break;
				}
			}
		}

		if (!relayMsg)
			return NO_MATCH;
	}

	// Copy rx buffer to tx buffer
	memcpy(TxData, RxData, 8);
	TxHeader.DLC = RxHeader.DLC;
	TxHeader.StdId = RxHeader.StdId;
	TxHeader.ExtId = RxHeader.ExtId;
	TxHeader.RTR = RxHeader.RTR;
	TxHeader.IDE = RxHeader.IDE;

	// Send message
	return sendCanMsg(can);
}

void sendSerialMsg(char *buffer, uint16_t bufferSize)
{
#ifndef DEBUG
	return;
#endif
	HAL_UART_Transmit(&huart3, buffer, bufferSize-1, 10);
}

void *memcpy (void *dest, const void *src, size_t len)
{
  char *d = dest;
  const char *s = src;
  while (len--)
    *d++ = *s++;
  return dest;
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
