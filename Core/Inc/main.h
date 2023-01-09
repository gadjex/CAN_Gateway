/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum CAN_TX_STATUS {TX_SUCCESSFUL, TX_BUFFER_FULL, TX_ERROR, NO_MATCH} CAN_TX_STATUS;
typedef enum CAN_RX_STATUS {RX_SUCCESSFUL, RX_BUFFER_EMPTY, RX_ERROR} CAN_RX_STATUS;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
CAN_TX_STATUS sendTestMsg(CAN_HandleTypeDef can, uint8_t data1);
CAN_TX_STATUS sendCanMsg(CAN_HandleTypeDef can);
CAN_RX_STATUS recvCanMsg(CAN_HandleTypeDef can);
CAN_TX_STATUS relayCanMsg(CAN_HandleTypeDef can, bool force);
void sendSerialMsg(char *buffer, uint16_t bufferSize);
void *memcpy (void *dest, const void *src, size_t len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
//#define DEBUG
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
