/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN Private defines */
#define FM25V10_CS_GPIO_PORT   GPIOE
#define FM25V10_CS_PIN         GPIO_PIN_3

#define FM25V10_CMD_WREN       0x06  // Set Write Enable Latch
#define FM25V10_CMD_WRDI       0x04  // Reset Write Enable Latch
#define FM25V10_CMD_RDSR       0x05  // Read Status Register
#define FM25V10_CMD_WRSR       0x01  // Write Status Register
#define FM25V10_CMD_READ       0x03  // Read Memory Data
#define FM25V10_CMD_WRITE      0x02  // Write Memory Data
/* USER CODE END Private defines */

void MX_SPI4_Init(void);

/* USER CODE BEGIN Prototypes */
void FM25V10_Write(uint32_t addr, uint8_t* data, uint16_t size);
void FM25V10_Read(uint32_t addr, uint8_t* data, uint16_t size);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

