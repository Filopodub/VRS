/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
/**
 * @brief  Read one byte from an I2C slave (polling, no interrupts)
 * @param  slave_address 7-bit slave address
 * @param  register_address Register/subaddress to read from
 * @param  timeout_ms Timeout in milliseconds
 * @retval Received byte (returns 0 on timeout/error)
 */
uint8_t i2c_master_read_byte_polling(uint8_t slave_address, uint8_t register_address, uint32_t timeout_ms);

/**
 * @brief  Read multiple bytes from an I2C slave (polling, no interrupts)
 * @param  slave_address 7-bit slave address
 * @param  register_address Register/subaddress to start reading from
 * @param  data Pointer to buffer where received data will be stored
 * @param  size Number of bytes to read
 * @param  timeout_ms Timeout in milliseconds for the entire transaction
 * @retval Number of bytes successfully read (0 on timeout/error)
 */
uint8_t i2c_master_read_multi_polling(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t size, uint32_t timeout_ms);

/**
 * @brief  Write one byte to an I2C slave (polling, no interrupts)
 * @param  slave_address 7-bit slave address
 * @param  register_address Register/subaddress to write to
 * @param  data Data byte to write
 * @param  timeout_ms Timeout in milliseconds
 * @retval 1 on success, 0 on timeout/error
 */
uint8_t i2c_master_write_byte_polling(uint8_t slave_address, uint8_t register_address, uint8_t data, uint32_t timeout_ms);

/**
 * @brief  Write multiple bytes to an I2C slave (polling, no interrupts)
 * @param  slave_address 7-bit slave address
 * @param  register_address Register/subaddress to start writing to
 * @param  data Pointer to buffer containing data to write
 * @param  size Number of bytes to write
 * @param  timeout_ms Timeout in milliseconds for the entire transaction
 * @retval Number of bytes successfully written (0 on timeout/error)
 */
uint8_t i2c_master_write_multi_polling(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t size, uint32_t timeout_ms);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

