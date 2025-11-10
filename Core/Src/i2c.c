/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "stm32f3xx_ll_utils.h"

uint8_t i2c_rx_data = 0;

/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00201D2B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */

/**
 * @brief Receive one byte from an I2C slave using polling (no interrupts)
 * @param slave_address 7-bit slave address
 * @param register_address Register/subaddress to read from
 * @param timeout_ms Timeout in milliseconds
 * @retval Received byte (0 on timeout/error)
 *
 * This function performs a write of the register address, followed by a read
 * of one byte. It polls status flags and uses LL_mDelay(1) for timeout control.
 */
uint8_t i2c_master_read_byte_polling(uint8_t slave_address, uint8_t register_address, uint32_t timeout_ms)
{
  uint8_t data = 0;
  uint32_t remaining_ms = timeout_ms;

  /* Send register address (1 byte write) */
  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* Wait for TXIS then transmit the register address, using LL_mDelay(1) per ms */
  while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_TransmitData8(I2C1, register_address);

  /* Wait for STOP (auto-end) */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_ClearFlag_STOP(I2C1);

  /* Start read of one byte */
  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

  /* Wait for data available */
  while(!LL_I2C_IsActiveFlag_RXNE(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  data = LL_I2C_ReceiveData8(I2C1);

  /* Wait for STOP and clear it */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  LL_I2C_ClearFlag_NACK(I2C1);

  return data;
}

/**
 * @brief Receive multiple bytes from an I2C slave using polling (no interrupts)
 * @param slave_address 7-bit slave address
 * @param register_address Register/subaddress to start reading from
 * @param data Pointer to buffer where received data will be stored
 * @param size Number of bytes to read
 * @param timeout_ms Timeout in milliseconds for the entire transaction
 * @retval Number of bytes successfully read (0 on timeout/error)
 *
 * This function performs a write of the register address, followed by a read
 * of multiple bytes. It polls status flags and uses LL_mDelay(1) for timeout control.
 */
uint8_t i2c_master_read_multi_polling(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t size, uint32_t timeout_ms)
{
  uint32_t remaining_ms = timeout_ms;
  uint8_t bytes_read = 0;

  /* Send register address (1 byte write) */
  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* Wait for TXIS then transmit the register address */
  while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_TransmitData8(I2C1, register_address);

  /* Wait for STOP (auto-end) */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_ClearFlag_STOP(I2C1);

  /* Start read of multiple bytes */
  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

  /* Read all requested bytes */
  for(bytes_read = 0; bytes_read < size; bytes_read++)
  {
    /* Wait for data available */
    while(!LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
      if(remaining_ms == 0) return bytes_read;
      LL_mDelay(1);
      remaining_ms--;
    }
    data[bytes_read] = LL_I2C_ReceiveData8(I2C1);
  }

  /* Wait for STOP and clear it */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    if(remaining_ms == 0) return bytes_read;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  LL_I2C_ClearFlag_NACK(I2C1);

  return bytes_read;
}

/**
 * @brief Write one byte to an I2C slave using polling (no interrupts)
 * @param slave_address 7-bit slave address
 * @param register_address Register/subaddress to write to
 * @param data Data byte to write
 * @param timeout_ms Timeout in milliseconds
 * @retval 1 on success, 0 on timeout/error
 *
 * This function performs a write of the register address followed by the data byte.
 */
uint8_t i2c_master_write_byte_polling(uint8_t slave_address, uint8_t register_address, uint8_t data, uint32_t timeout_ms)
{
  uint32_t remaining_ms = timeout_ms;

  /* Send register address + data (2 bytes write) */
  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* Wait for TXIS then transmit the register address */
  while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_TransmitData8(I2C1, register_address);

  /* Wait for TXIS then transmit the data byte */
  while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_TransmitData8(I2C1, data);

  /* Wait for STOP (auto-end) */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  LL_I2C_ClearFlag_NACK(I2C1);

  return 1;
}

/**
 * @brief Write multiple bytes to an I2C slave using polling (no interrupts)
 * @param slave_address 7-bit slave address
 * @param register_address Register/subaddress to start writing to
 * @param data Pointer to buffer containing data to write
 * @param size Number of bytes to write
 * @param timeout_ms Timeout in milliseconds for the entire transaction
 * @retval Number of bytes successfully written (0 on timeout/error)
 *
 * This function performs a write of the register address followed by multiple data bytes.
 */
uint8_t i2c_master_write_multi_polling(uint8_t slave_address, uint8_t register_address, uint8_t *data, uint8_t size, uint32_t timeout_ms)
{
  uint32_t remaining_ms = timeout_ms;
  uint8_t bytes_written = 0;

  /* Send register address + data (1 + size bytes write) */
  LL_I2C_HandleTransfer(I2C1, slave_address, LL_I2C_ADDRSLAVE_7BIT, size + 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  /* Wait for TXIS then transmit the register address */
  while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    if(remaining_ms == 0) return 0;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_TransmitData8(I2C1, register_address);

  /* Transmit all data bytes */
  for(bytes_written = 0; bytes_written < size; bytes_written++)
  {
    /* Wait for TXIS */
    while(!LL_I2C_IsActiveFlag_TXIS(I2C1))
    {
      if(remaining_ms == 0) return bytes_written;
      LL_mDelay(1);
      remaining_ms--;
    }
    LL_I2C_TransmitData8(I2C1, data[bytes_written]);
  }

  /* Wait for STOP (auto-end) */
  while(!LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    if(remaining_ms == 0) return bytes_written;
    LL_mDelay(1);
    remaining_ms--;
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  LL_I2C_ClearFlag_NACK(I2C1);

  return bytes_written;
}
/* USER CODE END 1 */
