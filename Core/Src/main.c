/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Ready-to-run for LPS22HB + HTS221)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lps22hb.h"
#include "hts221.h"
#include "height_calculation.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
LPS22HB_t LPS22HB_sensor;
HTS221_t hts221_sensor;
uint16_t tx_length;
uint8_t tx_buffer[128];
float pressure_hPa = 0.0f;
float temperature_C = 0.0f;
float humidity_rh = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Configure NVIC */
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* Register I2C callback functions for sensor drivers (polling versions) */
  LPS22HB_RegisterIOFunctions(i2c_master_read_multi_polling,
                              i2c_master_write_multi_polling,
                              i2c_master_write_byte_polling,
                              i2c_master_read_byte_polling);

  HTS221_RegisterIOFunctions(i2c_master_read_multi_polling,
                             i2c_master_write_multi_polling,
                             i2c_master_write_byte_polling,
                             i2c_master_read_byte_polling);

  /* Initialize driver contexts */
  LPS22HB_InitDriver(&LPS22HB_sensor);
  HTS221_InitDriver(&hts221_sensor);

  /* Initialize sensors */
  if (LPS22HB_Init(&LPS22HB_sensor, 100) != LPS22HB_OK)
  {
    Error_Handler();
  }

  if (HTS221_Init(&hts221_sensor, 100) != HTS221_OK)
  {
    Error_Handler();
  }

  /* Read initial pressure to set reference for height calculation */
  if (LPS22HB_ReadPressure(&LPS22HB_sensor, &pressure_hPa, 100) == LPS22HB_OK)
  {
    Height_SetReferencePressure(pressure_hPa);
  }

  /* Infinite loop */
  while (1)
  {
    /* Read sensors */
    if (LPS22HB_ReadPressure(&LPS22HB_sensor, &pressure_hPa, 100) != LPS22HB_OK)
    {
      /* handle error (skip this cycle) */
    }

    if (HTS221_ReadTemperature(&hts221_sensor, &temperature_C, 100) != HTS221_OK)
    {
      /* handle error */
    }

    if (HTS221_ReadHumidity(&hts221_sensor, &humidity_rh, 100) != HTS221_OK)
    {
      /* handle error */
    }

    /* Format CSV: temperature [°C] "xx.x", rel. humidity [%] "xx", pressure [hPa] "xxxx.xx", relative height [m] "xxx.xx" */
    float rel_height = Height_GetRelativeHeight(pressure_hPa);
    tx_length = sprintf((char*)tx_buffer, "%.1f,%d,%.2f,%.2f\r\n",
                        temperature_C, (int)humidity_rh, pressure_hPa, rel_height);

    /* Send via USART using DMA */
    while(LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_7)); /* wait previous */
    USART2_PutBuffer(tx_buffer, (uint8_t)tx_length);

    /* toggle heartbeat LED */
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_3);

    LL_mDelay(1000);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0) {}

  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1) {}

  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {}

  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_3);
    LL_mDelay(200);
  }
}
