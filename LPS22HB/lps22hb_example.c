/**
  ******************************************************************************
  * @file    LPS22HB_example.c
  * @brief   Example usage of LPS22HB pressure sensor driver
  ******************************************************************************
  * @attention
  *
  * This file demonstrates how to initialize and use the LPS22HB sensor
  * with the I2C polling functions.
  *
  ******************************************************************************
  */

/*
 * Example usage in main.c:
 * 
 * #include "LPS22HB/LPS22HB.h"
 * #include "i2c.h"
 * 
 * // Global LPS22HB context
 * LPS22HB_t LPS22HB;
 * 
 * int main(void)
 * {
 *     // ... System initialization, I2C init ...
 *     
 *     // Register I2C functions ONCE for all sensor instances
 *     LPS22HB_RegisterIOFunctions(i2c_master_read_multi_polling, 
 *                                 i2c_master_write_multi_polling, 
 *                                 i2c_master_write_byte_polling);
 *     
 *     // Initialize LPS22HB driver context
 *     LPS22HB_InitDriver(&LPS22HB);
 *     
 *     // Initialize LPS22HB sensor
 *     if (LPS22HB_Init(&LPS22HB, 100) == LPS22HB_OK)
 *     {
 *         // Sensor initialized successfully
 *     }
 *     else
 *     {
 *         // Initialization failed - handle error
 *     }
 *     
 *     while (1)
 *     {
 *         float pressure_hPa;
 *         float temperature_C;
 *         
 *         // Read pressure
 *         if (LPS22HB_ReadPressure(&LPS22HB, &pressure_hPa, 100) == LPS22HB_OK)
 *         {
 *             // Use pressure value
 *             // pressure_hPa contains pressure in hectopascals (hPa)
 *         }
 *         
 *         // Read temperature
 *         if (LPS22HB_ReadTemperature(&LPS22HB, &temperature_C, 100) == LPS22HB_OK)
 *         {
 *             // Use temperature value
 *             // temperature_C contains temperature in degrees Celsius
 *         }
 *         
 *         // Delay before next reading
 *         LL_mDelay(1000);  // 1 second delay
 *     }
 * }
 * 
 * 
 * Multiple sensor instances:
 * 
 * LPS22HB_t lps_sensor1;
 * LPS22HB_t lps_sensor2;
 * 
 * // Register I2C functions ONCE
 * LPS22HB_RegisterIOFunctions(i2c_master_read_multi_polling, 
 *                             i2c_master_write_multi_polling, 
 *                             i2c_master_write_byte_polling);
 * 
 * // Initialize first sensor
 * LPS22HB_InitDriver(&lps_sensor1);
 * LPS22HB_Init(&lps_sensor1, 100);
 * 
 * // Initialize second sensor (uses same registered functions)
 * LPS22HB_InitDriver(&lps_sensor2);
 * LPS22HB_Init(&lps_sensor2, 100);
 * 
 * // Both sensors use the same I2C functions
 * float pressure1, pressure2;
 * LPS22HB_ReadPressure(&lps_sensor1, &pressure1, 100);
 * LPS22HB_ReadPressure(&lps_sensor2, &pressure2, 100);
 * 
 * 
 * Advanced usage example - Check WHO_AM_I before init:
 * 
 * LPS22HB_t LPS22HB;
 * LPS22HB_Status_t status;
 * 
 * // Register functions first
 * LPS22HB_RegisterIOFunctions(i2c_master_read_multi_polling, 
 *                             i2c_master_write_multi_polling, 
 *                             i2c_master_write_byte_polling);
 * 
 * // Initialize driver
 * LPS22HB_InitDriver(&LPS22HB);
 * 
 * // Check WHO_AM_I register
 * status = LPS22HB_CheckWhoAmI(&LPS22HB, 100);
 * if (status == LPS22HB_OK)
 * {
 *     // Sensor is present and responding
 *     LPS22HB_Init(&LPS22HB, 100);
 * }
 * else if (status == LPS22HB_INVALID_WHO_AM_I)
 * {
 *     // Wrong sensor or communication error
 * }
 * else
 * {
 *     // Timeout or other error
 * }
 * 
 * 
 * Reading raw values:
 * 
 * int32_t raw_pressure;
 * int16_t raw_temperature;
 * 
 * if (LPS22HB_ReadPressureRaw(&LPS22HB, &raw_pressure, 100) == LPS22HB_OK)
 * {
 *     // raw_pressure is 24-bit signed value
 *     // To convert to hPa manually: pressure_hPa = raw_pressure / 4096.0
 * }
 * 
 * if (LPS22HB_ReadTemperatureRaw(&LPS22HB, &raw_temperature, 100) == LPS22HB_OK)
 * {
 *     // raw_temperature is 16-bit signed value
 *     // To convert to °C manually: temp_C = 42.5 + (raw_temperature / 480.0)
 * }
 * 
 * 
 * Custom register access:
 * 
 * uint8_t ctrl_reg1;
 * 
 * // Read CTRL_REG1
 * if (LPS22HB_ReadRegister(&LPS22HB, LPS22HB_REG_CTRL_REG1, &ctrl_reg1, 100) == LPS22HB_OK)
 * {
 *     // ctrl_reg1 contains current value
 * }
 * 
 * // Modify and write back
 * ctrl_reg1 |= LPS22HB_CTRL_REG1_BDU;  // Enable block data update
 * LPS22HB_WriteRegister(&LPS22HB, LPS22HB_REG_CTRL_REG1, ctrl_reg1, 100);
 * 
 * 
 * Check data availability:
 * 
 * uint8_t status_reg;
 * 
 * if (LPS22HB_ReadStatus(&LPS22HB, &status_reg, 100) == LPS22HB_OK)
 * {
 *     if (status_reg & LPS22HB_STATUS_P_DA)
 *     {
 *         // New pressure data available
 *     }
 *     
 *     if (status_reg & LPS22HB_STATUS_T_DA)
 *     {
 *         // New temperature data available
 *     }
 * }
 */
