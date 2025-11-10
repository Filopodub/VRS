/**
  ******************************************************************************
  * @file    LPS22HB.c
  * @brief   LPS22HB pressure sensor driver implementation
  ******************************************************************************
  * @attention
  *
  * LPS22HB MEMS pressure sensor driver
  * Provides functions for initialization, configuration, and reading
  * pressure and temperature data via I2C interface.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LPS22HB.h"
#include <stddef.h>

/* Private Variables ---------------------------------------------------------*/
static LPS22HB_ReadMulti_t  i2c_read_multi = NULL;
static LPS22HB_WriteMulti_t i2c_write_multi = NULL;
static LPS22HB_WriteByte_t  i2c_write_byte = NULL;
static LPS22HB_ReadByte_t   i2c_read_byte = NULL;

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief Register I2C functions for all LPS22HB instances
 */
LPS22HB_Status_t LPS22HB_RegisterIOFunctions(LPS22HB_ReadMulti_t read_multi, 
                                             LPS22HB_WriteMulti_t write_multi, 
                                             LPS22HB_WriteByte_t write_byte,
                                             LPS22HB_ReadByte_t read_byte)
{
    if (read_multi == NULL || write_multi == NULL || write_byte == NULL || read_byte == NULL) {
        return LPS22HB_ERROR;
    }

    i2c_read_multi = read_multi;
    i2c_write_multi = write_multi;
    i2c_write_byte = write_byte;
    i2c_read_byte = read_byte;

    return LPS22HB_OK;
}

/**
 * @brief Initialize LPS22HB driver context
 */
LPS22HB_Status_t LPS22HB_InitDriver(LPS22HB_t *lps)
{
    if (lps == NULL) {
        return LPS22HB_ERROR;
    }

    lps->i2c_address = LPS22HB_I2C_ADDRESS;

    return LPS22HB_OK;
}

/**
 * @brief Check WHO_AM_I register to verify sensor presence
 */
LPS22HB_Status_t LPS22HB_CheckWhoAmI(LPS22HB_t *lps, uint32_t timeout_ms)
{
    uint8_t who_am_i = 0;

    if (lps == NULL || i2c_read_byte == NULL) {
        return LPS22HB_ERROR;
    }

    /* Read WHO_AM_I register using single-byte read function */
    who_am_i = i2c_read_byte(lps->i2c_address, LPS22HB_REG_WHO_AM_I, timeout_ms);
    if (who_am_i == 0) {
        return LPS22HB_TIMEOUT;
    }

    /* Check if WHO_AM_I matches expected value */
    if (who_am_i != LPS22HB_WHO_AM_I_VALUE) {
        return LPS22HB_INVALID_WHO_AM_I;
    }

    return LPS22HB_OK;
}

/**
 * @brief Initialize LPS22HB sensor with default configuration
 */
LPS22HB_Status_t LPS22HB_Init(LPS22HB_t *lps, uint32_t timeout_ms)
{
    uint8_t ctrl_reg1_value;
    LPS22HB_Status_t status;

    if (lps == NULL || i2c_write_byte == NULL) {
        return LPS22HB_ERROR;
    }

    /* Check WHO_AM_I first */
    status = LPS22HB_CheckWhoAmI(lps, timeout_ms);
    if (status != LPS22HB_OK) {
        return status;
    }

    /* Configure CTRL_REG1 for LPS22HB:
     * - Output data rate: 25 Hz (ODR bits in LPS22HB control power mode)
     * - Block data update enabled
     * Note: In LPS22HB, ODR != 0 means active mode
     */
    ctrl_reg1_value = LPS22HB_CTRL_REG1_ODR_25HZ | 
                      LPS22HB_CTRL_REG1_BDU;

    /* Write using registered I2C function */
    if (i2c_write_byte(lps->i2c_address, LPS22HB_REG_CTRL_REG1, ctrl_reg1_value, timeout_ms) != 1) {
        return LPS22HB_TIMEOUT;
    }

    return LPS22HB_OK;
}

/**
 * @brief Read raw pressure value from sensor
 */
LPS22HB_Status_t LPS22HB_ReadPressureRaw(LPS22HB_t *lps, int32_t *pressure, uint32_t timeout_ms)
{
    uint8_t data[3];

    if (lps == NULL || pressure == NULL || i2c_read_multi == NULL) {
        return LPS22HB_ERROR;
    }

    /* Read 3 bytes of pressure data (24-bit value) using registered I2C function */
    if (i2c_read_multi(lps->i2c_address, LPS22HB_REG_PRESS_OUT_XL, data, 3, timeout_ms) != 3) {
        return LPS22HB_TIMEOUT;
    }

    /* Combine bytes into 24-bit signed value */
    *pressure = (int32_t)((data[2] << 16) | (data[1] << 8) | data[0]);

    /* Sign extend from 24-bit to 32-bit */
    if (*pressure & 0x00800000) {
        *pressure |= 0xFF000000;
    }

    return LPS22HB_OK;
}

/**
 * @brief Read pressure value in hPa (hectopascals)
 */
LPS22HB_Status_t LPS22HB_ReadPressure(LPS22HB_t *lps, float *pressure_hPa, uint32_t timeout_ms)
{
    int32_t raw_pressure;
    LPS22HB_Status_t status;

    if (pressure_hPa == NULL) {
        return LPS22HB_ERROR;
    }

    status = LPS22HB_ReadPressureRaw(lps, &raw_pressure, timeout_ms);
    if (status != LPS22HB_OK) {
        return status;
    }

    /* Convert raw value to hPa (hectopascals)
     * Sensitivity: 4096 LSB/hPa
     */
    *pressure_hPa = (float)raw_pressure / 4096.0f;

    return LPS22HB_OK;
}

/**
 * @brief Read raw temperature value from sensor
 */
LPS22HB_Status_t LPS22HB_ReadTemperatureRaw(LPS22HB_t *lps, int16_t *temperature, uint32_t timeout_ms)
{
    uint8_t data[2];

    if (lps == NULL || temperature == NULL || i2c_read_multi == NULL) {
        return LPS22HB_ERROR;
    }

    /* Read 2 bytes of temperature data (16-bit value) using registered I2C function */
    if (i2c_read_multi(lps->i2c_address, LPS22HB_REG_TEMP_OUT_L, data, 2, timeout_ms) != 2) {
        return LPS22HB_TIMEOUT;
    }

    /* Combine bytes into 16-bit signed value */
    *temperature = (int16_t)((data[1] << 8) | data[0]);

    return LPS22HB_OK;
}

/**
 * @brief Read temperature value in degrees Celsius
 */
LPS22HB_Status_t LPS22HB_ReadTemperature(LPS22HB_t *lps, float *temperature_C, uint32_t timeout_ms)
{
    int16_t raw_temperature;
    LPS22HB_Status_t status;

    if (temperature_C == NULL) {
        return LPS22HB_ERROR;
    }

    status = LPS22HB_ReadTemperatureRaw(lps, &raw_temperature, timeout_ms);
    if (status != LPS22HB_OK) {
        return status;
    }

    /* Convert raw value to degrees Celsius for LPS22HB
     * Sensitivity: 100 LSB/°C
     * Temperature (°C) = TEMP_OUT / 100
     */
    *temperature_C = (float)raw_temperature / 100.0f;

    return LPS22HB_OK;
}

/**
 * @brief Read status register
 */
LPS22HB_Status_t LPS22HB_ReadStatus(LPS22HB_t *lps, uint8_t *status, uint32_t timeout_ms)
{
    if (lps == NULL || status == NULL || i2c_read_byte == NULL) {
        return LPS22HB_ERROR;
    }

    /* Read status register using single-byte read function */
    *status = i2c_read_byte(lps->i2c_address, LPS22HB_REG_STATUS_REG, timeout_ms);
    if (*status == 0) {
        return LPS22HB_TIMEOUT;
    }

    return LPS22HB_OK;
}

/**
 * @brief Write to a single register
 */
LPS22HB_Status_t LPS22HB_WriteRegister(LPS22HB_t *lps, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms)
{
    if (lps == NULL || i2c_write_byte == NULL) {
        return LPS22HB_ERROR;
    }

    /* Write using registered I2C function */
    if (i2c_write_byte(lps->i2c_address, reg_addr, value, timeout_ms) != 1) {
        return LPS22HB_TIMEOUT;
    }

    return LPS22HB_OK;
}

/**
 * @brief Read from a single register
 */
LPS22HB_Status_t LPS22HB_ReadRegister(LPS22HB_t *lps, uint8_t reg_addr, uint8_t *value, uint32_t timeout_ms)
{
    if (lps == NULL || value == NULL || i2c_read_byte == NULL) {
        return LPS22HB_ERROR;
    }

    /* Read using single-byte read function */
    *value = i2c_read_byte(lps->i2c_address, reg_addr, timeout_ms);
    if (*value == 0) {
        return LPS22HB_TIMEOUT;
    }

    return LPS22HB_OK;
}

/**
 * @brief Set reference pressure (24-bit value)
 */
LPS22HB_Status_t LPS22HB_SetReferencePressure(LPS22HB_t *lps, int32_t ref_pressure, uint32_t timeout_ms)
{
    uint8_t data[3];

    if (lps == NULL || i2c_write_multi == NULL) {
        return LPS22HB_ERROR;
    }

    /* Split 24-bit value into 3 bytes (LSB, middle, MSB) */
    data[0] = (uint8_t)(ref_pressure & 0xFF);        /* REF_P_XL */
    data[1] = (uint8_t)((ref_pressure >> 8) & 0xFF); /* REF_P_L */
    data[2] = (uint8_t)((ref_pressure >> 16) & 0xFF);/* REF_P_H */

    /* Write 3 bytes starting from REF_P_XL using multi-byte write */
    if (i2c_write_multi(lps->i2c_address, LPS22HB_REG_REF_P_XL, data, 3, timeout_ms) != 3) {
        return LPS22HB_TIMEOUT;
    }

    return LPS22HB_OK;
}

/**
 * @brief Write multiple consecutive registers
 */
LPS22HB_Status_t LPS22HB_WriteMultipleRegisters(LPS22HB_t *lps, uint8_t start_reg_addr, uint8_t *data, uint8_t size, uint32_t timeout_ms)
{
    if (lps == NULL || data == NULL || i2c_write_multi == NULL) {
        return LPS22HB_ERROR;
    }

    /* Write multiple bytes using registered I2C function */
    if (i2c_write_multi(lps->i2c_address, start_reg_addr, data, size, timeout_ms) != size) {
        return LPS22HB_TIMEOUT;
    }

    return LPS22HB_OK;
}
