/**
  ******************************************************************************
  * @file    hts221.c
  * @brief   HTS221 humidity and temperature sensor driver implementation
  ******************************************************************************
  * @attention
  *
  * HTS221 capacitive digital sensor driver
  * Provides functions for initialization, configuration, and reading
  * humidity and temperature data via I2C interface.
  * Uses factory calibration data for accurate measurements.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hts221.h"
#include <stddef.h>
#include "math.h"

/* Private Variables ---------------------------------------------------------*/
static HTS221_ReadMulti_t  i2c_read_multi = NULL;
static HTS221_WriteMulti_t i2c_write_multi = NULL;
static HTS221_WriteByte_t  i2c_write_byte = NULL;
static HTS221_ReadByte_t   i2c_read_byte = NULL;

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief Register I2C functions for all HTS221 instances
 */
HTS221_Status_t HTS221_RegisterIOFunctions(HTS221_ReadMulti_t read_multi, 
                                           HTS221_WriteMulti_t write_multi, 
                                           HTS221_WriteByte_t write_byte,
                                           HTS221_ReadByte_t read_byte)
{
    if (read_multi == NULL || write_multi == NULL || write_byte == NULL || read_byte == NULL) {
        return HTS221_ERROR;
    }

    i2c_read_multi = read_multi;
    i2c_write_multi = write_multi;
    i2c_write_byte = write_byte;
    i2c_read_byte = read_byte;

    return HTS221_OK;
}

/**
 * @brief Initialize HTS221 driver context
 */
HTS221_Status_t HTS221_InitDriver(HTS221_t *hts)
{
    if (hts == NULL) {
        return HTS221_ERROR;
    }

    hts->i2c_address = HTS221_I2C_ADDRESS;

    return HTS221_OK;
}

/**
 * @brief Check WHO_AM_I register to verify sensor presence
 */
HTS221_Status_t HTS221_CheckWhoAmI(HTS221_t *hts, uint32_t timeout_ms)
{
    uint8_t who_am_i = 0;

    if (hts == NULL || i2c_read_byte == NULL) {
        return HTS221_ERROR;
    }

    /* Read WHO_AM_I register using registered I2C function */
    who_am_i = i2c_read_byte(hts->i2c_address, HTS221_REG_WHO_AM_I, timeout_ms);
    if (who_am_i == 0) {
        return HTS221_TIMEOUT;
    }

    /* Check if WHO_AM_I matches expected value */
    if (who_am_i != HTS221_WHO_AM_I_VALUE) {
        return HTS221_INVALID_WHO_AM_I;
    }

    return HTS221_OK;
}

/**
 * @brief Read and store calibration data from sensor
 */
HTS221_Status_t HTS221_ReadCalibration(HTS221_t *hts, uint32_t timeout_ms)
{
    uint8_t data[2];
    uint8_t t_msb;

    if (hts == NULL || i2c_read_multi == NULL || i2c_read_byte == NULL) {
        return HTS221_ERROR;
    }

    /* Read H0_RH_x2 using single-byte read */
    data[0] = i2c_read_byte(hts->i2c_address, HTS221_REG_H0_RH_X2, timeout_ms);
    if (data[0] == 0) {
        return HTS221_TIMEOUT;
    }
    hts->calibration.H0_RH = data[0] >> 1;  /* Divide by 2 */

    /* Read H1_RH_x2 using single-byte read */
    data[0] = i2c_read_byte(hts->i2c_address, HTS221_REG_H1_RH_X2, timeout_ms);
    if (data[0] == 0) {
        return HTS221_TIMEOUT;
    }
    hts->calibration.H1_RH = data[0] >> 1;  /* Divide by 2 */

    /* Read T0_degC_x8 and T1_degC_x8 using multi-byte read with auto-increment */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_T0_DEGC_X8 | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }

    /* Read T1_T0_msb for upper bits using single-byte read */
    t_msb = i2c_read_byte(hts->i2c_address, HTS221_REG_T1_T0_MSB, timeout_ms);
    if (t_msb == 0) {
        return HTS221_TIMEOUT;
    }

    /* Combine bits to form T0_DEGC and T1_DEGC (10-bit values) */
    hts->calibration.T0_DEGC = ((uint16_t)(t_msb & 0x03) << 8) | data[0];
    hts->calibration.T1_DEGC = ((uint16_t)(t_msb & 0x0C) << 6) | data[1];

    /* Read H0_T0_OUT (16-bit) using multi-byte read with auto-increment */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_H0_T0_OUT_L | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }
    hts->calibration.H0_T0_OUT = (int16_t)((data[1] << 8) | data[0]);

    /* Read H1_T0_OUT (16-bit) using multi-byte read with auto-increment */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_H1_T0_OUT_L | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }
    hts->calibration.H1_T0_OUT = (int16_t)((data[1] << 8) | data[0]);

    /* Read T0_OUT (16-bit) using multi-byte read with auto-increment */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_T0_OUT_L | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }
    hts->calibration.T0_OUT = (int16_t)((data[1] << 8) | data[0]);

    /* Read T1_OUT (16-bit) using multi-byte read with auto-increment */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_T1_OUT_L | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }
    hts->calibration.T1_OUT = (int16_t)((data[1] << 8) | data[0]);

    return HTS221_OK;
}

/**
 * @brief Initialize HTS221 sensor with default configuration
 */
HTS221_Status_t HTS221_Init(HTS221_t *hts, uint32_t timeout_ms)
{
    uint8_t ctrl_reg1_value;
    HTS221_Status_t status;

    if (hts == NULL || i2c_write_byte == NULL) {
        return HTS221_ERROR;
    }

    /* Check WHO_AM_I first */
    status = HTS221_CheckWhoAmI(hts, timeout_ms);
    if (status != HTS221_OK) {
        return status;
    }

    /* Read calibration data */
    status = HTS221_ReadCalibration(hts, timeout_ms);
    if (status != HTS221_OK) {
        return status;
    }

    /* Configure CTRL_REG1:
     * - Power-down mode disabled (active mode)
     * - Output data rate: 1 Hz
     * - Block data update enabled
     */
    ctrl_reg1_value = HTS221_CTRL_REG1_PD | 
                      HTS221_CTRL_REG1_ODR_7HZ | 
                      HTS221_CTRL_REG1_BDU;

    /* Write using registered I2C function */
    if (i2c_write_byte(hts->i2c_address, HTS221_REG_CTRL_REG1, ctrl_reg1_value, timeout_ms) != 1) {
        return HTS221_TIMEOUT;
    }

    return HTS221_OK;
}

/**
 * @brief Read raw humidity value from sensor
 */
HTS221_Status_t HTS221_ReadHumidityRaw(HTS221_t *hts, int16_t *humidity, uint32_t timeout_ms)
{
    uint8_t data[2];

    if (hts == NULL || humidity == NULL || i2c_read_multi == NULL) {
        return HTS221_ERROR;
    }

    /* Read 2 bytes of humidity data (16-bit value) with auto-increment enabled */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_HUMIDITY_OUT_L | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }

    /* Combine bytes into 16-bit signed value */
    *humidity = (int16_t)((data[1] << 8) | data[0]);

    return HTS221_OK;
}

/**
 * @brief Read humidity value in %RH (relative humidity)
 */
HTS221_Status_t HTS221_ReadHumidity(HTS221_t *hts, float *humidity_rh, uint32_t timeout_ms)
{
    int16_t raw_humidity;
    HTS221_Status_t status;
    float h_slope;

    if (humidity_rh == NULL) {
        return HTS221_ERROR;
    }

    status = HTS221_ReadHumidityRaw(hts, &raw_humidity, timeout_ms);
    if (status != HTS221_OK) {
        return status;
    }

    /* Linear interpolation using calibration data:
     * H_out = (H1_RH - H0_RH) / (H1_T0_OUT - H0_T0_OUT) * (H_OUT - H0_T0_OUT) + H0_RH
     */
    h_slope = (float)(hts->calibration.H1_RH - hts->calibration.H0_RH) / 
              (float)(hts->calibration.H1_T0_OUT - hts->calibration.H0_T0_OUT);

    *humidity_rh = h_slope * (float)(raw_humidity - hts->calibration.H0_T0_OUT) +
                   (float)hts->calibration.H0_RH;

    /* Clamp to valid range 0-100% */
    if (*humidity_rh < 0.0f) {
        *humidity_rh = 0.0f;
    } else if (*humidity_rh > 100.0f) {
        *humidity_rh = 100.0f;
    }

    return HTS221_OK;
}

/**
 * @brief Read raw temperature value from sensor
 */
HTS221_Status_t HTS221_ReadTemperatureRaw(HTS221_t *hts, int16_t *temperature, uint32_t timeout_ms)
{
    uint8_t data[2];

    if (hts == NULL || temperature == NULL || i2c_read_multi == NULL) {
        return HTS221_ERROR;
    }

    /* Read 2 bytes of temperature data (16-bit value) with auto-increment enabled */
    if (i2c_read_multi(hts->i2c_address, HTS221_REG_TEMP_OUT_L | 0x80, data, 2, timeout_ms) != 2) {
        return HTS221_TIMEOUT;
    }

    /* Combine bytes into 16-bit signed value */
    *temperature = (int16_t)((data[1] << 8) | data[0]);

    return HTS221_OK;
}

/**
 * @brief Read temperature value in degrees Celsius
 */
HTS221_Status_t HTS221_ReadTemperature(HTS221_t *hts, float *temperature_C, uint32_t timeout_ms)
{
    int16_t raw_temperature;
    HTS221_Status_t status;
    float t_slope;

    if (temperature_C == NULL) {
        return HTS221_ERROR;
    }

    status = HTS221_ReadTemperatureRaw(hts, &raw_temperature, timeout_ms);
    if (status != HTS221_OK) {
        return status;
    }

    /* Linear interpolation using calibration data:
     * T_out = (T1_degC - T0_degC) / (T1_OUT - T0_OUT) * (T_OUT - T0_OUT) + T0_degC
     * Note: T0_DEGC and T1_DEGC are stored as value * 8
     */
    t_slope = (float)(hts->calibration.T1_DEGC - hts->calibration.T0_DEGC) / 8.0f / 
              (float)(hts->calibration.T1_OUT - hts->calibration.T0_OUT);

    *temperature_C = t_slope * (float)(raw_temperature - hts->calibration.T0_OUT) + 
                     (float)hts->calibration.T0_DEGC / 8.0f;

    return HTS221_OK;
}

/**
 * @brief Read status register
 */
HTS221_Status_t HTS221_ReadStatus(HTS221_t *hts, uint8_t *status, uint32_t timeout_ms)
{
    if (hts == NULL || status == NULL || i2c_read_byte == NULL) {
        return HTS221_ERROR;
    }

    /* Read status register using single-byte read function */
    *status = i2c_read_byte(hts->i2c_address, HTS221_REG_STATUS_REG, timeout_ms);
    if (*status == 0) {
        return HTS221_TIMEOUT;
    }

    return HTS221_OK;
}

/**
 * @brief Write to a single register
 */
HTS221_Status_t HTS221_WriteRegister(HTS221_t *hts, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms)
{
    if (hts == NULL || i2c_write_byte == NULL) {
        return HTS221_ERROR;
    }

    /* Write using registered I2C function */
    if (i2c_write_byte(hts->i2c_address, reg_addr, value, timeout_ms) != 1) {
        return HTS221_TIMEOUT;
    }

    return HTS221_OK;
}

/**
 * @brief Read from a single register
 */
HTS221_Status_t HTS221_ReadRegister(HTS221_t *hts, uint8_t reg_addr, uint8_t *value, uint32_t timeout_ms)
{
    if (hts == NULL || value == NULL || i2c_read_byte == NULL) {
        return HTS221_ERROR;
    }

    /* Read using single-byte read function */
    *value = i2c_read_byte(hts->i2c_address, reg_addr, timeout_ms);
    if (*value == 0) {
        return HTS221_TIMEOUT;
    }

    return HTS221_OK;
}
