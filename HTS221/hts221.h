/**
  ******************************************************************************
  * @file    hts221.h
  * @brief   HTS221 humidity and temperature sensor driver header
  ******************************************************************************
  * @attention
  *
  * HTS221 capacitive digital sensor for relative humidity and temperature
  * Datasheet: https://www.st.com/resource/en/datasheet/hts221.pdf
  * Operating range: 0-100% RH, -40 to +120°C
  *
  ******************************************************************************
  */

#ifndef HTS221_H_
#define HTS221_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* HTS221 I2C Address --------------------------------------------------------*/
#define HTS221_I2C_ADDRESS          0xBE  /* 7-bit address (0x5F) shifted left by 1 */

/* HTS221 Register Map -------------------------------------------------------*/
#define HTS221_REG_WHO_AM_I         0x0F  /* Device identification register */
#define HTS221_WHO_AM_I_VALUE       0xBC  /* Expected WHO_AM_I value */

#define HTS221_REG_AV_CONF          0x10  /* Humidity and temperature resolution */
#define HTS221_REG_CTRL_REG1        0x20  /* Control register 1 */
#define HTS221_REG_CTRL_REG2        0x21  /* Control register 2 */
#define HTS221_REG_CTRL_REG3        0x22  /* Control register 3 */

#define HTS221_REG_STATUS_REG       0x27  /* Status register */

#define HTS221_REG_HUMIDITY_OUT_L   0x28  /* Humidity output LSB */
#define HTS221_REG_HUMIDITY_OUT_H   0x29  /* Humidity output MSB */

#define HTS221_REG_TEMP_OUT_L       0x2A  /* Temperature output LSB */
#define HTS221_REG_TEMP_OUT_H       0x2B  /* Temperature output MSB */

/* Calibration registers */
#define HTS221_REG_H0_RH_X2         0x30  /* Humidity calibration LSB */
#define HTS221_REG_H1_RH_X2         0x31  /* Humidity calibration MSB */
#define HTS221_REG_T0_DEGC_X8       0x32  /* Temperature calibration LSB */
#define HTS221_REG_T1_DEGC_X8       0x33  /* Temperature calibration MSB */
#define HTS221_REG_T1_T0_MSB        0x35  /* Temperature calibration MSB bits */

#define HTS221_REG_H0_T0_OUT_L      0x36  /* H0_T0_OUT LSB */
#define HTS221_REG_H0_T0_OUT_H      0x37  /* H0_T0_OUT MSB */
#define HTS221_REG_H1_T0_OUT_L      0x3A  /* H1_T0_OUT LSB */
#define HTS221_REG_H1_T0_OUT_H      0x3B  /* H1_T0_OUT MSB */

#define HTS221_REG_T0_OUT_L         0x3C  /* T0_OUT LSB */
#define HTS221_REG_T0_OUT_H         0x3D  /* T0_OUT MSB */
#define HTS221_REG_T1_OUT_L         0x3E  /* T1_OUT LSB */
#define HTS221_REG_T1_OUT_H         0x3F  /* T1_OUT MSB */

/* CTRL_REG1 Bit Definitions -------------------------------------------------*/
#define HTS221_CTRL_REG1_PD         0x80  /* Power-down control (1 = active, 0 = power-down) */
#define HTS221_CTRL_REG1_BDU        0x04  /* Block data update */
#define HTS221_CTRL_REG1_ODR_MASK   0x03  /* Output data rate mask */
#define HTS221_CTRL_REG1_ODR_ONE_SHOT 0x00  /* One-shot mode */
#define HTS221_CTRL_REG1_ODR_1HZ    0x01  /* 1 Hz */
#define HTS221_CTRL_REG1_ODR_7HZ    0x02  /* 7 Hz */
#define HTS221_CTRL_REG1_ODR_12_5HZ 0x03  /* 12.5 Hz */

/* CTRL_REG2 Bit Definitions -------------------------------------------------*/
#define HTS221_CTRL_REG2_BOOT       0x80  /* Reboot memory content */
#define HTS221_CTRL_REG2_HEATER     0x02  /* Heater enable */
#define HTS221_CTRL_REG2_ONE_SHOT   0x01  /* One-shot enable */

/* STATUS_REG Bit Definitions ------------------------------------------------*/
#define HTS221_STATUS_H_DA          0x02  /* Humidity data available */
#define HTS221_STATUS_T_DA          0x01  /* Temperature data available */

/* Data Types ----------------------------------------------------------------*/

/**
 * @brief HTS221 initialization status
 */
typedef enum {
    HTS221_OK = 0,
    HTS221_ERROR = 1,
    HTS221_TIMEOUT = 2,
    HTS221_INVALID_WHO_AM_I = 3
} HTS221_Status_t;

/**
 * @brief I2C multi-byte read function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to read from
 * @param data Pointer to buffer for received data
 * @param size Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @retval Number of bytes read
 */
typedef uint8_t (*HTS221_ReadMulti_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t size, uint32_t timeout_ms);

/**
 * @brief I2C multi-byte write function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to write to
 * @param data Pointer to data to write
 * @param size Number of bytes to write
 * @param timeout_ms Timeout in milliseconds
 * @retval Number of bytes written
 */
typedef uint8_t (*HTS221_WriteMulti_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t size, uint32_t timeout_ms);

/**
 * @brief I2C single-byte write function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to write to
 * @param data Data byte to write
 * @param timeout_ms Timeout in milliseconds
 * @retval 1 on success, 0 on error
 */
typedef uint8_t (*HTS221_WriteByte_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t data, uint32_t timeout_ms);

/**
 * @brief I2C single-byte read function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to read from
 * @param timeout_ms Timeout in milliseconds
 * @retval Read byte value
 */
typedef uint8_t (*HTS221_ReadByte_t)(uint8_t slave_addr, uint8_t reg_addr, uint32_t timeout_ms);

/**
 * @brief HTS221 calibration data structure
 */
typedef struct {
    int16_t H0_T0_OUT;    /* Humidity calibration point 0 */
    int16_t H1_T0_OUT;    /* Humidity calibration point 1 */
    int16_t T0_OUT;       /* Temperature calibration point 0 */
    int16_t T1_OUT;       /* Temperature calibration point 1 */
    uint8_t H0_RH;        /* Humidity calibration value 0 (in %RH) */
    uint8_t H1_RH;        /* Humidity calibration value 1 (in %RH) */
    uint16_t T0_DEGC;     /* Temperature calibration value 0 (in °C * 8) */
    uint16_t T1_DEGC;     /* Temperature calibration value 1 (in °C * 8) */
} HTS221_Calibration_t;

/**
 * @brief HTS221 driver context structure
 */
typedef struct {
    uint8_t i2c_address;              /* I2C device address */
    HTS221_Calibration_t calibration; /* Calibration data */
} HTS221_t;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Register I2C functions for all HTS221 instances
 * @param read_multi Pointer to multi-byte read function
 * @param write_multi Pointer to multi-byte write function
 * @param write_byte Pointer to single-byte write function
 * @param read_byte Pointer to single-byte read function
 * @retval HTS221_OK on success, HTS221_ERROR if any pointer is NULL
 * 
 * This function must be called once before using any HTS221 sensor instance.
 * The registered functions will be used by all sensor instances.
 */
HTS221_Status_t HTS221_RegisterIOFunctions(HTS221_ReadMulti_t read_multi, 
                                           HTS221_WriteMulti_t write_multi, 
                                           HTS221_WriteByte_t write_byte,
                                           HTS221_ReadByte_t read_byte);

/**
 * @brief Initialize HTS221 driver context
 * @param hts Pointer to HTS221 context structure
 * @retval HTS221_OK on success
 */
HTS221_Status_t HTS221_InitDriver(HTS221_t *hts);

/**
 * @brief Check WHO_AM_I register to verify sensor presence
 * @param hts Pointer to HTS221 context structure
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK if WHO_AM_I matches expected value, error otherwise
 */
HTS221_Status_t HTS221_CheckWhoAmI(HTS221_t *hts, uint32_t timeout_ms);

/**
 * @brief Read and store calibration data from sensor
 * @param hts Pointer to HTS221 context structure
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 * 
 * This function must be called before reading temperature or humidity.
 */
HTS221_Status_t HTS221_ReadCalibration(HTS221_t *hts, uint32_t timeout_ms);

/**
 * @brief Initialize HTS221 sensor with default configuration
 * @param hts Pointer to HTS221 context structure
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 *
 * Default configuration:
 * - Power-down mode disabled (active)
 * - Output data rate: 1 Hz
 * - Block data update enabled
 * - Reads and stores calibration data
 */
HTS221_Status_t HTS221_Init(HTS221_t *hts, uint32_t timeout_ms);

/**
 * @brief Read raw humidity value from sensor
 * @param hts Pointer to HTS221 context structure
 * @param humidity Pointer to store raw humidity value (16-bit)
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 */
HTS221_Status_t HTS221_ReadHumidityRaw(HTS221_t *hts, int16_t *humidity, uint32_t timeout_ms);

/**
 * @brief Read humidity value in %RH (relative humidity)
 * @param hts Pointer to HTS221 context structure
 * @param humidity_rh Pointer to store humidity in %RH
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 * 
 * Calibration data must be loaded before calling this function.
 */
HTS221_Status_t HTS221_ReadHumidity(HTS221_t *hts, float *humidity_rh, uint32_t timeout_ms);

/**
 * @brief Read raw temperature value from sensor
 * @param hts Pointer to HTS221 context structure
 * @param temperature Pointer to store raw temperature value (16-bit)
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 */
HTS221_Status_t HTS221_ReadTemperatureRaw(HTS221_t *hts, int16_t *temperature, uint32_t timeout_ms);

/**
 * @brief Read temperature value in degrees Celsius
 * @param hts Pointer to HTS221 context structure
 * @param temperature_C Pointer to store temperature in °C
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 * 
 * Calibration data must be loaded before calling this function.
 */
HTS221_Status_t HTS221_ReadTemperature(HTS221_t *hts, float *temperature_C, uint32_t timeout_ms);

/**
 * @brief Read status register
 * @param hts Pointer to HTS221 context structure
 * @param status Pointer to store status register value
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 */
HTS221_Status_t HTS221_ReadStatus(HTS221_t *hts, uint8_t *status, uint32_t timeout_ms);

/**
 * @brief Write to a single register
 * @param hts Pointer to HTS221 context structure
 * @param reg_addr Register address
 * @param value Value to write
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 */
HTS221_Status_t HTS221_WriteRegister(HTS221_t *hts, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms);

/**
 * @brief Read from a single register
 * @param hts Pointer to HTS221 context structure
 * @param reg_addr Register address
 * @param value Pointer to store read value
 * @param timeout_ms Timeout in milliseconds
 * @retval HTS221_OK on success, error code otherwise
 */
HTS221_Status_t HTS221_ReadRegister(HTS221_t *hts, uint8_t reg_addr, uint8_t *value, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* HTS221_H_ */
