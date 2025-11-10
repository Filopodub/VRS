/**
  ******************************************************************************
  * @file    LPS22HB.h
  * @brief   LPS22HB pressure sensor driver header
  ******************************************************************************
  * @attention
  *
  * LPS22HB MEMS pressure sensor: 260-1260 hPa absolute digital output barometer
  * Datasheet: https://www.st.com/resource/en/datasheet/lps22hb.pdf
  *
  ******************************************************************************
  */

#ifndef LPS22HB_H_
#define LPS22HB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* LPS22HB I2C Address -------------------------------------------------------*/
#define LPS22HB_I2C_ADDRESS         0xBA  /* 7-bit address (0x5D) shifted left by 1 */

/* LPS22HB Register Map ------------------------------------------------------*/
#define LPS22HB_REG_INTERRUPT_CFG   0x0B  /* Interrupt configuration */
#define LPS22HB_REG_THS_P_L         0x0C  /* Pressure threshold (LSB) */
#define LPS22HB_REG_THS_P_H         0x0D  /* Pressure threshold (MSB) */

#define LPS22HB_REG_WHO_AM_I        0x0F  /* Device identification register */
#define LPS22HB_WHO_AM_I_VALUE      0xB1  /* Expected WHO_AM_I value for LPS22HB */

#define LPS22HB_REG_CTRL_REG1       0x10  /* Control register 1 */
#define LPS22HB_REG_CTRL_REG2       0x11  /* Control register 2 */
#define LPS22HB_REG_CTRL_REG3       0x12  /* Control register 3 */

#define LPS22HB_REG_FIFO_CTRL       0x14  /* FIFO control register */

#define LPS22HB_REG_REF_P_XL        0x15  /* Reference pressure register (LSB) */
#define LPS22HB_REG_REF_P_L         0x16  /* Reference pressure register (middle) */
#define LPS22HB_REG_REF_P_H         0x17  /* Reference pressure register (MSB) */

#define LPS22HB_REG_RPDS_L          0x18  /* Pressure offset (LSB) */
#define LPS22HB_REG_RPDS_H          0x19  /* Pressure offset (MSB) */

#define LPS22HB_REG_RES_CONF        0x1A  /* Resolution configuration */

#define LPS22HB_REG_INT_SOURCE      0x25  /* Interrupt source */

#define LPS22HB_REG_FIFO_STATUS     0x26  /* FIFO status register */
#define LPS22HB_REG_STATUS_REG      0x27  /* Status register */

#define LPS22HB_REG_PRESS_OUT_XL    0x28  /* Pressure output (LSB) */
#define LPS22HB_REG_PRESS_OUT_L     0x29  /* Pressure output (middle) */
#define LPS22HB_REG_PRESS_OUT_H     0x2A  /* Pressure output (MSB) */

#define LPS22HB_REG_TEMP_OUT_L      0x2B  /* Temperature output (LSB) */
#define LPS22HB_REG_TEMP_OUT_H      0x2C  /* Temperature output (MSB) */

#define LPS22HB_REG_LPFP_RES        0x33  /* Low-pass filter reset register */

/* CTRL_REG1 Bit Definitions -------------------------------------------------*/
#define LPS22HB_CTRL_REG1_ODR_MASK  0x70  /* Output data rate mask */
#define LPS22HB_CTRL_REG1_ODR_ONE_SHOT 0x00  /* One-shot mode / Power-down */
#define LPS22HB_CTRL_REG1_ODR_1HZ   0x10  /* 1 Hz */
#define LPS22HB_CTRL_REG1_ODR_10HZ  0x20  /* 10 Hz */
#define LPS22HB_CTRL_REG1_ODR_25HZ  0x30  /* 25 Hz */
#define LPS22HB_CTRL_REG1_ODR_50HZ  0x40  /* 50 Hz */
#define LPS22HB_CTRL_REG1_ODR_75HZ  0x50  /* 75 Hz */
#define LPS22HB_CTRL_REG1_EN_LPFP   0x08  /* Enable low-pass filter */
#define LPS22HB_CTRL_REG1_LPFP_CFG  0x04  /* Low-pass filter configuration */
#define LPS22HB_CTRL_REG1_BDU       0x02  /* Block data update */
#define LPS22HB_CTRL_REG1_SIM       0x01  /* SPI serial interface mode */

/* CTRL_REG2 Bit Definitions -------------------------------------------------*/
#define LPS22HB_CTRL_REG2_BOOT      0x80  /* Reboot memory content */
#define LPS22HB_CTRL_REG2_FIFO_EN   0x40  /* FIFO enable */
#define LPS22HB_CTRL_REG2_STOP_ON_FTH 0x20  /* Stop on FIFO watermark */
#define LPS22HB_CTRL_REG2_IF_ADD_INC 0x10  /* Register address auto-increment */
#define LPS22HB_CTRL_REG2_I2C_DIS   0x08  /* Disable I2C interface */
#define LPS22HB_CTRL_REG2_SWRESET   0x04  /* Software reset */
#define LPS22HB_CTRL_REG2_AUTO_ZERO 0x02  /* Autozero enable */
#define LPS22HB_CTRL_REG2_ONE_SHOT  0x01  /* One-shot enable */

/* STATUS_REG Bit Definitions ------------------------------------------------*/
#define LPS22HB_STATUS_P_DA         0x02  /* Pressure data available */
#define LPS22HB_STATUS_T_DA         0x01  /* Temperature data available */
#define LPS22HB_STATUS_P_OR         0x20  /* Pressure data overrun */
#define LPS22HB_STATUS_T_OR         0x10  /* Temperature data overrun */

/* Data Types ----------------------------------------------------------------*/

/**
 * @brief LPS22HB initialization status
 */
typedef enum {
    LPS22HB_OK = 0,
    LPS22HB_ERROR = 1,
    LPS22HB_TIMEOUT = 2,
    LPS22HB_INVALID_WHO_AM_I = 3
} LPS22HB_Status_t;

/**
 * @brief I2C multi-byte read function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to read from
 * @param data Pointer to buffer for received data
 * @param size Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @retval Number of bytes read
 */
typedef uint8_t (*LPS22HB_ReadMulti_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t size, uint32_t timeout_ms);

/**
 * @brief I2C multi-byte write function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to write to
 * @param data Pointer to data to write
 * @param size Number of bytes to write
 * @param timeout_ms Timeout in milliseconds
 * @retval Number of bytes written
 */
typedef uint8_t (*LPS22HB_WriteMulti_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t size, uint32_t timeout_ms);

/**
 * @brief I2C single-byte write function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to write to
 * @param data Data byte to write
 * @param timeout_ms Timeout in milliseconds
 * @retval 1 on success, 0 on error
 */
typedef uint8_t (*LPS22HB_WriteByte_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t data, uint32_t timeout_ms);

/**
 * @brief I2C single-byte read function type
 * @param slave_addr I2C slave address
 * @param reg_addr Register address to read from
 * @param timeout_ms Timeout in milliseconds
 * @retval Read byte value
 */
typedef uint8_t (*LPS22HB_ReadByte_t)(uint8_t slave_addr, uint8_t reg_addr, uint32_t timeout_ms);

/**
 * @brief LPS22HB driver context structure
 */
typedef struct {
    uint8_t i2c_address;           /* I2C device address */
} LPS22HB_t;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Register I2C functions for all LPS22HB instances
 * @param read_multi Pointer to multi-byte read function
 * @param write_multi Pointer to multi-byte write function
 * @param write_byte Pointer to single-byte write function
 * @param read_byte Pointer to single-byte read function
 * @retval LPS22HB_OK on success, LPS22HB_ERROR if any pointer is NULL
 * 
 * This function must be called once before using any LPS22HB sensor instance.
 * The registered functions will be used by all sensor instances.
 */
LPS22HB_Status_t LPS22HB_RegisterIOFunctions(LPS22HB_ReadMulti_t read_multi, 
                                             LPS22HB_WriteMulti_t write_multi, 
                                             LPS22HB_WriteByte_t write_byte,
                                             LPS22HB_ReadByte_t read_byte);

/**
 * @brief Initialize LPS22HB driver context
 * @param lps Pointer to LPS22HB context structure
 * @retval LPS22HB_OK on success
 */
LPS22HB_Status_t LPS22HB_InitDriver(LPS22HB_t *lps);

/**
 * @brief Check WHO_AM_I register to verify sensor presence
 * @param lps Pointer to LPS22HB context structure
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK if WHO_AM_I matches expected value, error otherwise
 */
LPS22HB_Status_t LPS22HB_CheckWhoAmI(LPS22HB_t *lps, uint32_t timeout_ms);

/**
 * @brief Initialize LPS22HB sensor with default configuration
 * @param lps Pointer to LPS22HB context structure
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 *
 * Default configuration:
 * - Power-down mode disabled (active)
 * - Output data rate: 25 Hz
 * - Block data update enabled
 */
LPS22HB_Status_t LPS22HB_Init(LPS22HB_t *lps, uint32_t timeout_ms);

/**
 * @brief Read raw pressure value from sensor
 * @param lps Pointer to LPS22HB context structure
 * @param pressure Pointer to store raw pressure value (24-bit)
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_ReadPressureRaw(LPS22HB_t *lps, int32_t *pressure, uint32_t timeout_ms);

/**
 * @brief Read pressure value in hPa (hectopascals)
 * @param lps Pointer to LPS22HB context structure
 * @param pressure_hPa Pointer to store pressure in hPa
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_ReadPressure(LPS22HB_t *lps, float *pressure_hPa, uint32_t timeout_ms);

/**
 * @brief Read raw temperature value from sensor
 * @param lps Pointer to LPS22HB context structure
 * @param temperature Pointer to store raw temperature value (16-bit)
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_ReadTemperatureRaw(LPS22HB_t *lps, int16_t *temperature, uint32_t timeout_ms);

/**
 * @brief Read temperature value in degrees Celsius
 * @param lps Pointer to LPS22HB context structure
 * @param temperature_C Pointer to store temperature in °C
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_ReadTemperature(LPS22HB_t *lps, float *temperature_C, uint32_t timeout_ms);

/**
 * @brief Read status register
 * @param lps Pointer to LPS22HB context structure
 * @param status Pointer to store status register value
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_ReadStatus(LPS22HB_t *lps, uint8_t *status, uint32_t timeout_ms);

/**
 * @brief Write to a single register
 * @param lps Pointer to LPS22HB context structure
 * @param reg_addr Register address
 * @param value Value to write
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_WriteRegister(LPS22HB_t *lps, uint8_t reg_addr, uint8_t value, uint32_t timeout_ms);

/**
 * @brief Read from a single register
 * @param lps Pointer to LPS22HB context structure
 * @param reg_addr Register address
 * @param value Pointer to store read value
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_ReadRegister(LPS22HB_t *lps, uint8_t reg_addr, uint8_t *value, uint32_t timeout_ms);

/**
 * @brief Set reference pressure (24-bit value)
 * @param lps Pointer to LPS22HB context structure
 * @param ref_pressure Reference pressure value (24-bit, LSB = 1/4096 hPa)
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 * 
 * This function writes to REF_P_XL, REF_P_L, and REF_P_H registers using multi-byte write.
 */
LPS22HB_Status_t LPS22HB_SetReferencePressure(LPS22HB_t *lps, int32_t ref_pressure, uint32_t timeout_ms);

/**
 * @brief Write multiple consecutive registers
 * @param lps Pointer to LPS22HB context structure
 * @param start_reg_addr Starting register address
 * @param data Pointer to data to write
 * @param size Number of bytes to write
 * @param timeout_ms Timeout in milliseconds
 * @retval LPS22HB_OK on success, error code otherwise
 */
LPS22HB_Status_t LPS22HB_WriteMultipleRegisters(LPS22HB_t *lps, uint8_t start_reg_addr, uint8_t *data, uint8_t size, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* LPS22HB_H_ */
