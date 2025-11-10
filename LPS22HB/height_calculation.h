/**
  ******************************************************************************
  * @file    height_calculation.h
  * @brief   Height calculation from barometric pressure
  ******************************************************************************
  * @attention
  *
  * This module provides functions to calculate relative height changes
  * using barometric pressure measurements from the LPS25HB sensor.
  * Uses the barometric formula to calculate altitude differences.
  *
  ******************************************************************************
  */

#ifndef HEIGHT_CALCULATION_H_
#define HEIGHT_CALCULATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Set the reference (starting) pressure for height calculation
 * @param pressure_hPa Reference pressure in hectopascals (hPa)
 * 
 * This function stores the pressure at the starting point. All subsequent
 * height calculations will be relative to this reference pressure.
 */
void Height_SetReferencePressure(float pressure_hPa);

/**
 * @brief Calculate relative height from reference point
 * @param current_pressure_hPa Current pressure in hectopascals (hPa)
 * @retval Relative height in meters (positive = above reference, negative = below)
 * 
 * This function calculates the height difference between the current pressure
 * and the reference pressure using the barometric formula:
 * h = 44330 * (1 - (P/P0)^0.1903)
 * 
 * Where:
 * - h is the altitude difference in meters
 * - P is the current pressure
 * - P0 is the reference pressure
 */
float Height_GetRelativeHeight(float current_pressure_hPa);

/**
 * @brief Get the stored reference pressure
 * @retval Reference pressure in hectopascals (hPa)
 */
float Height_GetReferencePressure(void);

#ifdef __cplusplus
}
#endif

#endif /* HEIGHT_CALCULATION_H_ */
