/**
  ******************************************************************************
  * @file    height_calculation.c
  * @brief   Height calculation from barometric pressure implementation
  ******************************************************************************
  * @attention
  *
  * This module provides functions to calculate relative height changes
  * using barometric pressure measurements. The calculation is based on
  * the International Standard Atmosphere (ISA) barometric formula.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "height_calculation.h"
#include <math.h>

/* Private Variables ---------------------------------------------------------*/
static float reference_pressure_hPa = 1013.25f;  /* Standard sea level pressure as default */

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Set the reference (starting) pressure for height calculation
 */
void Height_SetReferencePressure(float pressure_hPa)
{
    reference_pressure_hPa = pressure_hPa;
}

/**
 * @brief Calculate relative height from reference point
 */
float Height_GetRelativeHeight(float current_pressure_hPa)
{
    float height_meters;
    
    /* Barometric formula for altitude calculation:
     * h = 44330 * (1 - (P/P0)^0.1903)
     * 
     * Where:
     * - 44330 is a constant derived from ISA model (in meters)
     * - P is the current pressure
     * - P0 is the reference pressure
     * - 0.1903 ≈ 1/5.255 (adiabatic index for air)
     * 
     * This formula assumes:
     * - Standard temperature lapse rate
     * - Dry air
     * - Valid for altitudes up to ~11 km
     */
    
    height_meters = 44330.0f * (1.0f - powf(current_pressure_hPa / reference_pressure_hPa, 0.1903f));
    
    return height_meters;
}

/**
 * @brief Get the stored reference pressure
 */
float Height_GetReferencePressure(void)
{
    return reference_pressure_hPa;
}
