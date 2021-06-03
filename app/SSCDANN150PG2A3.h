/*******************************************************************************
*           Copyright (C) 2021 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     SSCDANN150PG2A3.h
*
* Description: SSCDANN150PG2A3 "driver"
*
*******************************************************************************/
//#define TRACE_PRINT 1

#ifndef _SSCDANN150PG2A3_INCLUDED
#define _SSCDANN150PG2A3_INCLUDED

#include "OSandPlatform.h"

#ifdef GLOBAL_SSCDANN150PG2A3
#define SSCDANN150PG2A3GLOBAL
#define SSCDANN150PG2A3PRESET(A) = (A)
#else
#define SSCDANN150PG2A3PRESET(A)
#ifdef __cplusplus
#define SSCDANN150PG2A3GLOBAL extern "C"
#else
#define SSCDANN150PG2A3GLOBAL extern
#endif	/*__cplusplus*/
#endif				/*GLOBAL_SSCDANN150PG2A3 */

// ----------------------------------------------------------------
// PRIVATE API AND SUBJECT TO CHANGE!
// ----------------------------------------------------------------

// SSC150PG2A3 decodes as:
//Compensated temp range -20C->85C  0-150PSIGuage I2C addr 0x28 10%-90% Vsupply 2^14 count 3.3V
#define PRESSURE_SENS_ADDR 0x28

#define PS_OUT_MIN 0x666 //10%
#define PS_OUT_MAX 0x399A //90% of 2^14-1
#define PS_PRESSURE_MIN 0
#define PS_PRESSURE_MAX 150

struct cs_raw {
    uint8_t status;             // 2 bit
    uint16_t bridge_data;       // 14 bit
    uint16_t temperature_data;  // 11 bit
};

// ----------------------------------------------------------------
// PUBLIC API definition
// ----------------------------------------------------------------

/// function that requests raw data from the sensor via i2c
///
/// input
///  slave_addr    - i2c slave addr of the sensor chip
///
/// output
///  raw           - struct containing 4 bytes of read data
///
/// returns
///         0 if all is fine
///         1 if chip is in command mode
///         2 if old data is being read
///         3 if a diagnostic fault is triggered in the chip
///         4 if the sensor is not hooked up
uint8_t ps_get_raw(const uint8_t slave_addr, struct cs_raw *raw);

/// function that converts raw data read from the sensor into temperature and pressure values
///
/// input:
///  raw            - struct containing all 4 bytes read from the sensor
///  output_min     - output at minimal calibrated pressure (counts)
///  output_max     - output at maximum calibrated pressure (counts)
///  pressure_min   - minimal value of pressure range
///  pressure_max   - maxium value of pressure range
///
/// output:
///  pressure
///  temperature
uint8_t ps_convert(const struct cs_raw raw, float *pressure, float *temperature,
                   const uint16_t output_min, const uint16_t output_max, const float pressure_min,
                   const float pressure_max);


uint8_t measure_pressure( float *pressure, float *temperature);

#endif				//_SSCDANN150PG2A3_INCLUDED
