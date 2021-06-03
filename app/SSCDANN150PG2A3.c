/*******************************************************************************
*           Copyright (C) 2021 Michael R. Ferrara, All rights reserved.
*
*                       Santa Rosa, CA 95404
*                       Tel:(707)536-1330
*
* Filename:     SSCDANN150PG2A3.c
*
* Description: SSCDANN150PG2A3 "driver"
*
*******************************************************************************/
//#define TRACE_PRINT 1

#include "OSandPlatform.h"

#define GLOBAL_SSCDANN150PG2A3
#include "SSCDANN150PG2A3.h"

/* you must define the slave address. you can find it based on the part number:

    _SC_________XA_
    where X can be one of:

    S  - spi (this is not the right library for spi opperation)
    2  - i2c slave address 0x28
    3  - i2c slave address 0x38
    4  - i2c slave address 0x48
    5  - i2c slave address 0x58
    6  - i2c slave address 0x68
    7  - i2c slave address 0x78
*/

/// function that requests raw data from the sensor via i2c
///
/// input
///  slave_addr    - i2c slave addr of the sensor chip
/// output
///  raw           - struct containing 4 bytes of read data
/// returns
///         0 if all is fine
///         1 if chip is in command mode
///         2 if old data is being read
///         3 if a diagnostic fault is triggered in the chip
///         4 if the sensor is not hooked up
uint8_t ps_get_raw(const uint8_t slave_addr, struct cs_raw *raw)
{
    uint8_t val[4] = { 0, 0, 0, 0 };
    i2c_transfer7(I2C2, slave_addr, NULL, 0, val, 4);
    //Wire.requestFrom(slave_addr, (uint8_t) 4);
    //for (uint8_t i = 0; i <= 3; i++) {
    //    delay(4);                        // sensor might be missing, do not block
    //    val[i] = Wire.read();            // by using Wire.available()
    //}
    raw->status = (val[0] & 0xc0) >> 6;  // first 2 bits from first byte
    raw->bridge_data = ((val[0] & 0x3f) << 8) + val[1];
    raw->temperature_data = ((val[2] << 8) + (val[3] & 0xe0)) >> 5;
    if ( raw->temperature_data == 65535 ) return 4;
    return raw->status;
}


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
                   const float pressure_max)
{
    *pressure = 1.0 * (raw.bridge_data - output_min) * (pressure_max - pressure_min) / (output_max - output_min) + pressure_min;
    *temperature = (raw.temperature_data * 0.0977) - 50;
    return 0;
}


uint8_t measure_pressure( float *pressure, float *temperature)
{
  struct cs_raw raw;

  uint8_t status =  ps_get_raw(PRESSURE_SENS_ADDR, &raw);
  if (status!=0) {
    return(status);
  }
  status = ps_convert(raw, pressure, temperature, PS_OUT_MIN, PS_OUT_MAX, PS_PRESSURE_MIN, PS_PRESSURE_MAX);

  return(status);
}
