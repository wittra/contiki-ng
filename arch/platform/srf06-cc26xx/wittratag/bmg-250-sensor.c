/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup wittratag-cc26xx-bmg-sensor
 * @{
 *
 * \file
 *  Driver for the Wittratag Bosch BMG250 sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "bmg-250-sensor.h"
#include "sensor-common.h"
#include "board-i2c.h"

#include "ti-lib.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Sensor I2C address */
#define SENSOR_I2C_ADDRESS         0x68
/*---------------------------------------------------------------------------*/
/* Registers */
#define CHIP_ID       0x00
#define ERR_REG       0x02
#define PMU_STATUS    0x03
#define DATA_14       0x12
#define DATA_15       0x13
#define DATA_16       0x14
#define DATA_17       0x15
#define DATA_18       0x16
#define DATA_19       0x17
#define SENSORTIME_0  0x18
#define SENSORTIME_1  0x19
#define SENSORTIME_2  0x1a
#define STATUS        0x1b
#define INT_STATUS_1  0x1d
#define TEMPERATURE_0 0x20
#define TEMPERATURE_1 0x21
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, SENSOR_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_INITIALISED  1
#define SENSOR_STATE_BOOTING      2
#define SENSOR_STATE_ENABLED      3
/*---------------------------------------------------------------------------*/
static int state = SENSOR_STATE_DISABLED;
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type MPU_9250_SENSOR_TYPE_ACC_[XYZ] or MPU_9250_SENSOR_TYPE_GYRO_[XYZ]
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static int
value(int type)
{
  int rv;
  rv = 0;
  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the BMG250 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int
configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    break;
  case SENSORS_ACTIVE:
    if(enable) {
      PRINTF("BMG: Enabling\n");
    } else {
      PRINTF("BMG: Disabling\n");
    }
  default:
    break;
  }
  return state;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    break;
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(bmg_250_sensor, "BMG250", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
