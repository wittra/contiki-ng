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
 * \addtogroup wittratag-cc26xx-lsm303ah-sensor
 * @{
 *
 * \file
 *  Driver for the Wittratag LSM303AH accelerometer sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "dev/gpio-hal.h"
#include "lsm-303-ah-sensor.h"
#include "sensor-common.h"
#include "board-i2c.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Sensor I2C address */
#define SENSOR_I2C_ADDRESS        0x1d /*<< Accelerometer */
/*---------------------------------------------------------------------------*/
/* Registers */
#define WHO_AM_I      0x0f
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
#define SENSOR_STARTUP_DELAY 10
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  sensors_changed(&lsm_303_ah_acc_sensor);
}
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
 * \brief Configuration function for the LSM303AH sensor.
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
  uint8_t chip_id = 0;
  bool success = false;
  switch(type) {
  case SENSORS_HW_INIT:
    PRINTF("LSM: HW Init\n");
    break;
  case SENSORS_ACTIVE:
    {
      static uint8_t write_seq[] = {WHO_AM_I};
      static uint8_t read_seq[] = {0};
      SENSOR_SELECT();
      success = board_i2c_write_read(write_seq, sizeof(write_seq),
                                     read_seq, sizeof(read_seq));
      SENSOR_DESELECT();
      chip_id = read_seq[0];
    }
    PRINTF("Chip id 0x%2.2x (%s)\n", chip_id, success?"SUCCESS":"FAIL");
    if(enable) {
      PRINTF("LSM: Enabling\n");
      ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
    } else {
      ctimer_stop(&startup_timer);
      PRINTF("LSM: Disabling\n");
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
    return state;
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(lsm_303_ah_acc_sensor, "LSM303AH Acc", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
