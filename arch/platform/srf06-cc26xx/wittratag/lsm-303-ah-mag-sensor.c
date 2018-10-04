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
 *  Driver for the Wittratag LSM303AH magnetic sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "dev/gpio-hal.h"
#include "lsm-303-ah-sensor.h"
#include "sensor-common.h"
#include "board-i2c.h"

#include <limits.h>
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
#define SENSOR_I2C_ADDRESS        0x1e /*<< Magnetometer */
/*---------------------------------------------------------------------------*/
/* Registers */
#define WHO_AM_I_M       0x4f
#define CFG_REG_A_M      0x60
#define CFG_REG_B_M      0x61
#define CFG_REG_C_M      0x62
#define INT_CTRL_REG_M   0x63
#define INT_SOURCE_REG_M 0x64
#define INT_THS_L_REG_M  0x65
#define INT_THS_H_REG_M  0x66
#define STATUS_REG_M     0x67
#define OUTX_L_REG_M     0x68
#define OUTX_H_REG_M     0x69
#define OUTY_L_REG_M     0x6A
#define OUTY_H_REG_M     0x6B
#define OUTZ_L_REG_M     0x6C
#define OUTZ_H_REG_M     0x6D
/*---------------------------------------------------------------------------*/
/* Configuration register A bits */
#define CFG_REG_A_LP (1<<4)
#define CFG_REG_A_ODR_10  (0 << 2)
#define CFG_REG_A_ODR_20  (1 << 2)
#define CFG_REG_A_ODR_50  (2 << 2)
#define CFG_REG_A_ODR_100 (3 << 2)
#define CFG_REG_A_MODE_CONT 0
#define CFG_REG_A_MODE_SING 1
#define CFG_REG_A_MODE_IDLE 2
/*---------------------------------------------------------------------------*/
/* Configuration register B bits */
#define OFF_CANCEL 1
/*---------------------------------------------------------------------------*/
/* Status register bits */
#define STATUS_XDA   0x01
#define STATUS_YDA   0x02
#define STATUS_ZDA   0x03
#define STATUS_ZYXDA 0x04
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, SENSOR_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Wait for the magnetometer to have data ready */
rtimer_clock_t t0;

/*
 * Wait timeout in rtimer ticks. This is quite a high value, since we reread
 * every time we want a X/Y/Z value.
 */
#define READING_WAIT_TIMEOUT 1000
/*---------------------------------------------------------------------------*/
#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_INITIALISED  1
#define SENSOR_STATE_BOOTING      2
#define SENSOR_STATE_ENABLED      3
/*---------------------------------------------------------------------------*/
static int state = SENSOR_STATE_DISABLED;
/* ------------------------------------------------------------------------- */
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3

static int16_t sensor_value[SENSOR_DATA_BUF_SIZE];
#define DATA_SIZE (SENSOR_DATA_BUF_SIZE * sizeof sensor_value[0])
/*---------------------------------------------------------------------------*/
#define SENSOR_STARTUP_DELAY 20
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
#if DEBUG
static bool
sensor_chipid(uint8_t *chipid)
{
  bool success = false;
  static uint8_t write_seq[] = {WHO_AM_I_M};
  static uint8_t read_seq[] = {0};
  SENSOR_SELECT();
  success = board_i2c_write_read(write_seq, sizeof(write_seq),
                                 read_seq, sizeof(read_seq));
  SENSOR_DESELECT();
  *chipid = read_seq[0];
  return success;
}
#endif /* DEBUG */
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  sensors_changed(&lsm_303_ah_mag_sensor);
}
/*---------------------------------------------------------------------------*/
static bool
sensor_wakeup(void)
{
  bool success = false;
  uint8_t write_seq[] = {CFG_REG_A_M,
                         CFG_REG_A_LP |
                         CFG_REG_A_ODR_100 |
                         CFG_REG_A_MODE_CONT};

  SENSOR_SELECT();
  success = board_i2c_write(write_seq, sizeof(write_seq));
  SENSOR_DESELECT();
  return success;
}
/*---------------------------------------------------------------------------*/
static bool
sensor_sleep(void)
{
  bool success = false;
  uint8_t write_seq[] = {CFG_REG_A_M, CFG_REG_A_LP | CFG_REG_A_MODE_IDLE};

  SENSOR_SELECT();
  success = board_i2c_write(write_seq, sizeof(write_seq));
  SENSOR_DESELECT();
  return success;
}
/*---------------------------------------------------------------------------*/
static void
power_up(void)
{
  bool success = sensor_wakeup();
  if(success) {
    ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Check if data is ready to be read.
 * \return Return true if there is data to be read.
 */
static bool
data_ready(void)
{
  bool success = false;
  uint8_t status;
  SENSOR_SELECT();
  success = sensor_common_read_reg(STATUS_REG_M, &status, 1);
  SENSOR_DESELECT();
  return success && (status & STATUS_ZYXDA);
}
/*---------------------------------------------------------------------------*/
static bool
mag_read(int16_t *data)
{
  bool success = false;
  bool ready = false;

  t0 = RTIMER_NOW();

  ready = data_ready();
  while(!ready &&
        (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT))) {
    ready = data_ready();
  }

  if(ready) {
    SENSOR_SELECT();
    success = sensor_common_read_reg(OUTX_L_REG_M, (uint8_t *)data, DATA_SIZE);
    SENSOR_DESELECT();
  }

  return success && ready;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type LSM_303_AH_SENSOR_TYPE_MAG_[XYZ]
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static int
value(int type)
{
  bool success = false;
  int rv = INT_MIN;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("LSM Mag: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));

  success = mag_read(sensor_value);

  PRINTF("LSM Mag sensor values: %d, %d, %d (%s)\n",
         sensor_value[0], sensor_value[1], sensor_value[2],
         success?"valid":"not valid");

  if(!success) {
    return INT_MIN;
  }

  if(type == LSM_303_AH_SENSOR_TYPE_MAG_X) {
    rv = sensor_value[0];
  } else if(type == LSM_303_AH_SENSOR_TYPE_MAG_Y) {
    rv = sensor_value[1];
  } else if(type == LSM_303_AH_SENSOR_TYPE_MAG_Z) {
    rv = sensor_value[2];
  }

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
  switch(type) {
  case SENSORS_HW_INIT:
    PRINTF("LSM Mag: HW Init\n");
#if DEBUG
    {
      uint8_t chipid;
      bool success = sensor_chipid(&chipid);
      PRINTF("Chip id:0x%2.2x (%s)\n", chipid, success?"SUCCESS":"FAIL");
    }
#endif
    state = SENSOR_STATE_INITIALISED;
    break;
  case SENSORS_ACTIVE:
    if(enable) {
      PRINTF("LSM Mag: Enabling\n");
      power_up();
      state = SENSOR_STATE_BOOTING;
    } else {
      ctimer_stop(&startup_timer);
      sensor_sleep();
      PRINTF("LSM Mag: Disabling\n");
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
SENSORS_SENSOR(lsm_303_ah_mag_sensor, "LSM303AH Mag", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
