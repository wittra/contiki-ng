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
#include "dev/gpio-hal.h"
#include "bmg-250-sensor.h"
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
#define GYR_RANGE     0x43
#define CMD           0x7e
/*---------------------------------------------------------------------------*/
/* PMU Status register bits */
#define PMU_STATUS_SUSPEND      (0 << 2)
#define PMU_STATUS_NORMAL       (1 << 2)
#define PMU_STATUS_RESERVED     (2 << 2)
#define PMU_STATUS_FAST_STARTUP (3 << 2)
#define PMU_STATUS_MASK         (3 << 2)
/*---------------------------------------------------------------------------*/
/* Status register bits */
#define STATUS_DATA_READY       (1 << 6)
#define STATUS_FOC_COMPLETED    (1 << 3)
#define STATUS_SELFTEST_OK      (1 << 1)
#define STATUS_POWER_ON_DET     (1 << 0)
/*---------------------------------------------------------------------------*/
/* CMD register commands */
#define CMD_START_FOC           0x03
#define CMD_TMP_SUSP            0x10
#define CMD_TMP_NORM            0x11
#define CMD_GYRO_SUSP           0x14
#define CMD_GYRO_NORM           0x15
#define CMD_GYRO_FSUP           0x17
#define CMD_SOFT_RESET          0xb6
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
static uint8_t gyro_range;
/*---------------------------------------------------------------------------*/
const static gpio_hal_pin_t gyro_power_pin = BOARD_IOID_GYRO_POWER;
/*---------------------------------------------------------------------------*/
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3

static int16_t sensor_value[SENSOR_DATA_BUF_SIZE];
#define DATA_SIZE (SENSOR_DATA_BUF_SIZE * sizeof sensor_value[0])
/* ------------------------------------------------------------------------- */
/*
 * Wait SENSOR_BOOT_DELAY ticks for the sensor to boot and
 * SENSOR_STARTUP_DELAY for readings to be ready
 */
#define SENSOR_BOOT_DELAY     8
#define SENSOR_STARTUP_DELAY  5
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/* Wait for the MPU to have data ready */
static rtimer_clock_t t0;

/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 100
#define LAST_READING_TIME_DIFF 500
/*---------------------------------------------------------------------------*/
/**
 * \brief Set the range of the gyro
 * \param new_range: GYRO_RANGE_2000, GYRO_RANGE_1000, GYRO_RANGE_500, GYRO_RANGE_250, GYRO_RANGE_125
 * \return true if the write to the sensor succeeded
 */
static bool
gyro_set_range(uint8_t new_range)
{
  bool success = false;

  if(new_range == gyro_range) {
    return true;
  }

  /* Apply the range */
  SENSOR_SELECT();
  success = sensor_common_write_reg(GYR_RANGE, &new_range, 1);
  SENSOR_DESELECT();

  if(success) {
    gyro_range = new_range;
  }

  return success;
}
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  gyro_set_range(BMG_250_SENSOR_GYRO_RANGE);
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&bmg_250_sensor);
}
/*---------------------------------------------------------------------------*/
#if DEBUG
static bool
sensor_chipid(uint8_t *chipid)
{
  bool success = false;
  uint8_t read_seq[] = {0};
  SENSOR_SELECT();
  success = sensor_common_read_reg(CHIP_ID, read_seq, sizeof(read_seq));
  SENSOR_DESELECT();
  *chipid = read_seq[0];
  return success;
}
#endif
/*---------------------------------------------------------------------------*/
/**
 * \brief Exit low power mode
 */
static bool
sensor_wakeup(void)
{
  bool success = false;
  uint8_t write_seq[] = {CMD_GYRO_NORM};
  SENSOR_SELECT();
  success = sensor_common_write_reg(CMD, write_seq, sizeof(write_seq));
  SENSOR_DESELECT();
  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Exit low power mode
 */
static bool
sensor_sleep(void)
{
  bool success = false;
  uint8_t write_seq[] = {CMD_GYRO_SUSP};
  SENSOR_SELECT();
  success = sensor_common_write_reg(CMD, write_seq, sizeof(write_seq));
  SENSOR_DESELECT();
  return success;
}
/*---------------------------------------------------------------------------*/
static bool
power_up(void)
{
  bool success = false;
  sensor_wakeup();
  ctimer_set(&startup_timer, SENSOR_BOOT_DELAY, notify_ready, NULL);
  return success;
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
  success = sensor_common_read_reg(STATUS, &status, 1);
  SENSOR_DESELECT();
  return success && (status & STATUS_DATA_READY);
}
/*---------------------------------------------------------------------------*/
static bool
gyro_read(int16_t *data)
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
    success = sensor_common_read_reg(DATA_14, (uint8_t *)data, DATA_SIZE);
    SENSOR_DESELECT();
  }

  return success && ready;
}
/*---------------------------------------------------------------------------*/
static float
gyro_convert(int16_t raw_data)
{
  float v = 0.0;

  switch(gyro_range) {
  case BMG_250_SENSOR_GYRO_RANGE_2000:
    v = raw_data * 2000.0 / 32768.0;
    break;
  case BMG_250_SENSOR_GYRO_RANGE_1000:
    v = raw_data * 1000.0 / 32768.0;
    break;
  case BMG_250_SENSOR_GYRO_RANGE_500:
    v = raw_data * 500.0 / 32768.0;
    break;
  case BMG_250_SENSOR_GYRO_RANGE_250:
    v = raw_data * 250.0 / 32768.0;
    break;
  case BMG_250_SENSOR_GYRO_RANGE_125:
    v = raw_data * 125.0 / 32768.0;
    break;
  default:
    return 0.0;
  }

  return v;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type BMG_250_SENSOR_TYPE_GYRO_[XYZ]
 * \return centi-Deg/Sec (Gyro)
 */
static int
value(int type)
{
  bool success = false;
  int tries = 0;
  int rv;
  float v = 0.0;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("MPU: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));

  while(!success && (tries < 100)) {
    success = gyro_read(sensor_value);
    ++tries;
  }

  PRINTF("Read sensor values: %d, %d, %d (%s)\n",
         sensor_value[0], sensor_value[1], sensor_value[2],
         success?"valid":"not valid");

  if(!success) {
    return INT_MIN;
  }

  if(type == BMG_250_SENSOR_TYPE_GYRO_X) {
    v = gyro_convert(sensor_value[0]);
  } else if(type == BMG_250_SENSOR_TYPE_GYRO_Y) {
    v = gyro_convert(sensor_value[1]);
  } else if(type == BMG_250_SENSOR_TYPE_GYRO_Z) {
    v = gyro_convert(sensor_value[2]);
  }

  /* Scaling to centi deg/s and rounding fix */
  v *= 100.0;
  rv = (int)(v + 0.5);

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
    PRINTF("BMG: HW Init\n");
    gpio_hal_arch_set_pin(gyro_power_pin);
    gpio_hal_arch_pin_set_output(gyro_power_pin);
#if DEBUG
    {
      uint8_t chipid;
      sensor_chipid(&chipid);
      PRINTF("Chip id:0x%2.2x\n", chipid);
    }
#endif
    state = SENSOR_STATE_INITIALISED;
    break;
  case SENSORS_ACTIVE:
    if(enable) {
      PRINTF("BMG: Enabling\n");
      power_up();
      state = SENSOR_STATE_BOOTING;
    } else {
      PRINTF("BMG: Disabling\n");
      ctimer_stop(&startup_timer);
      sensor_sleep();
      state = SENSOR_STATE_DISABLED;
    }
    break;
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
SENSORS_SENSOR(bmg_250_sensor, "BMG250", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
