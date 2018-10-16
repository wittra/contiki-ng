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
#define SENSOR_I2C_ADDRESS 0x1d /*<< Accelerometer */
/*---------------------------------------------------------------------------*/
/* Registers */
#define WHO_AM_I_A         0x0f
#define CTRL1_A            0x20
#define CTRL2_A            0x21
#define CTRL3_A            0x22
#define CTRL4_A            0x23
#define CTRL5_A            0x24
#define OUT_T_A            0x26
#define STATUS_A           0x27
#define OUT_X_L_A          0x28
#define OUT_X_H_A          0x29
#define OUT_Y_L_A          0x2a
#define OUT_Y_H_A          0x2b
#define OUT_Z_L_A          0x2c
#define OUT_Z_H_A          0x2d
#define TAP_6D_THS_A       0x31
#define INT_DUR_A          0x32
#define WAKE_UP_THS_A      0x33
#define STATUS_DUP_A       0x36
#define WAKEUP_SRC_A       0x37
#define TAP_SRC_A          0x38
#define SIXD_SRC_A         0x39
#define FUNC_CK_GATE_A     0x3d
#define FUNC_CTRL_A        0x3f
/*---------------------------------------------------------------------------*/
/* Control register 1 bits */
/* Power modes */
#define POWER_MODE_PD      0x00
#define POWER_MODE_LP_1    0x08
#define POWER_MODE_LP_12_5 0x09
#define POWER_MODE_LP_25   0x0a
#define POWER_MODE_LP_50   0x0b
#define POWER_MODE_LP_100  0x0c
#define POWER_MODE_LP_200  0x0d
#define POWER_MODE_LP_400  0x0e
#define POWER_MODE_LP_800  0x0f
#define POWER_MODE_HR_12_5 0x01
#define POWER_MODE_HR_25   0x02
#define POWER_MODE_HR_50   0x03
#define POWER_MODE_HR_100  0x04
#define POWER_MODE_HR_200  0x05
#define POWER_MODE_HR_400  0x06
#define POWER_MODE_HR_800  0x07
/* Full scale selection */
#define FULL_SCALE_2G      0x00
#define FULL_SCALE_16G     0x01
#define FULL_SCALE_4G      0x02
#define FULL_SCALE_8G      0x03
/*---------------------------------------------------------------------------*/
/* Control register 2 bits */
#define SOFT_RESET         (1 << 6)
#define FUNC_CFG_EN        (1 << 4)
#define IF_ADD_INC         (1 << 2)
/*---------------------------------------------------------------------------*/
/* Control register 3 bits */
#define TAP_X_EN           (1 << 5)
#define TAP_Y_EN           (1 << 4)
#define TAP_Z_EN           (1 << 3)
#define LIR                (1 << 2)
#define H_LACTIVE          (1 << 1)
#define PP_OD              (1 << 0)
/*---------------------------------------------------------------------------*/
/* Control register 4 bits */
#define INT1_S_TAP         (1 << 6)
#define INT1_FF            (1 << 4)
#define INT1_TAP           (1 << 3)
#define INT1_6D            (1 << 2)
/*---------------------------------------------------------------------------*/
/* Control register 5 bits */
#define DRDY_PULSED        (1 << 7)
#define INT2_BOOT          (1 << 6)
#define INT2_ON_INT1       (1 << 5)
#define INT2_TILT          (1 << 4)
#define INT2_SIG_MOT       (1 << 3)
#define INT2_STEP          (1 << 2)
#define INT2_FTH           (1 << 1)
#define INT2_DRDY          (1 << 0)
/*---------------------------------------------------------------------------*/
#define TEMPERATURE_SENSOR_OFFSET 25
/*---------------------------------------------------------------------------*/
/* FUNC_CK_GATE_A register bits */
#define TILT_INT           (1 << 7)
#define FS_SRC_1           (1 << 6)
#define FS_SRC_0           (1 << 5)
#define SIG_MOT_DETECT     (1 << 4)
#define RST_SIGN_MOT       (1 << 3)
#define RST_PEDO           (1 << 2)
#define STEP_DETECT        (1 << 1)
#define CK_GATE_FUNC       (1 << 0)
/*---------------------------------------------------------------------------*/
/* Status register bits */
#define STATUS_DRDY        0x01
/*---------------------------------------------------------------------------*/
/* Function control register bits */
#define MODULE_ON          (1 << 5)
#define TILT_ON            (1 << 4)
#define SIGN_MOT_ON        (1 << 1)
#define STEP_CNT_ON        (1 << 0)
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, SENSOR_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Wait for the accelerometer to have data ready */
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
static int mot_state = SENSOR_STATE_DISABLED;
/*---------------------------------------------------------------------------*/
const static gpio_hal_pin_t acc_int_pin = BOARD_IOID_ACC_INT;
/* ------------------------------------------------------------------------- */
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3

static int16_t sensor_value[SENSOR_DATA_BUF_SIZE];
#define DATA_SIZE (SENSOR_DATA_BUF_SIZE * sizeof sensor_value[0])
/*---------------------------------------------------------------------------*/
#define SENSOR_STARTUP_DELAY 10
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
#if DEBUG
static bool
sensor_chipid(uint8_t *chipid)
{
  bool success = false;
  static uint8_t write_seq[] = {WHO_AM_I_A};
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
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&lsm_303_ah_acc_sensor);
}
/*---------------------------------------------------------------------------*/
static bool
sensor_wakeup(void)
{
  bool success = false;
  uint8_t write_seq[] = {CTRL1_A,
                         (POWER_MODE_LP_50 << 4) | (FULL_SCALE_2G << 2)};

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
  uint8_t write_seq[] = {CTRL1_A, POWER_MODE_PD};

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
static bool
mot_sensor_wakeup(void)
{
  bool success = false;

  SENSOR_SELECT();
  {
    uint8_t write_seq[] = { TAP_6D_THS_A, 0x40 };
    success = board_i2c_write(write_seq, sizeof(write_seq));
  }
  {
    uint8_t write_seq[] = { CTRL3_A, LIR | H_LACTIVE };
    success &= board_i2c_write(write_seq, sizeof(write_seq));
  }
  SENSOR_DESELECT();
  return success;
}
/*---------------------------------------------------------------------------*/
static bool
mot_sensor_enable(void)
{
  bool success = false;
  uint8_t write_seq[] = { CTRL4_A, INT1_6D };
  SENSOR_SELECT();
  success = board_i2c_write(write_seq, sizeof(write_seq));
  SENSOR_DESELECT();
  return success;
}
/*---------------------------------------------------------------------------*/
static bool
mot_sensor_disable(void)
{
  bool success = false;
  uint8_t write_seq[] = { CTRL4_A, 0 };
  SENSOR_SELECT();
  success = board_i2c_write(write_seq, sizeof(write_seq));
  SENSOR_DESELECT();
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
  uint8_t status = 0;
  SENSOR_SELECT();
  success = sensor_common_read_reg(STATUS_A, &status, 1);
  SENSOR_DESELECT();
  return success && (status & STATUS_DRDY);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Clear interrupt by reading interrupt status registers
 * \return Return false if clearing fails.
 */
static bool
clear_interrupt(void)
{
  bool success = false;
  uint8_t answer[4] = { 0 };

  SENSOR_SELECT();
  success = sensor_common_read_reg(STATUS_DUP_A, answer, 4);
  SENSOR_DESELECT();
#if DEBUG
  printf("SR:");
  for (int i = 0; i < 4; i++) {
    printf("0x%2.2x ", answer[i]);
  }
  printf("\n");
#endif
  return success;
}
/*---------------------------------------------------------------------------*/
static bool
acc_read(int16_t *data)
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
    success = sensor_common_read_reg(OUT_X_L_A, (uint8_t *)data, DATA_SIZE);
    SENSOR_DESELECT();
  }

  return success && ready;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns temperature reading from the sensor
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static bool
read_tmp(uint8_t *tmp)
{
  bool success = false;

  SENSOR_SELECT();
  success = sensor_common_read_reg(OUT_T_A, tmp, 1);
  SENSOR_DESELECT();

  *tmp += TEMPERATURE_SENSOR_OFFSET; /* Offset compensation */

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type LSM_303_AH_SENSOR_TYPE_ACC_[XYZ]
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static int
value(int type)
{
  bool success = false;
  int rv = INT_MIN;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("LSM Acc: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));

  if(type == LSM_303_AH_SENSOR_TYPE_TMP) {
    uint8_t tmp = 0;
    success = read_tmp(&tmp);
    if(success) {
      return tmp;
    }
    return INT_MIN;
  }

  success = acc_read(sensor_value);

  PRINTF("LSM Acc sensor values: %d, %d, %d (%s)\n",
         sensor_value[0], sensor_value[1], sensor_value[2],
         success?"valid":"not valid");

  if(!success) {
    return INT_MIN;
  }

  if(type == LSM_303_AH_SENSOR_TYPE_ACC_X) {
    rv = sensor_value[0];
  } else if(type == LSM_303_AH_SENSOR_TYPE_ACC_Y) {
    rv = sensor_value[1];
  } else if(type == LSM_303_AH_SENSOR_TYPE_ACC_Z) {
    rv = sensor_value[2];
  }

  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Handler for WittraTag-CC1350 LSM303AH interrupts
 */
static void
lsm_interrupt_handler(gpio_hal_pin_mask_t pin_mask)
{
  if(clear_interrupt()) {
    sensors_changed(&lsm_303_ah_mot_sensor);
  }
}
/*---------------------------------------------------------------------------*/
/* Event handler definitions for LSM Significant motion. */
static gpio_hal_event_handler_t lsm_motion_event_handler = {
  .handler = lsm_interrupt_handler,
  .pin_mask = gpio_hal_pin_to_mask(BOARD_IOID_ACC_INT),
};
static bool int_handler_registered = false;
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
    PRINTF("LSM Acc: HW Init\n");
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
      PRINTF("LSM Acc: Enabling\n");
      power_up();
      state = SENSOR_STATE_BOOTING;
    } else {
      PRINTF("LSM Acc: Disabling\n");
      ctimer_stop(&startup_timer);
      sensor_sleep();
      state = SENSOR_STATE_DISABLED;
    }
  default:
    break;
  }
  return state;
}
/*---------------------------------------------------------------------------*/
static int
configure_mot(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    PRINTF("LSM Mot: HW Init\n");
    mot_sensor_wakeup();
    gpio_hal_arch_pin_set_input(acc_int_pin);
    gpio_hal_arch_pin_cfg_set(acc_int_pin,
                              GPIO_HAL_PIN_CFG_PULL_UP |
                              GPIO_HAL_PIN_CFG_INT_ENABLE |
                              GPIO_HAL_PIN_CFG_EDGE_FALLING);

    mot_state = SENSOR_STATE_INITIALISED;
    break;
  case SENSORS_ACTIVE:
    if(!int_handler_registered) {
      PRINTF("LSM Mot: Registring\n");
      /* Register interrupt vector */
      gpio_hal_register_handler(&lsm_motion_event_handler);
      gpio_hal_arch_interrupt_disable(acc_int_pin);
      int_handler_registered = true;
    }
    if(enable) {
      PRINTF("LSM Mot: Enabling\n");
      if(mot_state != SENSOR_STATE_ENABLED) {
        mot_sensor_enable();
        clear_interrupt(); /* Clear interrupt before enabling */
        gpio_hal_arch_interrupt_enable(acc_int_pin);
        mot_state = SENSOR_STATE_ENABLED;
      }
    } else {
      PRINTF("LSM Mot: Disabling\n");
      gpio_hal_arch_interrupt_disable(acc_int_pin);
      mot_sensor_disable();
      mot_state = SENSOR_STATE_DISABLED;
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
static int
mot_status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return gpio_hal_arch_read_pin(acc_int_pin);
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(lsm_303_ah_acc_sensor, "LSM303AH Acc", value, configure, status);
SENSORS_SENSOR(lsm_303_ah_mot_sensor, "LSM303AH Mot", NULL, configure_mot, mot_status);
/*---------------------------------------------------------------------------*/
/** @} */
