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
 * \addtogroup sensortag-cc26xx-mpu
 * @{
 *
 * \file
 *  Driver for the Sensortag Invensense MPU9250 motion processing unit
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "mpu-9250-wakeup-sensor.h"
#include "sys/rtimer.h"
#include "sensor-common.h"
#include "board-i2c.h"
#include "dev/gpio-hal.h"

#include "ti-lib.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#define MPU_INT_IO_CFG          (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO |       \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  |       \
                                 IOC_HYST_DISABLE | IOC_FALLING_EDGE  |       \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL |       \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
#define MPU_INT_PIN                   BOARD_IOID_MPU_INT
/*---------------------------------------------------------------------------*/
/* Sensor I2C address */
#define SENSOR_I2C_ADDRESS            0x68
#define SENSOR_MAG_I2_ADDRESS         0x0C
/*---------------------------------------------------------------------------*/
/* Registers */
#define SELF_TEST_X_GYRO              0x00 /* R/W */
#define SELF_TEST_Y_GYRO              0x01 /* R/W */
#define SELF_TEST_Z_GYRO              0x02 /* R/W */
#define SELF_TEST_X_ACCEL             0x0D /* R/W */
#define SELF_TEST_Z_ACCEL             0x0E /* R/W */
#define SELF_TEST_Y_ACCEL             0x0F /* R/W */
/*---------------------------------------------------------------------------*/
#define XG_OFFSET_H                   0x13 /* R/W */
#define XG_OFFSET_L                   0x14 /* R/W */
#define YG_OFFSET_H                   0x15 /* R/W */
#define YG_OFFSET_L                   0x16 /* R/W */
#define ZG_OFFSET_H                   0x17 /* R/W */
#define ZG_OFFSET_L                   0x18 /* R/W */
/*---------------------------------------------------------------------------*/
#define SMPLRT_DIV                    0x19 /* R/W */
#define CONFIG                        0x1A /* R/W */
#define GYRO_CONFIG                   0x1B /* R/W */
#define ACCEL_CONFIG                  0x1C /* R/W */
#define ACCEL_CONFIG_2                0x1D /* R/W */
#define LP_ACCEL_ODR                  0x1E /* R/W */
#define WOM_THR                       0x1F /* R/W */
#define FIFO_EN                       0x23 /* R/W */
/*---------------------------------------------------------------------------*/
/*
 * Registers 0x24 - 0x36 are not applicable to the SensorTag HW configuration
 * (IC2 Master)
 */
#define INT_PIN_CFG                   0x37 /* R/W */
#define INT_ENABLE                    0x38 /* R/W */
#define INT_STATUS                    0x3A /* R */
#define ACCEL_XOUT_H                  0x3B /* R */
#define ACCEL_XOUT_L                  0x3C /* R */
#define ACCEL_YOUT_H                  0x3D /* R */
#define ACCEL_YOUT_L                  0x3E /* R */
#define ACCEL_ZOUT_H                  0x3F /* R */
#define ACCEL_ZOUT_L                  0x40 /* R */
#define TEMP_OUT_H                    0x41 /* R */
#define TEMP_OUT_L                    0x42 /* R */
#define GYRO_XOUT_H                   0x43 /* R */
#define GYRO_XOUT_L                   0x44 /* R */
#define GYRO_YOUT_H                   0x45 /* R */
#define GYRO_YOUT_L                   0x46 /* R */
#define GYRO_ZOUT_H                   0x47 /* R */
#define GYRO_ZOUT_L                   0x48 /* R */
/*---------------------------------------------------------------------------*/
#define WOM_INT_MASK                  0x40
/*---------------------------------------------------------------------------*/
/*
 * Registers 0x49 - 0x60 are not applicable to the SensorTag HW configuration
 * (external sensor data)
 *
 * Registers 0x63 - 0x67 are not applicable to the SensorTag HW configuration
 * (I2C master)
 */
#define SIGNAL_PATH_RESET             0x68 /* R/W */
#define ACCEL_INTEL_CTRL              0x69 /* R/W */
#define USER_CTRL                     0x6A /* R/W */
#define PWR_MGMT_1                    0x6B /* R/W */
#define PWR_MGMT_2                    0x6C /* R/W */
#define FIFO_COUNT_H                  0x72 /* R/W */
#define FIFO_COUNT_L                  0x73 /* R/W */
#define FIFO_R_W                      0x74 /* R/W */
#define WHO_AM_I                      0x75 /* R/W */
/*---------------------------------------------------------------------------*/
/* Masks is mpuConfig valiable */
#define ACC_CONFIG_MASK               0x38
#define GYRO_CONFIG_MASK              0x07
/*---------------------------------------------------------------------------*/
/* Values PWR_MGMT_1 */
#define MPU_SLEEP                     0x4F  /* Sleep + stop all clocks */
#define MPU_WAKE_UP                   0x09  /* Disable temp. + intern osc */
/*---------------------------------------------------------------------------*/
/* Values PWR_MGMT_2 */
#define ALL_AXES                      0x3F
#define GYRO_AXES                     0x07
#define ACC_AXES                      0x38
/*---------------------------------------------------------------------------*/
/* Data sizes */
#define DATA_SIZE                     6
/*---------------------------------------------------------------------------*/
/* Output data rates */
#define INV_LPA_0_3125HZ              0
#define INV_LPA_0_625HZ               1
#define INV_LPA_1_25HZ                2
#define INV_LPA_2_5HZ                 3
#define INV_LPA_5HZ                   4
#define INV_LPA_10HZ                  5
#define INV_LPA_20HZ                  6
#define INV_LPA_40HZ                  7
#define INV_LPA_80HZ                  8
#define INV_LPA_160HZ                 9
#define INV_LPA_320HZ                 10
#define INV_LPA_640HZ                 11
#define INV_LPA_STOPPED               255
/*---------------------------------------------------------------------------*/
/* Bit values */
#define BIT_ANY_RD_CLR                0x10
#define BIT_RAW_RDY_EN                0x01
#define BIT_WOM_EN                    0x40
#define BIT_LPA_CYCLE                 0x20
#define BIT_STBY_XA                   0x20
#define BIT_STBY_YA                   0x10
#define BIT_STBY_ZA                   0x08
#define BIT_STBY_XG                   0x04
#define BIT_STBY_YG                   0x02
#define BIT_STBY_ZG                   0x01
#define BIT_STBY_XYZA                 (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG                 (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
/*---------------------------------------------------------------------------*/
/* User control register */
#define BIT_ACTL                      0x80
#define BIT_LATCH_EN                  0x20
/*---------------------------------------------------------------------------*/
/* INT Pin / Bypass Enable Configuration */
#define BIT_AUX_IF_EN                 0x20 /* I2C_MST_EN */
#define BIT_BYPASS_EN                 0x02
/*---------------------------------------------------------------------------*/
#define ACC_RANGE_INVALID -1

#define ACC_RANGE_2G      0
#define ACC_RANGE_4G      1
#define ACC_RANGE_8G      2
#define ACC_RANGE_16G     3

#define MPU_AX_GYR_X      2
#define MPU_AX_GYR_Y      1
#define MPU_AX_GYR_Z      0
#define MPU_AX_GYR        0x07

#define MPU_AX_ACC_X      5
#define MPU_AX_ACC_Y      4
#define MPU_AX_ACC_Z      3
#define MPU_AX_ACC        0x38

#define MPU_AX_MAG        6
/*---------------------------------------------------------------------------*/
#define LPOSC_CLK_0_24    0
#define LPOSC_CLK_0_49    1
#define LPOSC_CLK_0_98    2
#define LPOSC_CLK_1_95    3
#define LPOSC_CLK_3_91    4
#define LPOSC_CLK_7_81    5
#define LPOSC_CLK_15_63   6
#define LPOSC_CLK_31_25   7
#define LPOSC_CLK_62_50   8
#define LPOSC_CLK_125     9
#define LPOSC_CLK_250     10
#define LPOSC_CLK_500     11
/*---------------------------------------------------------------------------*/
#define WOM_THRESHOLD(a) (a / 4) /* In mg */
/*---------------------------------------------------------------------------*/
#define ACCEL_INTEL_EN    0x80
#define ACCEL_INTEL_MODE  0x40
/*---------------------------------------------------------------------------*/
#define ACCEL_FCHOICE     0x08
/*#define A_DLPFCFG_218_1  0 */ /* Same parameters as below */
#define A_DLPFCFG_218_1   1
#define A_DLPFCFG_99      2
#define A_DLPFCFG_44_8    3
#define A_DLPFCFG_21_2    4
#define A_DLPFCFG_10_2    5
#define A_DLPFCFG_5_05    6
#define A_DLPFCFG_420     7
/*---------------------------------------------------------------------------*/
#define MPU_DATA_READY    0x01
#define MPU_MOVEMENT      0x40
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, SENSOR_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Delay */
#define delay_ms(i) (ti_lib_cpu_delay(8000 * (i)))
/*---------------------------------------------------------------------------*/
static uint8_t val;
static uint8_t interrupt_status;
/*---------------------------------------------------------------------------*/
#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_BOOTING      1
#define SENSOR_STATE_ENABLED      2

static int state = SENSOR_STATE_DISABLED;
/*---------------------------------------------------------------------------*/
/*
 * Wait SENSOR_BOOT_DELAY ticks for the sensor to boot and
 * SENSOR_STARTUP_DELAY for readings to be ready
 * Gyro is a little slower than Acc
 */
#define SENSOR_BOOT_DELAY     8
#define SENSOR_STARTUP_DELAY  5

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 10
/*---------------------------------------------------------------------------*/
/**
 * \brief Place the MPU in low power mode
 */
static void
sensor_sleep(void)
{
  SENSOR_SELECT();
  val = MPU_SLEEP;
  sensor_common_write_reg(PWR_MGMT_1, &val, 1);
  SENSOR_DESELECT();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Check whether a data or wake on motion interrupt has occurred
 * \return Return the interrupt status
 *
 * This driver does not use interrupts, however this function allows us to
 * determine whether a new sensor reading is available
 */
static uint8_t
get_interrupt_status(void)
{
  SENSOR_SELECT();
  sensor_common_read_reg(INT_STATUS, &interrupt_status, 1);
  SENSOR_DESELECT();

  return interrupt_status;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Enable wake-on-Motion interrupt in MPU-9250
 */
static void
enable_mpu9250_wom_irq(void)
{
    /* Configuration Wake-on-Motion Interrupt using low power Accel mode */
    SENSOR_SELECT();

    /* Make Sure Accel is running */
    val = 0;
    sensor_common_write_reg(PWR_MGMT_1, &val, 1);
    val = GYRO_AXES; /* Disable gyro axes */
    sensor_common_write_reg(PWR_MGMT_2, &val, 1);

    /* Set Accel LPF setting to 218.1 Hz Bandwidth: */
    val = ACCEL_FCHOICE | A_DLPFCFG_218_1;
    sensor_common_write_reg(ACCEL_CONFIG_2, &val, 1);

    /* Enable Motion interrupt */
    val = BIT_WOM_EN;
    sensor_common_write_reg(INT_ENABLE, &val, 1);

    /* Configure interrupt pin */
    val = BIT_ACTL | BIT_LATCH_EN;
    sensor_common_write_reg(INT_PIN_CFG, &val, 1);

    /* Enable Accel Hardware Intelligence */
    val = ACCEL_INTEL_EN | ACCEL_INTEL_MODE;
    sensor_common_write_reg(ACCEL_INTEL_CTRL, &val, 1);

    /* Set Motion Threshold */
    val = WOM_THRESHOLD(20); /* (0~1022mg) */
    sensor_common_write_reg(WOM_THR, &val, 1);

    /* Set Frequency of Wake-up */
    val = LPOSC_CLK_0_98; /* 0.98 Hz */
    sensor_common_write_reg(LP_ACCEL_ODR, &val, 1);

    /* Enable Cycle Mode (Accel Low Power Mode) */
    val = BIT_LPA_CYCLE;
    sensor_common_write_reg(PWR_MGMT_1, &val, 1);

    /* Done setting up sensor for wake-on-motion interrupt */
    SENSOR_DESELECT();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Handler for Sensortag-CC26XX MPU-9250 interrupts
 */
static void
mpu_interrupt_handler(gpio_hal_pin_mask_t pin_mask)
{
  if (pin_mask & MPU_INT_PIN ||
      /* By reading the interrupt register we also clears the interrupt. */
      get_interrupt_status() & WOM_INT_MASK) {
    sensors_changed(&mpu_9250_wakeup_sensor);
  }
}
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&mpu_9250_wakeup_sensor);
}
/*---------------------------------------------------------------------------*/
static void
initialise(void *not_used)
{
  enable_mpu9250_wom_irq();

  ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
}
/*---------------------------------------------------------------------------*/
static void
power_up(void)
{
  ti_lib_gpio_set_dio(BOARD_IOID_MPU_POWER);
  state = SENSOR_STATE_BOOTING;

  ctimer_set(&startup_timer, SENSOR_BOOT_DELAY, initialise, NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Return value of the MPU-9250 interrupt pin
 * \param type Not used
 * \return Status of the interrupt pin from the sensor
 */
static int
value(int type)
{
  /*
    PLEASE NOTE, reading the interrupt status register will cause the interrupt
    flag - and thus the pin - to be cleared. It will seems like the interrupt
    never occured.

    uint8_t status = int_status();
    PRINTF("Int Status: 0x%2.2x\n", status);
    return status & WOM_INT_MASK;
  */
  return (int)ti_lib_gpio_read_dio(MPU_INT_PIN);
}
/*---------------------------------------------------------------------------*/
/* Event handler definitions for MPU Wake-On-Motion. */
static gpio_hal_event_handler_t mpu9250_wom_event_handler;
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the MPU9250 sensor Wake Up.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * \param type SENSORS_HW_INIT: Initialise. SENSORS_ACTIVE: Enables/Disables
 *        depending on 'value'
 * \param value 0: disable, non-zero: enable
 * \return Always returns 1
 */
static int
configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    ti_lib_ioc_int_disable(MPU_INT_PIN);
    ti_lib_gpio_clear_event_dio(MPU_INT_PIN);

    /* Enable the GPIO clock when the CM3 is running */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

    /* S/W control, input, pull-down */
    ti_lib_ioc_port_configure_set(MPU_INT_PIN, IOC_PORT_GPIO,
                                  MPU_INT_IO_CFG);

    ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
    ti_lib_ioc_io_drv_strength_set(BOARD_IOID_MPU_POWER, IOC_CURRENT_4MA,
                                   IOC_STRENGTH_MAX);
    ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);

    /* Register interrupt vector */
    mpu9250_wom_event_handler.pin_mask = gpio_hal_pin_to_mask(MPU_INT_PIN);
    mpu9250_wom_event_handler.handler = mpu_interrupt_handler;
    gpio_hal_register_handler(&mpu9250_wom_event_handler);

    break;
  case SENSORS_ACTIVE:
    if(enable) {
      PRINTF("MPU WOM Enabled\n");
      power_up();
      ti_lib_ioc_int_enable(MPU_INT_PIN);
    } else {
      if(HWREG(GPIO_BASE + GPIO_O_DOUT31_0) & BOARD_MPU_POWER) {
        PRINTF("MPU WOM Disabled\n");
        /* Then check our state */
        ctimer_stop(&startup_timer);
        sensor_sleep();
        while(ti_lib_i2c_master_busy(I2C0_BASE));
        ti_lib_ioc_int_disable(MPU_INT_PIN);
        state = SENSOR_STATE_DISABLED;
        ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
      }
    }
    break;
  default:
    break;
  }
  return state;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor interrupt
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 Interrupt enabled, 0: Disabled
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
#ifdef USE_HW_INT_PIN
    return (ti_lib_ioc_port_configure_get(MPU_INT_PIN)
            & IOC_INT_ENABLE) == IOC_INT_ENABLE;
#else
    return get_interrupt_status() & WOM_INT_MASK;
#endif
    break;
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(mpu_9250_wakeup_sensor, "MPU9250WakeUp", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
