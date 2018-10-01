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
/** \addtogroup cc26xx-srf-tag
 * @{
 *
 * \defgroup wittratag-cc13xx-peripherals CC1350 Wittratag Peripherals
 *
 * Defines related to the CC1350 Wittratag
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file can be used as the basis to configure other boards using the
 * CC13xx code as their basis.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the Wittratag
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED HAL configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define LEDS_CONF_COUNT                 1
#define LEDS_CONF_RED                   1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_16
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_DP4_UARTRX     IOID_28
#define BOARD_IOID_DP5_UARTTX     IOID_29

#define BOARD_IOID_UART_RX        BOARD_IOID_DP4_UARTRX
#define BOARD_IOID_UART_TX        BOARD_IOID_DP5_UARTTX

#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name External flash IOID mapping and part-related constants
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define EXT_FLASH_SPI_CONTROLLER    SPI_CONTROLLER_SPI0

#define BOARD_IOID_FLASH_SCK        IOID_17
#define BOARD_IOID_FLASH_MOSI       IOID_19
#define BOARD_IOID_FLASH_MISO       IOID_18
#define BOARD_IOID_FLASH_CS         IOID_21

#define EXT_FLASH_SPI_PIN_SCK       17
#define EXT_FLASH_SPI_PIN_MOSI      19
#define EXT_FLASH_SPI_PIN_MISO      18
#define EXT_FLASH_SPI_PIN_CS        21

#define EXT_FLASH_DEVICE_ID         0x14
#define EXT_FLASH_MID               0xC2

#define EXT_FLASH_PROGRAM_PAGE_SIZE 256
#define EXT_FLASH_ERASE_SECTOR_SIZE 4096
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SDA_HP         IOID_UNUSED /**< Interface 1 SDA: MPU */
#define BOARD_IOID_SCL_HP         IOID_UNUSED /**< Interface 1 SCL: MPU */
#define BOARD_IOID_SDA            IOID_5 /**< Interface 0 SDA: All sensors */
#define BOARD_IOID_SCL            IOID_6 /**< Interface 0 SCL: All sensors */
#define BOARD_IOID_I2C_BUS_EN     IOID_3 /**< Enable I2C bus to USB conn */
#define BOARD_EN_I2C_BUS          (1 << BOARD_IOID_I2C_BUS_EN)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Gyroscope IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_GYRO_POWER     IOID_8
#define BOARD_GYRO_POWER          (1 << BOARD_IOID_GYRO_POWER)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Accelerometer IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ACC_INT        IOID_2
#define BOARD_ACC_INT             (1 << BOARD_IOID_ACC_INT)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Hall Sensor IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_HALL_INT       IOID_1
#define BOARD_IOID_HALL_PWR       IOID_7
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief GPS IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_GPS_POWER      IOID_10
#define BOARD_IOID_GPS_WAKEUP     IOID_11
#define BOARD_IOID_GPS_CTS        IOID_12
#define BOARD_IOID_GPS_ON         IOID_13
#define BOARD_IOID_GPS_RTS        IOID_14
#define BOARD_GPS_POWER           (1 << BOARD_IOID_GPS_POWER)
#define BOARD_GPS_WAKEUP          (1 << BOARD_IOID_GPS_WAKEUP)
#define BOARD_GPS_ON              (1 << BOARD_IOID_GPS_ON)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Charger IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_CHARGER_EN     IOID_24
#define BOARD_IOID_CHARGER_INT    IOID_22
#define BOARD_CHARGER_EN          (1 << BOARD_IOID_CHARGER_EN)
#define BOARD_CHARGER_INT         (1 << BOARD_IOID_CHARGER_INT)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief 1V8 VBUS Control IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_1V8_VBUS_EN    IOID_23
#define BOARD_1V8_VBUS_EN         (1 << BOARD_IOID_1V8_VBUS_EN)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief ADC inputs and control IOID mappings
 *
 * For DIO to AUXIO mapping, see "Technical Reference Manual", SWCU117H,
 * page 988, table 11-2.
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ADC_VEXT       IOID_25
#define BOARD_IOID_ADC_ID         IOID_26
#define BOARD_IOID_ADC_VMAIN      IOID_27
#define BOARD_IOID_ADC_ON         IOID_30
#define BOARD_ADC_ON              (1 << BOARD_IOID_ADC_ON)
#define BOARD_AUXIO_ADC_VEXT      ADC_COMPB_IN_AUXIO5
#define BOARD_AUXIO_ADC_ID        ADC_COMPB_IN_AUXIO4
#define BOARD_AUXIO_ADC_VMAIN     ADC_COMPB_IN_AUXIO3
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Unused pins IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_DP0            IOID_4
#define BOARD_IOID_DP1            IOID_9
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name RF Front End configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define RF_CORE_CONF_RADIO_SETUP_FRONT_END_MODE 0x01 /* Single-Ended, RFP */
#define RF_CORE_CONF_RADIO_SETUP_BIAS_MODE      0x01 /* External */
#define RF_CORE_CONF_PROP_FRONT_END_MODE        0x02 /* Single-Ended, RFN */
#define RF_CORE_CONF_PROP_BIAS_MODE             0x01 /* External */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Board-specific overrides
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define IEEE_MODE_CONF_BOARD_OVERRIDES        ADI_HALFREG_OVERRIDE(0, 16, 0x7, 1),
#define RF_BLE_CONF_BOARD_OVERRIDES           ADI_HALFREG_OVERRIDE(0, 16, 0x7, 1),
#define SMARTRF_SETTINGS_CONF_BOARD_OVERRIDES ADI_HALFREG_OVERRIDE(0, 16, 0x7, 2),

#define SMARTRF_SETTINGS_CONF_RSSI_OFFSET_779_930  0x00F688A3
#define SMARTRF_SETTINGS_CONF_OVERRIDE_TRIM_OFFSET 0x00018883
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Board indices for the button HAL, external units considered buttons
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_BUTTON_HAL_INDEX_HALL_RELAY 0x00
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "Wittratag CC1350"

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Board specific iniatialisation
 * @{
 */
void board_init(void);
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
