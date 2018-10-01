/*
 * Copyright (c) 2018, Nidatech AB - http://www.wittra.com/
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
 * \addtogroup wittratag-cc26xx-peripherals
 * @{
 *
 * \defgroup wittratag-cc26xx-bmg-sensor WittraTag 2.0 TI BMG250 Sensor


 * @{
 *
 * \file
 * Header file for the Wittratag Bosch BMG250 sensor
 */
/*---------------------------------------------------------------------------*/
#ifndef BMG_250_SENSOR_H
#define BMG_250_SENSOR_H
/*---------------------------------------------------------------------------*/
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
/* ACC / Gyro Axes */
#define BMG_250_SENSOR_TYPE_GYRO_Z   0x01
#define BMG_250_SENSOR_TYPE_GYRO_Y   0x02
#define BMG_250_SENSOR_TYPE_GYRO_X   0x04
#define BMG_250_SENSOR_TYPE_GYRO_ALL 0x07
/*---------------------------------------------------------------------------*/
/* Gyro range */
#define BMG_250_SENSOR_GYRO_RANGE_2000 0
#define BMG_250_SENSOR_GYRO_RANGE_1000 1
#define BMG_250_SENSOR_GYRO_RANGE_500  2
#define BMG_250_SENSOR_GYRO_RANGE_250  3
#define BMG_250_SENSOR_GYRO_RANGE_125  4
/*---------------------------------------------------------------------------*/
/* Gyro range configuration */
#ifdef BMG_250_SENSOR_CONF_GYRO_RANGE
#define BMG_250_SENSOR_GYRO_RANGE BMG_250_SENSOR_CONF_GYRO_RANGE
#else
#define BMG_250_SENSOR_GYRO_RANGE BMG_250_SENSOR_GYRO_RANGE_125
#endif
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor bmg_250_sensor;
/*---------------------------------------------------------------------------*/
#endif /* BMG_250_SENSOR_H */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
