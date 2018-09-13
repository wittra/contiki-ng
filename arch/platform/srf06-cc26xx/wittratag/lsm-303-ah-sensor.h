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
 * \defgroup wittratag-cc26xx-lsm303ah-sensor WittraTag 2.0 LSM303AH Sensor


 * @{
 *
 * \file
 * Header file for the Wittratag LSM303AH sensor, both accelerometer and magnetometer.
 */
/*---------------------------------------------------------------------------*/
#ifndef LSM_303_AH_SENSOR_H
#define LSM_303_AH_SENSOR_H
/*---------------------------------------------------------------------------*/
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
/* Accelerometer Axes */
#define LSM_303_AH_SENSOR_TYPE_ACC_Z   0x01
#define LSM_303_AH_SENSOR_TYPE_ACC_Y   0x02
#define LSM_303_AH_SENSOR_TYPE_ACC_X   0x04
#define LSM_303_AH_SENSOR_TYPE_ACC_ALL 0x07
/*---------------------------------------------------------------------------*/
/* Magnetometer Axes */
#define LSM_303_AH_SENSOR_TYPE_MAG_Z   0x01
#define LSM_303_AH_SENSOR_TYPE_MAG_Y   0x02
#define LSM_303_AH_SENSOR_TYPE_MAG_X   0x04
#define LSM_303_AH_SENSOR_TYPE_MAG_ALL 0x07
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor lsm_303_ah_acc_sensor;
extern const struct sensors_sensor lsm_303_ah_mag_sensor;
/*---------------------------------------------------------------------------*/
#endif /* LSM_303_AH_SENSOR_H */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
