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
 * \addtogroup wittratag-cc26xx-peripherals
 * @{
 *
 * \defgroup wittratag-cc26xx-mpu Wittratag 2.0 Motion Processing Unit
 *
 * Driver for the Invensense MPU9250 Motion Processing Unit.
 *
 * Due to the time required between triggering a reading and the reading
 * becoming available, this driver is meant to be used in an asynchronous
 * fashion. The caller must first activate the sensor by calling
 * mpu_9250_sensor.configure(SENSORS_ACTIVE, xyz);
 * The value for the xyz arguments depends on the required readings. If the
 * caller intends to read both the accelerometer as well as the gyro then
 * xyz should be MPU_9250_SENSOR_TYPE_ALL. If the caller only needs to take a
 * reading from one of the two elements, xyz should be one of
 * MPU_9250_SENSOR_TYPE_ACC or MPU_9250_SENSOR_TYPE_GYRO
 *
 * Calling .configure() will power up the sensor and initialise it. When the
 * sensor is ready to provide readings, the driver will generate a
 * sensors_changed event.
 *
 * Calls to .status() will return the driver's state which could indicate that
 * the sensor is off, booting or on.
 *
 * Once a reading has been taken, the caller has two options:
 * - Turn the sensor off by calling SENSORS_DEACTIVATE, but in order to take
 *   subsequent readings the sensor must be started up all over
 * - Leave the sensor on. In this scenario, the caller can simply keep calling
 *   value() for subsequent readings, but having the sensor on will consume
 *   more energy, especially if both accelerometer and the gyro are on.
 * @{
 *
 * \file
 * Header file for the Wittratag Invensense MPU9250 motion processing unit
 */
/*---------------------------------------------------------------------------*/
#ifndef MPU_9250_WAKEUP_SENSOR_H_
#define MPU_9250_WAKEUP_SENSOR_H_
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor mpu_9250_wakeup_sensor;
/*---------------------------------------------------------------------------*/
#endif /* MPU_9250_WAKEUP_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
