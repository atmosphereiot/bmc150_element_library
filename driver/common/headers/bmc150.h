/*
 * Copyright (c) 2015, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BMC150_H__
#define __BMC150_H__

#include <stdint.h>
#include "../i2c/i2c.h"

typedef enum {
	BMC150_OK = (uint8_t) 0,
	BMC150_ERROR = !BMC150_OK
} BMC150_error_t;


/**
 * Pin value to set I2C address
 */
typedef enum {
	BMC150_SDO_LOW,
	BMC150_SDO_HIGH,
} BMC150_i2c_addr_select_t;

/**
 * Range of the accelerometer measures.
 */
typedef enum {
	BMC150_MODE_2G = 0x3,
	BMC150_MODE_4G = 0x5,
	BMC150_MODE_8G = 0x8,
	BMC150_MODE_16G = 0xC,
} BMC150_accel_mode_t;

/**
 * Size of the data filter.
 */
typedef enum {
	BMC150_BANDWIDTH_64MS = 0x8,
	BMC150_BANDWIDTH_32MS = 0x9,
	BMC150_BANDWIDTH_16MS = 0xA,
	BMC150_BANDWIDTH_8MS = 0xB,
	BMC150_BANDWIDTH_4MS = 0xC,
	BMC150_BANDWIDTH_2MS = 0xD,
	BMC150_BANDWIDTH_1MS = 0xE,
	BMC150_BANDWIDTH_500US = 0xF,
} BMC150_bandwidth_t;

typedef enum {
	BMC150_MAG_POWER_SUSPEND = 0x0,
	BMC150_MAG_POWER_ACTIVE = 0x1,
} BMC150_mag_power_t;

typedef enum {
	BMC150_MAG_PRESET_REGULAR,
	BMC150_MAG_PRESET_LOW_POWER,
	BMC150_MAG_PRESET_ENHANCED,
	BMC150_MAG_PRESET_HIGH_ACCURACY,
} BMC150_mag_preset_t;

typedef struct {
	int16_t x, y, z;
} BMC150_accel_t;

typedef struct {
	int x, y, z;
} BMC150_mag_t;

BMC150_error_t BMC150_init(BMC150_i2c_addr_select_t pos);

BMC150_error_t BMC150_read_accel(BMC150_accel_t *const accel);

BMC150_error_t BMC150_set_accel_mode(BMC150_accel_mode_t mode);

BMC150_error_t BMC150_set_bandwidth(BMC150_bandwidth_t bw);

BMC150_error_t BMC150_read_mag(BMC150_mag_t *const mag);

BMC150_error_t BMC150_mag_set_power(BMC150_mag_power_t power);

BMC150_error_t BMC150_mag_set_preset(BMC150_mag_preset_t preset);

#endif /* __BMC150_H__ */
