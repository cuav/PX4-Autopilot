/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Analog_Devices_ADIS16500_registers.hpp
 *
 * Analog Devices ADIS16500 registers.
 *
 */

#pragma once

#include <cstdint>

static constexpr uint16_t Bit0  = (1 << 0);
static constexpr uint16_t Bit1  = (1 << 1);
static constexpr uint16_t Bit2  = (1 << 2);
static constexpr uint16_t Bit3  = (1 << 3);
static constexpr uint16_t Bit4  = (1 << 4);
static constexpr uint16_t Bit5  = (1 << 5);
static constexpr uint16_t Bit6  = (1 << 6);
static constexpr uint16_t Bit7  = (1 << 7);
static constexpr uint16_t Bit8  = (1 << 8);
static constexpr uint16_t Bit9  = (1 << 9);
static constexpr uint16_t Bit10 = (1 << 10);
static constexpr uint16_t Bit11 = (1 << 11);
static constexpr uint16_t Bit12 = (1 << 12);
static constexpr uint16_t Bit13 = (1 << 13);
static constexpr uint16_t Bit14 = (1 << 14);
static constexpr uint16_t Bit15 = (1 << 15);

namespace Analog_Devices_ADIS16500
{
static constexpr uint32_t SPI_SPEED = 2 * 1000 * 1000;		// 2 MHz SPI serial interface
static constexpr uint32_t SPI_SPEED_BURST = 1 * 1000 * 1000;	// 1 MHz SPI serial interface for burst read

static constexpr uint32_t SPI_STALL_PERIOD = 16;	// 16 us Stall period between data

static constexpr uint16_t DIR_WRITE = 0x80;

static constexpr uint16_t PRODUCT_IDENTIFICATION = 0x4074;

static constexpr uint16_t BURST_READ = 0x6800;

static constexpr uint32_t SAMPLE_INTERVAL_US = 500; // 2000 Hz


enum class Register : uint16_t {
	DIAG_STAT  = 0x02,

	MSC_CTRL   = 0x60,

	GLOB_CMD   = 0x68,

	FIRM_REV   = 0x6C,
	FIRM_DM    = 0x6E,
	FIRM_Y     = 0x70,
	PROD_ID    = 0x72,
	SERIAL_NUM = 0x74,
};

enum MSC_CTRL_BIT : uint16_t {
	DR_POLARITY = Bit0,	// 1 = active high when data is valid
};

enum GLOB_CMD_BIT : uint16_t {
	SOFTWARE_RESET 	 = Bit7,
	SENSOR_SELF_TEST = Bit2,
};

}
