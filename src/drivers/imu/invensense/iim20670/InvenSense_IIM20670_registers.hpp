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
 * @file InvenSense_IIM20670_registers.hpp
 *
 * Invensense IIM-20670 registers.
 *
 */

#pragma once

#include <cstdint>
#include <cstddef>

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

namespace InvenSense_IIM20670
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI
static constexpr uint8_t DIR_WRITE = 0x80;

static constexpr uint16_t WHOAMI = 0xF3;
static constexpr uint16_t FIXED_VALUE = 0xAA55;

static constexpr uint32_t  INTERNAL_SAMPLING_US = 1250; // 8000 Samples/s

namespace Register
{

enum class BANK_0 : uint8_t {

	GYRO_X_DATA = 0x00,
	GYRO_Y_DATA = 0x01,
	GYRO_Z_DATA = 0x02,
	TEMP_DATA1 = 0x03,
	ACCEL_X_DATA = 0x04,
	ACCEL_Y_DATA = 0x05,
	ACCEL_Z_DATA = 0x06,
	TEMP_DATA2 = 0x07,

	RESET_CONTROL = 0x18,

	MODE_REGISTERS = 0x19,

	GYRO_ACCEL_ID = 0x1B,
	HW_REV = 0x1C,
	ID_CODE_1 = 0x1D,
	ID_CODE_2 = 0x1E,

	BANK_SELECT = 0x1F,

	FIXED_VALUE = 0x0B,
};

enum class BANK_1 : uint8_t {
	WHOAMI = 0x0E,
};

enum class BANK_7 : uint8_t {
	GYRO_SENSITIVITY_CONFIG = 0x14,
};

}

enum RESET_CONTROL_BIT : uint16_t {
	HARD_RESET = Bit2,
};

enum MODE_REGISTERS_BIT : uint16_t {
	CAPTURE_MODE = Bit3,	// Writing a 0 resumes the refresh of the output data registers.
};

enum GYRO_SENSITIVITY_CONFIG_BIT : uint16_t {
	gyro_fs_sel = Bit0 | Bit1,	// ±1966 dps
};

enum BANK_SELECT_BIT : uint16_t {
	BANK_0 = 0,
	BANK_1 = Bit0,
	BANK_3 = Bit1 | Bit0,
	BANK_7 = Bit2 | Bit1 | Bit0,
};
}
