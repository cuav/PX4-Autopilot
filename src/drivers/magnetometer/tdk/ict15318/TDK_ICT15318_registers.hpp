/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file TDK_ICT15318_registers.hpp
 *
 * TDK ICT15318 registers.
 *
 */

#pragma once

#include <cstdint>

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace TDK_ICT15318
{
static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x1E;

static constexpr uint8_t chip_id = 0x45;

enum class Register : uint8_t {
	CHIP_ID = 0x01,

	MODE_CTRL = 0x04,
	MODE_STATUS = 0x05,	// Read Only
	STATUS = 0x06,		// Data ready flag
	FRAME_CNT = 0x07,

	TEMP_DATA_LSB = 0x08,
	TEMP_DATA_MSB = 0x09,

	MAG_DATA_X_LSB = 0x0A,
	MAG_DATA_X_MSB = 0x0B,
	MAG_DATA_Y_LSB = 0x0C,
	MAG_DATA_Y_MSB = 0x0D,
	MAG_DATA_Z_LSB = 0x0E,
	MAG_DATA_Z_MSB = 0x0F,

	SEQUENCER_CTRL = 0x7C,
};

enum MODE_CTRL_BIT : uint8_t {
	ODR_100Hz = Bit4 | Bit5 | Bit6, //  000: ODR 100Hz
	SLEEP_MODE = 0, 	        // 0: sleep mode
	PULSED_MODE = Bit0,		// 1: pulsed mode
	SINGLE_SHOT_MODE = Bit1,	// 2: pulsed mode
	MRM_MODE = Bit0 | Bit1,		// 3: pulsed mode
};

enum SEQUENCER_CTRL_BIT : uint8_t {
	SOFTWARE_RESET = Bit7,
};

} // namespace TDK_ICT15318
