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
 * @file IIM20670.hpp
 *
 * Driver for the Invensense IIM20670 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_IIM20670_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace InvenSense_IIM20670;

class IIM20670 : public device::SPI, public I2CSPIDriver<IIM20670>
{
public:
	IIM20670(const I2CSPIDriverConfig &config);
	~IIM20670() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};

	struct register_bank7_config_t {
		Register::BANK_7 reg;
		uint16_t set_bits{0};
		uint16_t clear_bits{0};
	};


	int probe() override;

	bool Reset();

	int Configure();

	void SelectRegisterBank(enum BANK_SELECT_BIT bank, bool force = false);
	void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(BANK_SELECT_BIT::BANK_0); }
	void SelectRegisterBank(Register::BANK_1 reg) { SelectRegisterBank(BANK_SELECT_BIT::BANK_1); }
	void SelectRegisterBank(Register::BANK_7 reg) { SelectRegisterBank(BANK_SELECT_BIT::BANK_7); }

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	uint8_t crc_algorithm(uint32_t data);

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> uint16_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint16_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint16_t setbits, uint16_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint16_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint16_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": CRC8 bad"))};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};

	int _failure_count{0};

	bool _unlock_status_enabled{false};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum BANK_SELECT_BIT _last_register_bank {BANK_SELECT_BIT::BANK_0};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{1};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_0::MODE_REGISTERS, 0, MODE_REGISTERS_BIT::CAPTURE_MODE},
	};

	uint8_t _checked_register_bank7{0};
	static constexpr uint8_t size_register_bank7_cfg{1};
	register_bank7_config_t _register_bank7_cfg[size_register_bank7_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_7::GYRO_SENSITIVITY_CONFIG, GYRO_SENSITIVITY_CONFIG_BIT::gyro_fs_sel},
	};
};
