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

#include "IAM20685.hpp"
#include "InvenSense_IAM20685_registers.hpp"

using namespace time_literals;

IAM20685::IAM20685(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}
}

IAM20685::~IAM20685()
{
	perf_free(_bad_register_perf);
	perf_free(_perf_crc_bad);
	perf_free(_drdy_missed_perf);
}

int IAM20685::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool IAM20685::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void IAM20685::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void IAM20685::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_drdy_missed_perf);
}

int IAM20685::probe()
{
	const uint16_t fixed_value = RegisterRead(Register::BANK_0::FIXED_VALUE);
	const uint16_t whoami = RegisterRead(Register::BANK_1::WHOAMI);

	if ((whoami != WHOAMI) && (fixed_value != FIXED_VALUE)) {
		PX4_ERR("whoami:0x%X, fixed_value:0x%X", whoami, fixed_value);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void IAM20685::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		RegisterWrite(Register::BANK_0::RESET_CONTROL, RESET_CONTROL_BIT::HARD_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_unlock_status_enabled = false;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(200_ms); // 200ms From power-up
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_1::WHOAMI) == WHOAMI) && (RegisterRead(Register::BANK_0::FIXED_VALUE) == FIXED_VALUE)
		    && (RegisterRead(Register::BANK_0::RESET_CONTROL)) == 0) {
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure() == PX4_OK) {
			_state = STATE::READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(INTERNAL_SAMPLING_US, INTERNAL_SAMPLING_US);
			}

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < INTERNAL_SAMPLING_US) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				ScheduleDelayed(INTERNAL_SAMPLING_US * 2);
			}

			int16_t gyro_x = RegisterRead(Register::BANK_0::GYRO_X_DATA);
			int16_t gyro_y = RegisterRead(Register::BANK_0::GYRO_Y_DATA);
			int16_t gyro_z = RegisterRead(Register::BANK_0::GYRO_Z_DATA);

			int16_t temperature1 = RegisterRead(Register::BANK_0::TEMP_DATA1);
			int16_t temperature2 = RegisterRead(Register::BANK_0::TEMP_DATA2);

			int16_t accel_x = RegisterRead(Register::BANK_0::ACCEL_X_DATA);
			int16_t accel_y = RegisterRead(Register::BANK_0::ACCEL_Y_DATA);
			int16_t accel_z = RegisterRead(Register::BANK_0::ACCEL_Z_DATA);

			// sensor's frame is +x forward, +y left, +z up
			//  flip y & z to publish right handed with z down (x forward, y right, z down)
			gyro_y = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
			gyro_z = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
			accel_y = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
			accel_z = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;


			temperature1 = 25 + temperature1 / 20;
			temperature2 = 25 + temperature2 / 20;
			int16_t temperature = (temperature1 + temperature2) / 2;

			_px4_accel.set_temperature(temperature);
			_px4_gyro.set_temperature(temperature);
			_px4_accel.update(timestamp_sample, accel_x, accel_y, accel_z);
			_px4_gyro.update(timestamp_sample, gyro_x, gyro_y, gyro_z);

			if (hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
				    && RegisterCheck(_register_bank7_cfg[_checked_register_bank7])
				   ) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
					_checked_register_bank7 = (_checked_register_bank7 + 1) % size_register_bank7_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}
			}
		}
	}
}

int IAM20685::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<IAM20685 *>(arg)->DataReady();
	return 0;
}

void IAM20685::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool IAM20685::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool IAM20685::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

int IAM20685::Configure()
{
	// first set and clear all configured register bits

	for (const auto &reg_cfg : _register_bank6_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank6_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			return PX4_ERROR;
		}
	}

	for (const auto &reg_cfg : _register_bank7_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank7_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			return PX4_ERROR;
		}
	}

	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			return PX4_ERROR;
		}
	}

	// IIM-20670 default full-scale settings ±16.384 g for accelerometer
	_px4_accel.set_range(16.384f * CONSTANTS_ONE_G);
	_px4_accel.set_scale(CONSTANTS_ONE_G / 2000.f);

	// gyro: ±1966 dps
	_px4_gyro.set_range(math::radians(1966.f));
	_px4_gyro.set_scale(math::radians(1.f / 16.67f)); // sensitivty: 16.67 LSB/dps

	return PX4_OK;
}


uint8_t IAM20685::crc_algorithm(uint32_t data)
{
	uint8_t crc = 0xFF;
	uint8_t crc_new = 0x00;

	for (int8_t i = 23; i >= 0; i--) {
		/* encoding algorithm
		   crc_new_bit7 = crc_bit6
		  crc_new_bit6 = crc_bit5
		  crc_new_bit5 = crc_bit4
		  crc_new_bit4 = crc_bit3 ^ crc_bit7
		  crc_new_bit3 = crc_bit2 ^ crc_bit7
		  crc_new_bit2 = crc_bit1 ^ crc_bit7
		  crc_new_bit1 = crc_bit0
		  crc_new_bit0 = input_data_bit[i] ^ crc_bit7 */
		crc_new = (crc & 0x71) << 1;
		crc_new |= (((crc >> 3) & 0x01) ^ (crc >> 7)) << 4;
		crc_new |= (((crc >> 2) & 0x01) ^ (crc >> 7)) << 3;
		crc_new |= (((crc >> 1) & 0x01) ^ (crc >> 7)) << 2;
		crc_new |= ((data >> i) & 0x01) ^ (crc >> 7);
		crc = crc_new;
	}

	crc = crc ^ 0xFF;

	return crc;
}

void IAM20685::SelectRegisterBank(enum BANK_SELECT_BIT bank, bool force)
{
	if (!_unlock_status_enabled) {
		//  Unlock the full-scale change operation
		uint8_t cmd[4] {};
		// 0xE4000288
		cmd[0] = 0xE4;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		cmd[3] = 0x88;
		transfer(&cmd[0], nullptr, 4);
		// 0xE400018B
		cmd[0] = 0xE4;
		cmd[1] = 0x00;
		cmd[2] = 0x01;
		cmd[3] = 0x8B;
		transfer(&cmd[0], nullptr, 4);
		// 0xE400048E
		cmd[0] = 0xE4;
		cmd[1] = 0x00;
		cmd[2] = 0x04;
		cmd[3] = 0x8E;
		transfer(&cmd[0], nullptr, 4);
		// 0xE40300AD
		cmd[0] = 0xE4;
		cmd[1] = 0x03;
		cmd[2] = 0x00;
		cmd[3] = 0xAD;
		transfer(&cmd[0], nullptr, 4);
		// 0xE4018017
		cmd[0] = 0xE4;
		cmd[1] = 0x01;
		cmd[2] = 0x80;
		cmd[3] = 0x17;
		transfer(&cmd[0], nullptr, 4);
		// 0xE4028030
		cmd[0] = 0xE4;
		cmd[1] = 0x02;
		cmd[2] = 0x80;
		cmd[3] = 0x30;
		transfer(&cmd[0], nullptr, 4);

		_unlock_status_enabled = true;
	}

	if (bank != _last_register_bank || force) {
		uint8_t cmd_bank_sel[4] {};
		uint32_t crc_data = 0;

		cmd_bank_sel[0] = (static_cast<uint8_t>(Register::BANK_0::BANK_SELECT) << 2) | DIR_WRITE;
		cmd_bank_sel[1] = ((bank & 0xFF00) >> 8);
		cmd_bank_sel[2] = (bank & 0x00FF);
		crc_data = (cmd_bank_sel[0] << 16) + (cmd_bank_sel[1] << 8) + cmd_bank_sel[2];
		cmd_bank_sel[3] = crc_algorithm(crc_data);

		transfer(&cmd_bank_sel[0], nullptr, 4);

		_last_register_bank = bank;
	}
}

template <typename T>
bool IAM20685::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

template <typename T>
uint16_t IAM20685::RegisterRead(T reg)
{
	uint8_t cmd[4] {};
	uint32_t crc_data = 0;

	cmd[0] = (static_cast<uint8_t>(reg) << 2);
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	crc_data = (cmd[0] << 16) + (cmd[1] << 8) + cmd[2];
	cmd[3] = crc_algorithm(crc_data);

	SelectRegisterBank(reg);

	transfer(&cmd[0], nullptr, 4);
	transfer(nullptr, &cmd[0], 4);

	crc_data = (cmd[0] << 16) + (cmd[1] << 8) + cmd[2];

	if (crc_algorithm(crc_data) != cmd[3]) {
		perf_count(_perf_crc_bad);
	}

	return ((cmd[1] << 8) + cmd[2]);
}
template <typename T>
void IAM20685::RegisterWrite(T reg, uint16_t value)
{
	uint8_t cmd[4] {};
	uint32_t crc_data = 0;

	cmd[0] = (static_cast<uint8_t>(reg) << 2) | DIR_WRITE;
	cmd[1] = ((value & 0xFF00) >> 8);
	cmd[2] = (value & 0x00FF);

	crc_data = (cmd[0] << 16) + (cmd[1] << 8) + cmd[2];
	cmd[3] = crc_algorithm(crc_data);

	SelectRegisterBank(reg);
	transfer(&cmd[0], nullptr, 4);
}

template <typename T>
void IAM20685::RegisterSetAndClearBits(T reg, uint16_t setbits, uint16_t clearbits)
{
	const uint16_t orig_val = RegisterRead(reg);

	uint16_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
