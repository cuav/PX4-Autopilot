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

#include "ICT15318.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICT15318::ICT15318(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

ICT15318::~ICT15318()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_drdy_missed_perf);
}

int ICT15318::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICT15318::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICT15318::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_drdy_missed_perf);
}

int ICT15318::probe()
{
	for (int retry = 0; retry < 3; retry++) {
		const uint8_t CHIP_ID = RegisterRead(Register::CHIP_ID);

		if (chip_id == CHIP_ID) {
			_retries = 1;
			return PX4_OK;

		} else {
			DEVICE_DEBUG("unexpected CHIP_ID 0x%02x", CHIP_ID);
		}
	}

	return PX4_ERROR;
}

void ICT15318::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		RegisterWrite(Register::SEQUENCER_CTRL, SEQUENCER_CTRL_BIT::SOFTWARE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		perf_count(_reset_perf);
		ScheduleDelayed(5_ms);
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::CHIP_ID) == chip_id)) {
			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(10_ms);

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
		if (Configure()) {
			// if configure succeeded then start measurement cycle
			_state = STATE::READ;
			ScheduleOnInterval(10_ms, 10_ms);

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
			struct TransferBuffer {
				// uint8_t FRAME_CNT;
				uint8_t TEMP_LSB;
				uint8_t TEMP_MSB;
				uint8_t DATAX_LSB;
				uint8_t DATAX_MSB;
				uint8_t DATAY_LSB;
				uint8_t DATAY_MSB;
				uint8_t DATAZ_LSB;
				uint8_t DATAZ_MSB;
			} buffer{};

			bool success = false;
			uint8_t cmd = static_cast<uint8_t>(Register::TEMP_DATA_LSB);

			uint8_t status = RegisterRead(Register::STATUS);

			if ((status & 0x01) != 0x01) {
				perf_count(_drdy_missed_perf);

			} else {
				if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {
					int16_t x = combine(buffer.DATAX_MSB, buffer.DATAX_LSB);
					int16_t y = combine(buffer.DATAY_MSB, buffer.DATAY_LSB);
					int16_t z = combine(buffer.DATAZ_MSB, buffer.DATAZ_LSB);
					int16_t temp = combine(buffer.TEMP_MSB, buffer.TEMP_LSB);

					// PX4_INFO("x:%x %x", buffer.DATAX_MSB, buffer.DATAX_LSB);
					// PX4_INFO("y:%x %x", buffer.DATAY_MSB, buffer.DATAY_LSB);
					// PX4_INFO("z:%x %x", buffer.DATAZ_MSB, buffer.DATAZ_LSB);
					// PX4_INFO("t:%x %x", buffer.TEMP_MSB, buffer.TEMP_LSB);

					// sensor's frame is +x forward, +y left, z up
					//  flip y to publish right handed with z down (x forward, y right, z down)
					y = (y == INT16_MIN) ? INT16_MAX : -y;
					z = (z == INT16_MIN) ? INT16_MAX : -z;

					const float temperature = temp * 6.25f * 0.001f + 25.0f;



					_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf));
					_px4_mag.set_temperature(temperature);
					_px4_mag.update(now, x, y, z);
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}

				} else {
					perf_count(_bad_transfer_perf);
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
					return;
				}
			}
		}

		break;
	}

}

bool ICT15318::Configure()
{
	// Enter MRM mode and magnetically reset TMR sensor components
	RegisterWrite(Register::MODE_CTRL, MODE_CTRL_BIT::MRM_MODE);
	ScheduleDelayed(1_ms);

	// Returning to Sleep mode, the pulse mode should always be activated from the sleep state
	RegisterWrite(Register::MODE_CTRL, MODE_CTRL_BIT::SLEEP_MODE);
	ScheduleDelayed(1_ms);

	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	_px4_mag.set_scale(0.75e-3f); // 75nT/LSB

	return success;
}

bool ICT15318::RegisterCheck(const register_config_t &reg_cfg)
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

int ICT15318::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	const int ret = transfer(&cmd, 1, &buffer, 1);

	if (ret != OK) { return -1; }

	return buffer;
}

void ICT15318::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void ICT15318::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
