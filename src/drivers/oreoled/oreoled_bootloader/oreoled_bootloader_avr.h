/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file oreoled_bootloader_avr.h
 * @author Angus Peaty <angusp@gmail.com>
 */

#ifndef OREOLED_BOOTLOADER_AVR_H
#define OREOLED_BOOTLOADER_AVR_H

#include "oreoled_bootloader.h"

class OREOLED_BOOTLOADER_AVR : public OREOLED_BOOTLOADER, public device::I2C
{
public:
	OREOLED_BOOTLOADER_AVR(int bus, int i2c_addr);
	~OREOLED_BOOTLOADER_AVR();

	int			start(void);
	int			update(bool force);
	int			ioctl(const unsigned cmd, const unsigned long arg);

private:
	void		print_info(void);
	void		startup_discovery(void);
	void		discover(void);
	void		run_updates(bool force);

	void		update_application(const bool force_update);
	int			app_reset(const int led_num);
	int			app_reset_all(void);
	int			app_ping(const int led_num);
	uint16_t	inapp_checksum(const int led_num);
	int			ping(const int led_num);
	uint8_t		version(const int led_num);
	int			version_is_supported(const uint8_t ver);
	uint16_t	app_version(const int led_num);
	uint16_t	app_checksum(const int led_num);
	int			set_colour(const int led_num, const uint8_t red, const uint8_t green);
	int			flash(const int led_num);
	int			flash_all(const bool force_update);
	int			boot(const int led_num);
	int			boot_all(void);
	uint16_t	firmware_checksum(void);
	int			coerce_healthy(void);
	void		cmd_add_checksum(oreoled_cmd_t* cmd);
	int			response_is_valid(const oreoled_cmd_t* cmd, const uint8_t* response, const uint8_t response_len);
	void		print_response(const uint8_t* response, const uint8_t response_length);

	/* internal variables */
	bool			_healthy[OREOLED_NUM_LEDS];		///< health of each LED
	bool			_in_boot[OREOLED_NUM_LEDS];		///< true for each LED that is in bootloader mode
	uint8_t			_num_healthy;					///< number of healthy LEDs
	uint8_t			_num_inboot;					///< number of LEDs in bootloader
	uint64_t		_start_time;					///< system time we first attempt to communicate with battery
	uint16_t		_fw_checksum;					///< the current 16bit XOR checksum of the built in oreoled firmware binary

	/* performance checking */
	perf_counter_t      _probe_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _reply_errors;
};

#endif /* OREOLED_BOOTLOADER_AVR_H */
