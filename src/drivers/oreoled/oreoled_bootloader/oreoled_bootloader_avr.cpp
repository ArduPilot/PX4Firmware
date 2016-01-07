/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file oreoled_bootloader_avr.cpp
 * @author Angus Peart <angusp@gmail.com>
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/stat.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_oreoled.h>
#include <drivers/drv_oreoled_bootloader.h>

#include "oreoled_bootloader_avr.h"

#define OREOLED_BL_TARGET_DEFAULT		OREOLED_BL_TARGET_AVR

#define OREOLED_BOOT_FLASH_WAITMS		10

#define OREOLED_BOOT_SUPPORTED_VER_MIN	0x01
#define OREOLED_BOOT_SUPPORTED_VER_MAX	0x02

#define OREOLED_BOOT_CMD_PING			0x40
#define OREOLED_BOOT_CMD_BL_VER			0x41
#define OREOLED_BOOT_CMD_APP_VER		0x42
#define OREOLED_BOOT_CMD_APP_CHECKSUM	0x43
#define OREOLED_BOOT_CMD_SET_COLOUR		0x44

#define OREOLED_BOOT_CMD_WRITE_FLASH_A	0x50
#define OREOLED_BOOT_CMD_WRITE_FLASH_B	0x51
#define OREOLED_BOOT_CMD_FINALISE_FLASH	0x55

#define OREOLED_BOOT_CMD_BOOT_APP		0x60

#define OREOLED_BL_I2C_RETRYCOUNT		2
#define OEROLED_BOOT_COMMAND_RETRIES	10

/* magic number used to verify the software reset is valid */
#define OEROLED_RESET_NONCE				0x2A

#define OREOLED_BOOT_CMD_PING_NONCE		0x2A
#define OREOLED_BOOT_CMD_BOOT_NONCE		0xA2

#define OREOLED_FW_FILE_HEADER_LENGTH	2
#define OREOLED_FW_FILE_SIZE_LIMIT		6144
#define OREOLED_FW_FILE					"/etc/firmware/oreoled.bin"


#define OREOLED_STARTUP_TIMEOUT_USEC	2000000U			///< timeout looking for oreoleds 2 seconds after startup
#define OREOLED_STARTUP_INTERVAL_US		(1000000U / 10U)	///< time in microseconds, run at 10hz
#define OREOLED_UPDATE_INTERVAL_US		(1000000U / 50U)	///< time in microseconds, run at 50hz

/* constructor */
OREOLED_BOOTLOADER_AVR::OREOLED_BOOTLOADER_AVR(int bus, int i2c_addr) :
	I2C("oreoledbl_avr", OREOLEDBL0_DEVICE_PATH, bus, i2c_addr, 100000),
	_num_healthy(0),
	_num_inboot(0),
	_fw_checksum(0x0000),
	_probe_perf(perf_alloc(PC_ELAPSED, "oreoled_bl_probe")),
	_comms_errors(perf_alloc(PC_COUNT, "oreoled_bl_comms_errors")),
	_reply_errors(perf_alloc(PC_COUNT, "oreoled_bl_reply_errors"))
{
	/* initialise to unhealthy */
	memset(_healthy, 0, sizeof(_healthy));

	/* initialise to in application */
	memset(_in_boot, 0, sizeof(_in_boot));

	/* capture startup time */
	_start_time = hrt_absolute_time();
}

/* destructor */
OREOLED_BOOTLOADER_AVR::~OREOLED_BOOTLOADER_AVR()
{
	/* free perf counters */
	perf_free(_probe_perf);
	perf_free(_comms_errors);
	perf_free(_reply_errors);
}

int
OREOLED_BOOTLOADER_AVR::start(void)
{
	int ret;

	/* initialise I2C bus */
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	startup_discovery();

	return OK;
}

int
OREOLED_BOOTLOADER_AVR::update(bool force)
{
	int ret;

	ret = start();
	if (ret != OK) {
		return ret;
	}

	run_updates(force);

	print_info();

	return OK;
}

int
OREOLED_BOOTLOADER_AVR::ioctl(const unsigned cmd, const unsigned long arg)
{
	int ret = EINVAL;

	switch (cmd) {
	case OREOLED_BL_RESET:
		ret = app_reset_all();
		return ret;

	case OREOLED_BL_PING:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				ping(i);
				ret = OK;
			}
		}
		return ret;

	case OREOLED_BL_VER:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				version(i);
				ret = OK;
			}
		}
		return ret;

	case OREOLED_BL_FLASH:
		ret = flash_all(true);
		return ret;

	case OREOLED_BL_APP_VER:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				app_version(i);
				ret = OK;
			}
		}
		return ret;

	case OREOLED_BL_APP_CHECKSUM:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				app_checksum(i);
				ret = OK;
			}
		}
		return ret;

	case OREOLED_BL_SET_COLOUR:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				set_colour(i, ((oreoled_rgbset_t *) arg)->red, ((oreoled_rgbset_t *) arg)->green);
				ret = OK;
			}
		}
		return ret;

	case OREOLED_BL_BOOT_APP:
		ret = boot_all();
		return ret;
	}

	return ret;
}

void
OREOLED_BOOTLOADER_AVR::print_info(void)
{
	/* print health info on each LED */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i]) {
			warnx("oreo %u: BAD", (unsigned)i);

		} else {
			warnx("oreo %u: OK", (unsigned)i);
		}
	}

	/* display perf info */
	perf_print_counter(_probe_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_reply_errors);
}

void
OREOLED_BOOTLOADER_AVR::startup_discovery(void)
{
	/* check time since startup */
	uint64_t now = hrt_absolute_time();
	bool startup_timeout = (now - _start_time > OREOLED_STARTUP_TIMEOUT_USEC);

	/* during startup period keep searching for unhealthy LEDs */
	do {
		discover();

		if (_num_healthy == OREOLED_NUM_LEDS) {
			break;
		}

		/* wait for 0.1 sec before running discovery again */
		usleep(OREOLED_STARTUP_INTERVAL_US);
	} while ((hrt_absolute_time() - _start_time) < OREOLED_STARTUP_TIMEOUT_USEC);
}

void
OREOLED_BOOTLOADER_AVR::discover(void)
{
	/* prepare command to turn off LED */
	/* add two bytes of pre-amble to for higher signal to noise ratio */
	uint8_t msg[] = {0xAA, 0x55, OREOLED_PATTERN_OFF, 0x00};

	/* attempt to contact each unhealthy LED */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i]) {
			perf_begin(_probe_perf);

			/* set I2C address */
			set_address(OREOLED_BASE_I2C_ADDR + i);

			/* Calculate XOR checksum and append to the i2c write data */
			msg[sizeof(msg) - 1] = OREOLED_BASE_I2C_ADDR + i;

			for (uint8_t j = 0; j < sizeof(msg) - 1; j++) {
				msg[sizeof(msg) - 1] ^= msg[j];
			}

			/* send I2C command */
			uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];
			if (transfer(msg, sizeof(msg), reply, 3) == OK) {
				if (reply[1] == OREOLED_BASE_I2C_ADDR + i &&
				    reply[2] == msg[sizeof(msg) - 1]) {
					warnx("oreoled %u ok - in bootloader", (unsigned)i);
					_healthy[i] = true;
					_num_healthy++;

					/* If slaves are in application record that so we can reset if we need to bootload */
					/* This additional check is required for LED firmwares below v1.3 and can be
					   deprecated once all LEDs in the wild have firmware >= v1.3 */
					if(ping(i) == OK) {
						_in_boot[i] = true;
						_num_inboot++;
					}

				/* Check for a reply with a checksum offset of 1,
				   which indicates a response from firmwares >= 1.3 */
				} else if (reply[1] == OREOLED_BASE_I2C_ADDR + i &&
				    reply[2] == msg[sizeof(msg) - 1] + 1) {
					warnx("oreoled %u ok - in application", (unsigned)i);
					_healthy[i] = true;
					_num_healthy++;

				} else {
					warnx("oreo reply errors: %u", (unsigned)_reply_errors);
					perf_count(_reply_errors);
				}

			} else {
				perf_count(_comms_errors);
			}

			perf_end(_probe_perf);
		}
	}
}

void
OREOLED_BOOTLOADER_AVR::run_updates(bool force)
{
	update_application(force);

	if (_num_inboot > 0) {
		boot_all();
		coerce_healthy();
		_num_inboot = 0;
	}
}

void
OREOLED_BOOTLOADER_AVR::update_application(const bool force_update)
{
	/* check booted oreoleds to see if the app can report it's checksum (release versions >= v1.2) */
	if(!force_update) {
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i] && !_in_boot[i]) {
				/* put any out of date oreoleds into bootloader mode */
				/* being in bootloader mode signals to be code below that the will likey need updating */
				if (inapp_checksum(i) != firmware_checksum()) {
					app_reset(i);
				}
			}
		}
	}

	/* reset all healthy oreoleds if the number of outdated oreoled's is > 0 */
	/* this is done for consistency, so if only one oreoled is updating, all LEDs show the same behaviour */
	/* otherwise a single oreoled could appear broken or failed. */
	if (_num_inboot > 0 || force_update) {
		app_reset_all();

		/* update each outdated and healthy LED in bootloader mode */
		flash_all(force_update);

		/* boot each healthy LED */
		boot_all();

		/* coerce LEDs with startup issues to be healthy again */
		coerce_healthy();
	}
}

int
OREOLED_BOOTLOADER_AVR::app_reset(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	/* send a reset */
	boot_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
	boot_cmd.buff[1] = OREOLED_PARAM_RESET;
	boot_cmd.buff[2] = OEROLED_RESET_NONCE;
	boot_cmd.buff[3] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 4;
	cmd_add_checksum(&boot_cmd);

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* send I2C command with a retry limit */
	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		if (transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 3) == OK) {
			if (reply[1] == (OREOLED_BASE_I2C_ADDR + boot_cmd.led_num) &&
			    reply[2] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
				/* slave returned a valid response */
				ret = OK;
				/* set this LED as being in boot mode now */
				_in_boot[led_num] = true;
				_num_inboot++;
				break;
			}
		}
	}

	/* Allow time for the LED to reboot */
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::app_reset_all(void)
{
	int ret = OK;

	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (_healthy[i] && !_in_boot[i]) {
			/* reset the LED if it's not in the bootloader */
			/* (this happens during a pixhawk OTA update, since the LEDs stay powered) */
			if (app_reset(i) != OK) {
				ret = -1;
			}
		}
	}

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::app_ping(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	/* send a pattern off command */
	boot_cmd.buff[0] = 0xAA;
	boot_cmd.buff[1] = 0x55;
	boot_cmd.buff[2] = OREOLED_PATTERN_OFF;
	boot_cmd.buff[3] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 4;
	cmd_add_checksum(&boot_cmd);

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* send I2C command with a retry limit */
	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		if (transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 3) == OK) {
			if (reply[1] == (OREOLED_BASE_I2C_ADDR + boot_cmd.led_num) &&
			    reply[2] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
				/* slave returned a valid response */
				ret = OK;
				break;
			}
		}
	}

	return ret;
}

uint16_t
OREOLED_BOOTLOADER_AVR::inapp_checksum(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint16_t ret = 0x0000;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
	boot_cmd.buff[1] = OREOLED_PARAM_APP_CHECKSUM;
	boot_cmd.buff[2] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 3;
	cmd_add_checksum(&boot_cmd);

	const uint8_t APP_CHECKSUM_REPLY_LENGTH = 6;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, APP_CHECKSUM_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, APP_CHECKSUM_REPLY_LENGTH) == OK) {
			warnx("bl app checksum OK from LED %i", boot_cmd.led_num);
			warnx("bl app checksum msb: 0x%x", reply[3]);
			warnx("bl app checksum lsb: 0x%x", reply[4]);
			ret = ((reply[3] << 8) | reply[4]);
			break;

		} else {
			warnx("bl app checksum FAIL from LED %i", boot_cmd.led_num);
			print_response(reply, APP_CHECKSUM_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl app checksum retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app checksum failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::ping(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_PING;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;
	cmd_add_checksum(&boot_cmd);

	const uint8_t BL_PING_REPLY_LENGTH = 5;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_PING_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_PING_REPLY_LENGTH) == OK &&
			reply[3] == OREOLED_BOOT_CMD_PING_NONCE) {
			warnx("bl ping OK from LED %i", boot_cmd.led_num);
			ret = OK;
			break;

		} else {
			warnx("bl ping FAIL from LED %i", boot_cmd.led_num);
			print_response(reply, BL_PING_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl ping retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl ping failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	return ret;
}

uint8_t
OREOLED_BOOTLOADER_AVR::version(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint8_t ret = 0x00;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_BL_VER;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;
	cmd_add_checksum(&boot_cmd);

	const uint8_t BL_VER_REPLY_LENGTH = 5;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_VER_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_VER_REPLY_LENGTH) == OK && version_is_supported(reply[3]) == OK) {
			warnx("bl ver from LED %i = %i", boot_cmd.led_num, reply[3]);
			ret = reply[3];
			break;
		} else {
			warnx("bl ver FAIL from LED %i", boot_cmd.led_num);
			print_response(reply, BL_VER_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl ver retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl ver failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::version_is_supported(const uint8_t ver)
{
	if (ver >= OREOLED_BOOT_SUPPORTED_VER_MIN && ver <= OREOLED_BOOT_SUPPORTED_VER_MAX) {
		return OK;
	}

	return EINVAL;
}

uint16_t
OREOLED_BOOTLOADER_AVR::app_version(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint16_t ret = 0x0000;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_APP_VER;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;
	cmd_add_checksum(&boot_cmd);

	const uint8_t BL_APP_VER_REPLY_LENGTH = 6;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_APP_VER_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_APP_VER_REPLY_LENGTH) == OK) {
			warnx("bl app version OK from LED %i", boot_cmd.led_num);
			warnx("bl app version msb: 0x%x", reply[3]);
			warnx("bl app version lsb: 0x%x", reply[4]);
			ret = ((reply[3] << 8) | reply[4]);
			break;

		} else {
			warnx("bl app version FAIL from LED %i", boot_cmd.led_num);
			print_response(reply, BL_APP_VER_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl app version retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app version failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	return ret;
}

uint16_t
OREOLED_BOOTLOADER_AVR::app_checksum(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint16_t ret = 0x0000;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_APP_CHECKSUM;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;
	cmd_add_checksum(&boot_cmd);

	const uint8_t BL_APP_CHECKSUM_REPLY_LENGTH = 6;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_APP_CHECKSUM_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_APP_CHECKSUM_REPLY_LENGTH) == OK) {
			warnx("bl app checksum OK from LED %i", boot_cmd.led_num);
			warnx("bl app checksum msb: 0x%x", reply[3]);
			warnx("bl app checksum lsb: 0x%x", reply[4]);
			ret = ((reply[3] << 8) | reply[4]);
			break;

		} else {
			warnx("bl app checksum FAIL from LED %i", boot_cmd.led_num);
			print_response(reply, BL_APP_CHECKSUM_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl app checksum retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app checksum failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::set_colour(const int led_num, const uint8_t red, const uint8_t green)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_SET_COLOUR;
	boot_cmd.buff[1] = red;
	boot_cmd.buff[2] = green;
	boot_cmd.buff[3] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 4;
	cmd_add_checksum(&boot_cmd);

	const uint8_t BL_COLOUR_REPLY_LENGTH = 4;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_COLOUR_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_COLOUR_REPLY_LENGTH) == OK) {
			warnx("bl set colour OK from LED %i", boot_cmd.led_num);
			ret = OK;
			break;

		} else {
			warnx("bl set colour FAIL from LED %i", boot_cmd.led_num);
			print_response(reply, BL_COLOUR_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl app colour retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app colour failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::flash(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	/* Open the bootloader file */
	int fd = ::open(OREOLED_FW_FILE, O_RDONLY);

	/* check for error opening the file */
	if (fd < 0) {
		return -1;
	}

	struct stat s;

	/* attempt to stat the file */
	if (stat(OREOLED_FW_FILE, &s) != 0) {
		::close(fd);
		return -1;
	}

	uint16_t fw_length = s.st_size - OREOLED_FW_FILE_HEADER_LENGTH;

	/* sanity-check file size */
	if (fw_length > OREOLED_FW_FILE_SIZE_LIMIT) {
		::close(fd);
		return -1;
	}

	uint8_t *buf = new uint8_t[s.st_size];

	/* check that the buffer has been allocated */
	if (buf == NULL) {
		::close(fd);
		return -1;
	}

	/* check that the firmware can be read into the buffer */
	if (::read(fd, buf, s.st_size) != s.st_size) {
		::close(fd);
		delete[] buf;
		return -1;
	}

	::close(fd);

	/* Grab the version bytes from the binary */
	uint8_t version_major = buf[0];
	uint8_t version_minor = buf[1];

	/* calculate flash pages (rounded up to nearest integer) */
	uint8_t flash_pages = ((fw_length + 64 - 1) / 64);

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	const uint8_t BL_FLASH_REPLY_LENGTH = 4;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* Loop through flash pages */
	for (uint8_t page_idx = 0; page_idx < flash_pages; page_idx++) {

		/* Send the first half of the 64 byte flash page */
		memset(boot_cmd.buff, 0, sizeof(boot_cmd.buff));
		boot_cmd.buff[0] = OREOLED_BOOT_CMD_WRITE_FLASH_A;
		boot_cmd.buff[1] = page_idx;
		memcpy(boot_cmd.buff + 2, buf + (page_idx * 64) + OREOLED_FW_FILE_HEADER_LENGTH, 32);
		boot_cmd.buff[32 + 2] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
		boot_cmd.num_bytes = 32 + 3;
		cmd_add_checksum(&boot_cmd);

		for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
			/* Send the I2C Write+Read */
			memset(reply, 0, sizeof(reply));
			transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_FLASH_REPLY_LENGTH);

			/* Check the response */
			if (response_is_valid(&boot_cmd, reply, BL_FLASH_REPLY_LENGTH) == OK) {
				warnx("bl flash %ia OK for LED %i", page_idx, boot_cmd.led_num);
				break;

			} else {
				warnx("bl flash %ia FAIL for LED %i", page_idx, boot_cmd.led_num);
				print_response(reply, BL_FLASH_REPLY_LENGTH);

				if (retry > 1) {
					warnx("bl flash %ia retrying LED %i", page_idx, boot_cmd.led_num);

				} else {
					warnx("bl flash %ia failed on LED %i", page_idx, boot_cmd.led_num);
					delete[] buf;
					return -1;
				}
			}
		}

		/* Send the second half of the 64 byte flash page */
		memset(boot_cmd.buff, 0, sizeof(boot_cmd.buff));
		boot_cmd.buff[0] = OREOLED_BOOT_CMD_WRITE_FLASH_B;
		memcpy(boot_cmd.buff + 1, buf + (page_idx * 64) + 32 + OREOLED_FW_FILE_HEADER_LENGTH, 32);
		boot_cmd.buff[32 + 1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
		boot_cmd.num_bytes = 32 + 2;
		cmd_add_checksum(&boot_cmd);

		for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
			/* Send the I2C Write+Read */
			memset(reply, 0, sizeof(reply));
			transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_FLASH_REPLY_LENGTH);

			/* Check the response */
			if (response_is_valid(&boot_cmd, reply, BL_FLASH_REPLY_LENGTH) == OK) {
				warnx("bl flash %ib OK for LED %i", page_idx, boot_cmd.led_num);
				break;

			} else {
				warnx("bl flash %ib FAIL for LED %i", page_idx, boot_cmd.led_num);
				print_response(reply, BL_FLASH_REPLY_LENGTH);

				if (retry > 1) {
					warnx("bl flash %ib retrying LED %i", page_idx, boot_cmd.led_num);

				} else {
					errx(1, "bl flash %ib failed on LED %i", page_idx, boot_cmd.led_num);
					delete[] buf;
					return -1;
				}
			}
		}

		/* Sleep to allow flash to write */
		/* Wait extra long on the first write, to allow time for EEPROM updates */
		if (page_idx == 0) {
			usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

		} else {
			usleep(OREOLED_BOOT_FLASH_WAITMS * 1000);
		}
	}

	uint16_t calculated_app_checksum = firmware_checksum();

	/* Flash writes must have succeeded so finalise the flash */
	boot_cmd.buff[0] = OREOLED_BOOT_CMD_FINALISE_FLASH;
	boot_cmd.buff[1] = version_major;
	boot_cmd.buff[2] = version_minor;
	boot_cmd.buff[3] = (uint8_t)(fw_length >> 8);
	boot_cmd.buff[4] = (uint8_t)(fw_length & 0xFF);
	boot_cmd.buff[5] = (uint8_t)(calculated_app_checksum >> 8);
	boot_cmd.buff[6] = (uint8_t)(calculated_app_checksum & 0xFF);
	boot_cmd.buff[7] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 8;
	cmd_add_checksum(&boot_cmd);

	/* Try to finalise for twice the amount of normal retries */
	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES * 2; retry > 0; retry--) {
		/* Send the I2C Write */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, BL_FLASH_REPLY_LENGTH);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_FLASH_REPLY_LENGTH) == OK) {
			warnx("bl finalise OK from LED %i", boot_cmd.led_num);
			break;

		} else {
			warnx("bl flash finalise FAIL for LED %i", boot_cmd.led_num);
			print_response(reply, BL_FLASH_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl finalise retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl finalise failed on LED %i", boot_cmd.led_num);
				delete[] buf;
				return -1;
			}
		}
	}

	/* allow time for flash to finalise */
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

	/* clean up file buffer */
	delete[] buf;

	return OK;
}

int
OREOLED_BOOTLOADER_AVR::boot(const int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_BOOT_APP;
	boot_cmd.buff[1] = OREOLED_BOOT_CMD_BOOT_NONCE;
	boot_cmd.buff[2] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 3;
	cmd_add_checksum(&boot_cmd);

	const uint8_t BL_BOOT_REPLY_LENGTH = 4;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write */
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 4);

		/* Check the response */
		if (response_is_valid(&boot_cmd, reply, BL_BOOT_REPLY_LENGTH) == OK) {
			warnx("bl boot OK from LED %i", boot_cmd.led_num);
			_in_boot[led_num] = false;
			_num_inboot--;
			ret = OK;
			break;

		/* 
		 * The bootloader will respond with OREOLED_BOOT_CMD_BOOT_NONCE in
		 * reply[2] if the boot operation failed
		 */
		} else if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
			   reply[2] == OREOLED_BOOT_CMD_BOOT_NONCE &&
			   reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl boot error from LED %i: no app", boot_cmd.led_num);
			break;

		} else {
			warnx("bl boot FAIL for LED %i", boot_cmd.led_num);
			print_response(reply, BL_BOOT_REPLY_LENGTH);

			if (retry > 1) {
				warnx("bl boot retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl boot failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	/* allow time for the LEDs to boot */
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::boot_all(void) {
	int ret = OK;

	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (_healthy[i] && _in_boot[i]) {
			/* boot the application */
			if (boot(i) != OK) {
				ret = -1;
			}
		}
	}

	return ret;
}

uint16_t
OREOLED_BOOTLOADER_AVR::firmware_checksum(void)
{
	/* Calculate the 16 bit XOR checksum of the firmware on the first call of this function */
	if (_fw_checksum == 0x0000) {
		/* Open the bootloader file */
		int fd = ::open(OREOLED_FW_FILE, O_RDONLY);

		/* check for error opening the file */
		if (fd < 0) {
			return -1;
		}

		struct stat s;

		/* attempt to stat the file */
		if (stat(OREOLED_FW_FILE, &s) != 0) {
			::close(fd);
			return -1;
		}

		uint16_t fw_length = s.st_size - OREOLED_FW_FILE_HEADER_LENGTH;

		/* sanity-check file size */
		if (fw_length > OREOLED_FW_FILE_SIZE_LIMIT) {
			::close(fd);
			return -1;
		}

		uint8_t *buf = new uint8_t[s.st_size];

		/* check that the buffer has been allocated */
		if (buf == NULL) {
			::close(fd);
			return -1;
		}

		/* check that the firmware can be read into the buffer */
		if (::read(fd, buf, s.st_size) != s.st_size) {
			::close(fd);
			delete[] buf;
			return -1;
		}

		::close(fd);

		/* Calculate a 16 bit XOR checksum of the flash */
		/* Skip the first two bytes which are the version information, plus
		   the next two bytes which are modified by the bootloader */
		uint16_t calculated_app_checksum = 0x0000;

		for (uint16_t j = 2 + OREOLED_FW_FILE_HEADER_LENGTH; j < s.st_size; j += 2) {
			calculated_app_checksum ^= (buf[j] << 8) | buf[j + 1];
		}

		delete[] buf;

		warnx("fw length = %i", fw_length);
		warnx("fw checksum = %i", calculated_app_checksum);

		/* Store the checksum so it's only calculated once */
		_fw_checksum = calculated_app_checksum;
	}

	return _fw_checksum;
}

int
OREOLED_BOOTLOADER_AVR::flash_all(const bool force_update)
{
	int ret = OK;

	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (_healthy[i] && _in_boot[i]) {
			int result = EINVAL;

			if (force_update) {
				result = flash(i);
			} else if (app_checksum(i) != firmware_checksum()) {
				/* only flash LEDs with an old version of the applictioon */
				result = flash(i);
			}

			if (result != OK) {
				ret = -1;
			}
		}
	}

	return ret;
}

int
OREOLED_BOOTLOADER_AVR::coerce_healthy(void)
{
	int ret = -1;

	/* check each unhealthy LED */
	/* this re-checks "unhealthy" LEDs as they can sometimes power up with the wrong ID, */
	/*  but will have the correct ID once they jump to the application and be healthy again */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i] && app_ping(i) == OK) {
			/* mark as healthy */
			_healthy[i] = true;
			_num_healthy++;
			ret = OK;
		}
	}

	return ret;
}

void
OREOLED_BOOTLOADER_AVR::cmd_add_checksum(oreoled_cmd_t* cmd)
{
	if (cmd->num_bytes == 0 || cmd->num_bytes >= OREOLED_CMD_LENGTH_MAX) {
		return;
	}

	/* write a basic 8-bit XOR checksum into the last byte of the command bytes array*/
	uint8_t checksum_idx = cmd->num_bytes - 1;
	for (uint8_t i = 0; i < checksum_idx; i++) {
		cmd->buff[checksum_idx] ^= cmd->buff[i];
	}
}

int
OREOLED_BOOTLOADER_AVR::response_is_valid(const oreoled_cmd_t* cmd, const uint8_t* response, const uint8_t response_len)
{
	if (response[1] == OREOLED_BASE_I2C_ADDR + cmd->led_num &&
		response[2] == cmd->buff[0] &&
		response[response_len - 1] == cmd->buff[cmd->num_bytes - 1]) {
		return OK;
	}

	return EINVAL;
}

void
OREOLED_BOOTLOADER_AVR::print_response(const uint8_t* response, const uint8_t response_length)
{
	const uint8_t checksum_idx = response_length - 1;
	warnx("bl ver response  ADDR: 0x%x", response[1]);
	warnx("bl ver response   CMD: 0x%x", response[2]);

	for (uint8_t i = 3; i < checksum_idx; i++) {
		warnx("bl ver response   [%i]: 0x%x", i, response[3]);
	}

	warnx("bl ver response   XOR: 0x%x", response[checksum_idx]);
}