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
 * @file oreoled_bootloader.cpp
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
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_oreoled.h>
#include <drivers/drv_oreoled_bootloader.h>

#include "oreoled_bootloader.h"
#include "oreoled_bootloader_avr.h"

/* for now, we only support one OREOLED */
namespace
{
	OREOLED_BOOTLOADER *g_oreoled_bl = nullptr;
}

static void oreoled_bl_init_usage();
static void oreoled_bl_cmd_usage();
static oreoled_bl_target_t oreoled_bl_parse_target(char* target);

extern "C" __EXPORT int oreoledbl_main(int argc, char *argv[]);

void
oreoled_bl_init_usage()
{
	warnx("missing command: try 'start', 'update [force]'");
	warnx("options:");
	warnx("    -t target (auto)");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", OREOLED_BASE_I2C_ADDR);
}

void
oreoled_bl_cmd_usage()
{
	warnx("missing command: try 'stop', 'ping', 'ver', 'appver', 'checksum', 'colour <red> <green>', 'flash', 'boot'");
}

oreoled_bl_target_t
oreoled_bl_parse_target(char *target)
{
	if (!strcmp(target, "avr")) {
		return OREOLED_BL_TARGET_AVR;
	} else if (!strcmp(target, "auto")) {
		return OREOLED_BL_TARGET_DEFAULT;
	}

	return OREOLED_BL_TARGET_INVALID;
}

int
oreoledbl_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int i2c_addr = OREOLED_BASE_I2C_ADDR; /* 7bit */
	oreoled_bl_target_t target = OREOLED_BL_TARGET_INVALID;

	int ch;

	/* jump over ping/ver/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:t:")) != EOF) {
		switch (ch) {
		case 'a':
			i2c_addr = (int)strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = (int)strtol(optarg, NULL, 0);
			break;

		case 't':
			target = oreoled_bl_parse_target(optarg);
			if (target != OREOLED_BL_TARGET_INVALID) {
				break;
			}
			warnx("invalid target '%s' specified", optarg);

		default:
			oreoled_bl_init_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		oreoled_bl_init_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	int ret;

	/* start driver */
	if (!strcmp(verb, "start") || !strcmp(verb, "update")) {
		if (g_oreoled_bl != nullptr) {
			errx(1, "already started");
		}

		/* by default use LED bus */
		if (i2cdevice == -1) {
			i2cdevice = PX4_I2C_BUS_LED;
		}

		/* instantiate driver */
		switch (target) {
			case OREOLED_BL_TARGET_AVR:
				g_oreoled_bl = new OREOLED_BOOTLOADER_AVR(i2cdevice, i2c_addr);
				break;

			default:
				errx(1, "unable to instantiate target driver");
		}

		/* check if object was created */
		if (g_oreoled_bl == nullptr) {
			errx(1, "failed to allocated memory for driver");
		}

		if (!strcmp(verb, "start")) {
			/* check object was created successfully */
			if (g_oreoled_bl->start() != OK) {
				delete g_oreoled_bl;
				g_oreoled_bl = nullptr;
				errx(1, "failed to start driver");
			}
		} else {
			/* handle update flag */
			bool force_update = false;
			if (argc > optind && !strcmp(argv[optind + 1], "force")) {
				warnx("forcing update");
				force_update = true;
			}

			/* check object was created successfully */
			if (g_oreoled_bl->update(force_update) != OK) {
				errx(1, "failed to start driver");
			}

			/* clean up the driver */
			delete g_oreoled_bl;
			g_oreoled_bl = nullptr;
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_oreoled_bl == nullptr) {
		warnx("not started");
		oreoled_bl_init_usage();
		exit(1);
	}

	if (!strcmp(verb, "stop")) {
		/* delete the oreoled object if stop was requested, in addition to turning off the LED. */
		OREOLED_BOOTLOADER *tmp_oreoled = g_oreoled_bl;
		g_oreoled_bl = nullptr;
		delete tmp_oreoled;
		exit(0);
	}

	/* send reset request to all LEDS */
	if (!strcmp(verb, "reset")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl reset");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_RESET, 0)) != OK) {
			errx(1, "failed to run reset");
		}

		close(fd);
		exit(ret);
	}

	/* attempt to flash all LEDS in bootloader mode*/
	if (!strcmp(verb, "flash")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl flash");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_FLASH, 0)) != OK) {
			errx(1, "failed to run flash");
		}

		close(fd);
		exit(ret);
	}

	/* send bootloader boot request to all LEDS */
	if (!strcmp(verb, "boot")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl boot");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_BOOT_APP, 0)) != OK) {
			errx(1, "failed to run boot");
		}

		close(fd);
		exit(ret);
	}

	/* send bootloader ping all LEDs */
	if (!strcmp(verb, "ping")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl ping");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_PING, 0)) != OK) {
			errx(1, "failed to run blping");
		}

		close(fd);
		exit(ret);
	}

	/* ask all LEDs for their bootloader version */
	if (!strcmp(verb, "ver")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl ver");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_VER, 0)) != OK) {
			errx(1, "failed to get bootloader version");
		}

		close(fd);
		exit(ret);
	}

	/* ask all LEDs for their application version */
	if (!strcmp(verb, "appver")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl appver");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_APP_VER, 0)) != OK) {
			errx(1, "failed to get app version");
		}

		close(fd);
		exit(ret);
	}

	/* ask all LEDs for their application checksum */
	if (!strcmp(verb, "checksum")) {
		if (argc < 2) {
			errx(1, "Usage: oreoledbl checksum");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_APP_CHECKSUM, 0)) != OK) {
			errx(1, "failed to get app checksum");
		}

		close(fd);
		exit(ret);
	}

	/* set the default bootloader LED colour on all LEDs */
	if (!strcmp(verb, "colour")) {
		if (argc < 4) {
			errx(1, "Usage: oreoledbl colour <red> <green>");
		}

		int fd = open(OREOLEDBL0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLEDBL0_DEVICE_PATH);
		}

		uint8_t red = (uint8_t)strtol(argv[2], NULL, 0);
		uint8_t green = (uint8_t)strtol(argv[3], NULL, 0);
		oreoled_rgbset_t rgb_set = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, red, green, 0};

		if ((ret = g_oreoled_bl->ioctl(OREOLED_BL_SET_COLOUR, (unsigned long)&rgb_set)) != OK) {
			errx(1, "failed to set boot startup colours");
		}

		close(fd);
		exit(ret);
	}

	oreoled_bl_cmd_usage();
	exit(0);
}
