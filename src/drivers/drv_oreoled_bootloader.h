/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file drv_oreoled.h
 * @author Angus Peart <angusp@gmail.com>
 * OreoLED bootloader device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/* oreoled device path */
#define OREOLEDBL0_DEVICE_PATH "/dev/oreoledbl0"

/*
 * ioctl() definitions
 */

#define _OREOLED_BL_IOC_BASE	(0x2f00)
#define _OREOLED_BL_IOC(_n)		(_IOC(_OREOLED_BL_IOC_BASE, _n))

/** send reset */
#define OREOLED_BL_RESET		_OREOLED_BL_IOC(1)

/** boot ping */
#define OREOLED_BL_PING			_OREOLED_BL_IOC(2)

/** boot version */
#define OREOLED_BL_VER			_OREOLED_BL_IOC(3)

/** boot write flash */
#define OREOLED_BL_FLASH		_OREOLED_BL_IOC(4)

/** boot application version */
#define OREOLED_BL_APP_VER		_OREOLED_BL_IOC(5)

/** boot application crc */
#define OREOLED_BL_APP_CHECKSUM	_OREOLED_BL_IOC(6)

/** boot startup colour */
#define OREOLED_BL_SET_COLOUR	_OREOLED_BL_IOC(7)

/** boot application */
#define OREOLED_BL_BOOT_APP		_OREOLED_BL_IOC(8)
