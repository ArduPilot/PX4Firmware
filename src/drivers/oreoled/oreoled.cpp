/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Randy Mackay <rmackay9@yahoo.com>
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
 * @file oreoled.cpp
 *
 * Driver for oreoled ESCs found in solo, connected via I2C.
 *
 */

#include <px4_config.h>

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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_oreoled.h>
#include <drivers/device/ringbuffer.h>

#define OPEOLED_I2C_RETRYCOUNT  2           ///< i2c retry count
#define OREOLED_TIMEOUT_USEC	1000000U	///< timeout looking for oreoleds 1 second after startup
#define OREOLED_GENERALCALL_US	2000000U	///< general call sent every 2 seconds
#define OREOLED_GENERALCALL_CMD	0x00		///< general call command sent at regular intervals

#define OREOLED_STARTUP_INTERVAL_US		(1000000U / 10U)	///< time in microseconds, run at 10hz
#define OREOLED_UPDATE_INTERVAL_US		(1000000U / 50U)	///< time in microseconds, run at 50hz

#define OREOLED_CMD_QUEUE_SIZE	10		///< up to 10 messages can be queued up to send to the LEDs

/* magic number used to verify the software reset is valid */
#define OREOLED_RESET_NONCE				0x2A

class OREOLED : public device::I2C
{
public:
	OREOLED(int bus, int i2c_addr);
	virtual ~OREOLED();

	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/* send general call on I2C bus to syncronise all LEDs */
	int				send_general_call();

	/* send cmd to an LEDs (used for testing only) */
	int				send_cmd(oreoled_cmd_t sb);

private:

	/**
	 * Start periodic updates to the LEDs
	 */
	void			start();

	/**
	 * Stop periodic updates to the LEDs
	 */
	void			stop();

	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * update the colours displayed by the LEDs
	 */
	void			cycle();

	void			startup_discovery(void);

	uint8_t			cmd_add_checksum(oreoled_cmd_t *cmd);

	/* internal variables */
	work_s			_work;							///< work queue for scheduling reads
	bool			_healthy[OREOLED_NUM_LEDS];		///< health of each LED
	uint8_t			_num_healthy;					///< number of healthy LEDs
	ringbuffer::RingBuffer	*_cmd_queue;					///< buffer of commands to send to LEDs
	uint64_t		_last_gencall;
	uint64_t		_start_time;					///< system time we first attempt to communicate with battery

	/* performance checking */
	perf_counter_t      _call_perf;
	perf_counter_t      _gcall_perf;
	perf_counter_t      _probe_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _reply_errors;
};

/* for now, we only support one OREOLED */
namespace
{
OREOLED *g_oreoled = nullptr;
}

void oreoled_usage();

extern "C" __EXPORT int oreoled_main(int argc, char *argv[]);

/* constructor */
OREOLED::OREOLED(int bus, int i2c_addr) :
	I2C("oreoled", OREOLED0_DEVICE_PATH, bus, i2c_addr, 100000),
	_work{},
	_num_healthy(0),
	_cmd_queue(nullptr),
	_last_gencall(0),
	_call_perf(perf_alloc(PC_ELAPSED, "oreoled_call")),
	_gcall_perf(perf_alloc(PC_ELAPSED, "oreoled_gcall")),
	_probe_perf(perf_alloc(PC_ELAPSED, "oreoled_probe")),
	_comms_errors(perf_alloc(PC_COUNT, "oreoled_comms_errors")),
	_reply_errors(perf_alloc(PC_COUNT, "oreoled_reply_errors"))
{
	/* initialise to unhealthy */
	memset(_healthy, 0, sizeof(_healthy));

	/* capture startup time */
	_start_time = hrt_absolute_time();
}

/* destructor */
OREOLED::~OREOLED()
{
	/* make sure we are truly inactive */
	stop();

	/* clear command queue */
	if (_cmd_queue != nullptr) {
		delete _cmd_queue;
	}

	/* free perf counters */
	perf_free(_call_perf);
	perf_free(_gcall_perf);
	perf_free(_probe_perf);
	perf_free(_comms_errors);
	perf_free(_reply_errors);
}

int
OREOLED::init()
{
	int ret;

	/* initialise I2C bus */
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/* allocate command queue */
	_cmd_queue = new ringbuffer::RingBuffer(OREOLED_CMD_QUEUE_SIZE, sizeof(oreoled_cmd_t));

	if (_cmd_queue == nullptr) {
		return ENOTTY;

	} else {
		/* start work queue */
		start();
	}

	return OK;
}

int
OREOLED::probe()
{
	/* set retry count */
	_retries = OPEOLED_I2C_RETRYCOUNT;

	/* always return true */
	return OK;
}

int
OREOLED::info()
{
	/* print health info on each LED */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i]) {
			DEVICE_LOG("oreo %u: BAD", (unsigned)i);

		} else {
			DEVICE_LOG("oreo %u: OK", (unsigned)i);
		}
	}

	/* display perf info */
	perf_print_counter(_call_perf);
	perf_print_counter(_gcall_perf);
	perf_print_counter(_probe_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_reply_errors);

	return OK;
}

void
OREOLED::start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this, 1);
}

void
OREOLED::stop()
{
	work_cancel(HPWORK, &_work);
}

void
OREOLED::cycle_trampoline(void *arg)
{
	OREOLED *dev = (OREOLED *)arg;

	/* check global oreoled and cycle */
	if (g_oreoled != nullptr) {
		dev->cycle();
	}
}

void
OREOLED::cycle()
{
	/* check time since startup */
	uint64_t now = hrt_absolute_time();
	bool startup_timeout = (now - _start_time > OREOLED_TIMEOUT_USEC);

	/* during startup period keep searching for unhealthy LEDs */
	if (!startup_timeout && _num_healthy < OREOLED_NUM_LEDS) {
		startup_discovery();

		/* schedule another attempt in 0.1 sec */
		work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
			   USEC2TICK(OREOLED_STARTUP_INTERVAL_US));
		return;
	}

	/* get next command from queue */
	oreoled_cmd_t next_cmd;

	while (_cmd_queue->get(&next_cmd, sizeof(oreoled_cmd_t))) {
		/* send valid messages to healthy LEDs */
		if ((next_cmd.led_num < OREOLED_NUM_LEDS) && _healthy[next_cmd.led_num]
		    && (next_cmd.num_bytes <= OREOLED_CMD_LENGTH_MAX)) {
			/* start performance timer */
			perf_begin(_call_perf);

			/* set I2C address */
			set_address(OREOLED_BASE_I2C_ADDR + next_cmd.led_num);

			/* Calculate XOR checksum and append to the i2c write data */
			uint8_t next_cmd_xor = cmd_add_checksum(&next_cmd);

			/* send I2C command with a retry limit */
			uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];
			for (uint8_t retry = OEROLED_COMMAND_RETRIES; retry > 0; retry--) {
				if (transfer(next_cmd.buff, next_cmd.num_bytes, reply, 3) == OK) {
					if (!(reply[1] == (OREOLED_BASE_I2C_ADDR + next_cmd.led_num) && reply[2] == next_cmd_xor)) {
						_healthy[next_cmd.led_num] = false;
						_num_healthy--;
						perf_count(_reply_errors);
					}
				} else {
					_healthy[next_cmd.led_num] = false;
					_num_healthy--;
					perf_count(_comms_errors);
				}
			}

			perf_end(_call_perf);
		}
	}

	/* send general call every 4 seconds, if we aren't bootloading*/
	if ((now - _last_gencall) > OREOLED_GENERALCALL_US) {
		perf_begin(_gcall_perf);
		send_general_call();
		perf_end(_gcall_perf);
	}

	/* schedule a fresh cycle call when the command is sent */
	work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
		   USEC2TICK(OREOLED_UPDATE_INTERVAL_US));
}

void
OREOLED::startup_discovery(void)
{
	oreoled_cmd_t cmd;
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* attempt to contact each unhealthy LED */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i]) {
			perf_begin(_probe_perf);

			/* prepare command to turn off LED */
			cmd.led_num = i;
			cmd.buff[0] = 0xAA;
			cmd.buff[1] = 0x55;
			cmd.buff[2] = OREOLED_PATTERN_OFF;
			cmd.num_bytes = 3;
			uint8_t cmd_checksum = cmd_add_checksum(&cmd);

			/* set I2C address */
			set_address(OREOLED_BASE_I2C_ADDR + cmd.led_num);

			/* send I2C command */
			if (transfer(cmd.buff, cmd.num_bytes, reply, 3) == OK) {
				/* Check for a reply with a checksum offset of 1,
				   which indicates a response from firmwares >= 1.3 */
				if (reply[1] == OREOLED_BASE_I2C_ADDR + cmd.led_num &&
				    reply[2] == (cmd_checksum + 1)) {
					DEVICE_LOG("oreoled %u ok - in application", (unsigned)i);
					_healthy[i] = true;
					_num_healthy++;
				} else {
					warnx("startup response  ADDR: 0x%x expected 0x%x", reply[1], cmd.led_num);
					warnx("startup response   CMD: 0x%x expected 0x%x", reply[2], (cmd_checksum + 1));
					perf_count(_reply_errors);
				}
			} else {
				perf_count(_comms_errors);
			}

			perf_end(_probe_perf);
		}
	}
}

uint8_t
OREOLED::cmd_add_checksum(oreoled_cmd_t *cmd)
{
	/* Calculate XOR checksum and append to the command buffer */
	cmd->num_bytes++;
	uint8_t checksum_idx = cmd->num_bytes - 1;

	/* XOR seed */
	cmd->buff[checksum_idx] = OREOLED_BASE_I2C_ADDR + cmd->led_num;

	/* XOR Calculation */
	for (uint8_t i = 0; i < checksum_idx; i++) {
		cmd->buff[checksum_idx] ^= cmd->buff[i];
	}

	return cmd->buff[checksum_idx];
}

int
OREOLED::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -ENODEV;
	oreoled_cmd_t new_cmd;

	switch (cmd) {
	case OREOLED_SET_RGB:
		/* set the specified color */
		new_cmd.led_num = ((oreoled_rgbset_t *) arg)->instance;
		new_cmd.buff[0] = ((oreoled_rgbset_t *) arg)->pattern;
		new_cmd.buff[1] = OREOLED_PARAM_BIAS_RED;
		new_cmd.buff[2] = ((oreoled_rgbset_t *) arg)->red;
		new_cmd.buff[3] = OREOLED_PARAM_BIAS_GREEN;
		new_cmd.buff[4] = ((oreoled_rgbset_t *) arg)->green;
		new_cmd.buff[5] = OREOLED_PARAM_BIAS_BLUE;
		new_cmd.buff[6] = ((oreoled_rgbset_t *) arg)->blue;
		new_cmd.num_bytes = 7;

		/* special handling for request to set all instances rgb values */
		if (new_cmd.led_num == OREOLED_ALL_INSTANCES) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				/* add command to queue for all healthy leds */
				if (_healthy[i]) {
					new_cmd.led_num = i;
					_cmd_queue->force(&new_cmd);
					ret = OK;
				}
			}

		} else if (new_cmd.led_num < OREOLED_NUM_LEDS) {
			/* request to set individual instance's rgb value */
			if (_healthy[new_cmd.led_num]) {
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_RUN_MACRO:
		/* run a macro */
		new_cmd.led_num = ((oreoled_macrorun_t *) arg)->instance;
		new_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
		new_cmd.buff[1] = OREOLED_PARAM_MACRO;
		new_cmd.buff[2] = ((oreoled_macrorun_t *) arg)->macro;
		new_cmd.num_bytes = 3;

		/* special handling for request to set all instances */
		if (new_cmd.led_num == OREOLED_ALL_INSTANCES) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				/* add command to queue for all healthy leds */
				if (_healthy[i]) {
					new_cmd.led_num = i;
					_cmd_queue->force(&new_cmd);
					ret = OK;
				}
			}

		} else if (new_cmd.led_num < OREOLED_NUM_LEDS) {
			/* request to set individual instance's rgb value */
			if (_healthy[new_cmd.led_num]) {
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_SEND_RESET:
		/* send a reset */
		new_cmd.led_num = OREOLED_ALL_INSTANCES;
		new_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
		new_cmd.buff[1] = OREOLED_PARAM_RESET;
		new_cmd.buff[2] = OREOLED_RESET_NONCE;
		new_cmd.num_bytes = 3;

		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			/* add command to queue for all healthy leds */
			if (_healthy[i]) {
				new_cmd.led_num = i;
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_SEND_BYTES:
		/* send bytes */
		new_cmd = *((oreoled_cmd_t *) arg);

		/* special handling for request to set all instances */
		if (new_cmd.led_num == OREOLED_ALL_INSTANCES) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				/* add command to queue for all healthy leds */
				if (_healthy[i]) {
					new_cmd.led_num = i;
					_cmd_queue->force(&new_cmd);
					ret = OK;
				}
			}

		} else if (new_cmd.led_num < OREOLED_NUM_LEDS) {
			/* request to set individual instance's rgb value */
			if (_healthy[new_cmd.led_num]) {
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_FORCE_SYNC:
		send_general_call();
		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}

/* send general call on I2C bus to syncronise all LEDs */
int
OREOLED::send_general_call()
{
	int ret = -ENODEV;

	/* set I2C address to zero */
	set_address(0);

	/* prepare command : 0x01 = general hardware call, 0x00 = I2C address of master (but we don't act as a slave so set to zero)*/
	uint8_t msg[] = {0x01, 0x00};

	/* send I2C command */
	if (transfer(msg, sizeof(msg), nullptr, 0) == OK) {
		ret = OK;
	}

	/* record time */
	_last_gencall = hrt_absolute_time();

	return ret;
}

/* send a cmd to an LEDs (used for testing only) */
int
OREOLED::send_cmd(oreoled_cmd_t new_cmd)
{
	int ret = -ENODEV;

	/* sanity check led number, health and cmd length */
	if ((new_cmd.led_num < OREOLED_NUM_LEDS) && _healthy[new_cmd.led_num] && (new_cmd.num_bytes < OREOLED_CMD_LENGTH_MAX)) {
		/* set I2C address */
		set_address(OREOLED_BASE_I2C_ADDR + new_cmd.led_num);

		/* add to queue */
		_cmd_queue->force(&new_cmd);
		ret = OK;
	}

	return ret;
}

void
oreoled_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'reset', 'rgb 30 40 50', 'macro 4', 'gencall', 'bytes <lednum> 7 9 6'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", OREOLED_BASE_I2C_ADDR);
}

int
oreoled_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int i2c_addr = OREOLED_BASE_I2C_ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			i2c_addr = (int)strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = (int)strtol(optarg, NULL, 0);
			break;

		default:
			oreoled_usage();
			return 0;
		}
	}

	if (optind >= argc) {
		oreoled_usage();
		return 1;
	}

	const char *verb = argv[optind];

	int ret;

	/* start driver */
	if (!strcmp(verb, "start")) {
		if (g_oreoled != nullptr) {
			errx(1, "already started");
		}

		/* by default use LED bus */
		if (i2cdevice == -1) {
			i2cdevice = PX4_I2C_BUS_LED;
		}

		/* instantiate driver */
		g_oreoled = new OREOLED(i2cdevice, i2c_addr);

		/* check if object was created */
		if (g_oreoled == nullptr) {
			errx(1, "failed to allocated memory for driver");
		}

		/* check object was created successfully */
		if (g_oreoled->init() != OK) {
			delete g_oreoled;
			g_oreoled = nullptr;
			errx(1, "failed to start driver");
		}

		return 0;
	}

	/* need the driver past this point */
	if (g_oreoled == nullptr) {
		warnx("not started");
		oreoled_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		int fd = open(OREOLED0_DEVICE_PATH, O_RDWR);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		/* structure to hold desired colour */
		oreoled_rgbset_t rgb_set_red = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, 0xFF, 0x0, 0x0};
		oreoled_rgbset_t rgb_set_blue = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, 0x0, 0x0, 0xFF};
		oreoled_rgbset_t rgb_set_off = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_OFF, 0x0, 0x0, 0x0};

		/* flash red and blue for 3 seconds */
		for (uint8_t i = 0; i < 30; i++) {
			/* red */
			if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_red)) != OK) {
				errx(1, " failed to update rgb");
			}

			/* sleep for 0.05 seconds */
			usleep(50000);

			/* blue */
			if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_blue)) != OK) {
				errx(1, " failed to update rgb");
			}

			/* sleep for 0.05 seconds */
			usleep(50000);
		}

		/* turn off LED */
		if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_off)) != OK) {
			errx(1, " failed to turn off led");
		}

		close(fd);
		return ret;
	}

	/* display driver status */
	if (!strcmp(verb, "info")) {
		g_oreoled->info();
		return 0;
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		/* turn off LED */
		oreoled_rgbset_t rgb_set_off = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_OFF, 0x0, 0x0, 0x0};
		ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_off);

		close(fd);

		/* delete the oreoled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {
			OREOLED *tmp_oreoled = g_oreoled;
			g_oreoled = nullptr;
			delete tmp_oreoled;
			return 0;
		}

		return ret;
	}

	/* send rgb request to all LEDS */
	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			errx(1, "Usage: oreoled rgb <red> <green> <blue>");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		uint8_t red = (uint8_t)strtol(argv[2], NULL, 0);
		uint8_t green = (uint8_t)strtol(argv[3], NULL, 0);
		uint8_t blue = (uint8_t)strtol(argv[4], NULL, 0);
		oreoled_rgbset_t rgb_set = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, red, green, blue};

		if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set)) != OK) {
			errx(1, "failed to set rgb");
		}

		close(fd);
		return ret;
	}

	/* send macro request to all LEDS */
	if (!strcmp(verb, "macro")) {
		if (argc < 3) {
			errx(1, "Usage: oreoled macro <macro_num>");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		uint8_t macro = (uint8_t)strtol(argv[2], NULL, 0);

		/* sanity check macro number */
		if (macro > OREOLED_PARAM_MACRO_ENUM_COUNT) {
			errx(1, "invalid macro number %d", (int)macro);
			return ret;
		}

		oreoled_macrorun_t macro_run = {OREOLED_ALL_INSTANCES, (enum oreoled_macro)macro};

		if ((ret = ioctl(fd, OREOLED_RUN_MACRO, (unsigned long)&macro_run)) != OK) {
			errx(1, "failed to run macro");
		}

		close(fd);
		return ret;
	}

	/* send reset request to all LEDS */
	if (!strcmp(verb, "reset")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled reset");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_SEND_RESET, 0)) != OK) {
			errx(1, "failed to run macro");
		}

		close(fd);
		return ret;
	}

	/* send general hardware call to all LEDS */
	if (!strcmp(verb, "gencall")) {
		ret = g_oreoled->send_general_call();
		warnx("sent general call");
		return ret;
	}

	/* send a string of bytes to an LED using send_bytes function */
	if (!strcmp(verb, "bytes")) {
		if (argc < 3) {
			errx(1, "Usage: oreoled bytes <led_num> <byte1> <byte2> <byte3> ...");
		}

		/* structure to be sent */
		oreoled_cmd_t sendb;

		/* maximum of 20 bytes can be sent */
		if (argc > 20 + 3) {
			errx(1, "Max of 20 bytes can be sent");
		}

		/* check led num */
		sendb.led_num = (uint8_t)strtol(argv[optind + 1], NULL, 0);

		if (sendb.led_num > 3) {
			errx(1, "led number must be between 0 ~ 3");
		}

		/* get bytes */
		sendb.num_bytes = argc - (optind + 2);
		uint8_t byte_count;

		for (byte_count = 0; byte_count < sendb.num_bytes; byte_count++) {
			sendb.buff[byte_count] = (uint8_t)strtol(argv[byte_count + optind + 2], NULL, 0);
		}

		/* send bytes */
		if ((ret = g_oreoled->send_cmd(sendb)) != OK) {
			errx(1, "failed to send command");

		} else {
			warnx("sent %d bytes", (int)sendb.num_bytes);
		}

		return ret;
	}

	oreoled_usage();
	return 0;
}
