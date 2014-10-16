/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file pcf8575.cpp
 * @author Jonas Vantilt
 *
 * Driver for IO board PCF8575 via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_ioexpander.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

/* Configuration Constants */
#define PCF8575_BUS 			PX4_I2C_BUS_EXPANSION
#define PCF8575_ADDRESS	0x20 /* 7-bit address. 8-bit address is 0xE0 */
#define PCF8575_DEVICE_PATH	"/dev/pcf8575"

/* PCF8575 Registers addresses */

#define PCF8575_FIRE_NONE	0x00
#define PCF8575_FIRE_SONAR_1	0x01		/* Measure range Register */
#define PCF8575_FIRE_SONAR_2	0x02
#define PCF8575_FIRE_SONAR_3	0x04
#define PCF8575_FIRE_SONAR_4	0x08
#define PCF8575_FIRE_SONAR_5	0x10
#define PCF8575_FIRE_SONAR_6	0x20

#define PCF8575_CONVERSION_INTERVAL 100000 /* 100ms */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class PCF8575 : public device::I2C
{
public:
	PCF8575(int bus = PCF8575_BUS, int address = PCF8575_ADDRESS);
	virtual ~PCF8575();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void			print_info();

protected:
	virtual int		probe();

private:
	work_s			_work;
	RingBuffer		*_reports;
	int			_measure_ticks;
	int			_class_instance;
	bool			_sensor_ok;

	orb_advert_t		_ioexpander_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	
	uint8_t 		_val;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int				probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int				measure(uint8_t val);
	int				collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pcf8575_main(int argc, char *argv[]);

PCF8575::PCF8575(int bus, int address) :
	I2C("PCF8575", PCF8575_DEVICE_PATH, bus, address, 100000),
	_reports(nullptr),
	_measure_ticks(0),
	_ioexpander_topic(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "pcf8575_read")),
	_comms_errors(perf_alloc(PC_COUNT, "pcf8575_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "pcf8575_buffer_overflows"))
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;
	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

PCF8575::~PCF8575()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(IOEXPANDER_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
PCF8575::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(ioexpander_report));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(IOEXPANDER_DEVICE_PATH);
	if (_class_instance == CLASS_DEVICE_PRIMARY) {	
	  /* get a publish handle on the range finder topic */
	  struct ioexpander_report report;
	  measure(0);
	  _reports->get(&report);
	  _ioexpander_topic = orb_advertise(ORB_ID(sensor_ioexpander), &report);
	  if (_ioexpander_topic < 0) {
	    debug("failed to create sensor_range_finder object. Did you start uOrb?");
	  }
	}
	
	ret = OK;
out:
	return ret;
}

int
PCF8575::probe()
{
	return measure(0);
}

int
PCF8575::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(PCF8575_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(PCF8575_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
PCF8575::read(struct file *filp, char *buffer, size_t buflen)
{
	_val = 0;
	unsigned count = buflen / sizeof(struct ioexpander_report);
	struct ioexpander_report *rbuf = reinterpret_cast<struct ioexpander_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		log("Buffer to small");
		return -ENOSPC;
	}
	warnx("count = %d", count);
	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			warnx("in while: rbuf = %d", rbuf);
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
				warnx("report->get is gelukt");
			}
		}
		warnx("groote buffer %d", sizeof(*rbuf));
		warnx("ret = %d",ret);
		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
		warnx("er is data!");
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure(_val)) {
			ret = -EIO;
			warnx("measure faalt");
			break;
		}

		/* wait for it to complete */
		usleep(PCF8575_CONVERSION_INTERVAL);

		collect();

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
PCF8575::measure(uint8_t val)
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = PCF8575_FIRE_NONE;
	switch (val) {
	
	case 0:
		cmd = PCF8575_FIRE_NONE;
		break;

	case 1:
		cmd = PCF8575_FIRE_SONAR_1;
		break;

	case 2:
		cmd = PCF8575_FIRE_SONAR_2;
		break;

	case 3:
		cmd = PCF8575_FIRE_SONAR_3;
		break;

	case 4:
		cmd = PCF8575_FIRE_SONAR_4;
		break;

	case 5:
		cmd = PCF8575_FIRE_SONAR_5;
		break;

	case 6:
		cmd = PCF8575_FIRE_SONAR_6;
		break;
	default:
		cmd = PCF8575_FIRE_NONE;
		break;
	}

	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int
PCF8575::collect()
{
	//int	ret = -EIO;
	
	struct ioexpander_report report;
	/*if (OK != ret) {
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		report.succes = false;
		report.timestamp = hrt_absolute_time();
	}
	else{*/
		report.timestamp = hrt_absolute_time();
		report.succes = true;
	//}


	if (_ioexpander_topic >= 0) {
		/* publish it */
		orb_publish(ORB_ID(sensor_ioexpander), _ioexpander_topic, &report);
	}

	/* post a report to the ring */
	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	perf_end(_sample_perf);
	return OK;
}


void
PCF8575::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&PCF8575::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_RANGEFINDER
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
PCF8575::stop()
{
	work_cancel(HPWORK, &_work);
}

void
PCF8575::cycle_trampoline(void *arg)
{
	PCF8575 *dev = (PCF8575 *)arg;

	dev->cycle();
}

void
PCF8575::cycle()
{
	
	/* measurement phase */
	if (OK != measure(_val)) {
		log("measure error");
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&PCF8575::cycle_trampoline,
		   this,
		   USEC2TICK(PCF8575_CONVERSION_INTERVAL));
}

void
PCF8575::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace pcf8575
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

PCF8575	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new PCF8575(PCF8575_BUS);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(PCF8575_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct ioexpander_report report;
	ssize_t sz;
	int ret;

	int fd = open(PCF8575_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'pcf8575 start' if the driver is not running", PCF8575_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		warnx("sz: %d", sz);
		warnx("size report: %d", sizeof(report));
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("time:        %lld", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(PCF8575_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
pcf8575_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		pcf8575::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		pcf8575::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		pcf8575::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		pcf8575::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		pcf8575::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
