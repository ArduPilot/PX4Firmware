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
 * @file vrinput.cpp
 * Driver for the VRBRAIN board.
 *
 *
 */
#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <crc32.h>

extern "C" uint64_t	ppm_last_valid_decode;

#include <arch/board/board.h>
#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
//#include <drivers/drv_sbus.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
//#include <drivers/vrbrain/vrinput/controls/controls.h>
#include <uORB/topics/rc_channels.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_rc.h>
#include <systemlib/ppm_decode.h>

#ifndef ARDUPILOT_BUILD
# define RC_HANDLING_DEFAULT false
#else
// APM uses raw PWM output
# define RC_HANDLING_DEFAULT true
#endif
/**
 * The VRBRAIN class.
 *
 *
 */
class F4BY_INPUT : public device::CDev
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	F4BY_INPUT();
	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~F4BY_INPUT();
	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 */
	virtual int		init();
	/**
	 * Push failsafe values to IO.
	 *
	 * @param[in] vals	Failsafe control inputs: in us PPM (900 for zero, 1500 for centered, 2100 for full)
	 * @param[in] len	Number of channels, could up to 8
	 */
	int			set_failsafe_values(const uint16_t *vals, unsigned len);
	/**
	 * Disable RC input handling
	 */
private:
	// XXX
	enum InputType
	{
		ePPM = 1,
		ePPMSUM,
		eSBUS,
		eDSM
	};
		
	unsigned		_max_rc_input;		///< Maximum receiver channels supported by PX4IO

	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp
	volatile int		_task;			///< worker task id
	volatile bool		_task_should_exit;	///< worker terminate flag
	perf_counter_t		_perf_chan_count;	///<local performance counter for channel number changes
	/* advertised topics */
	orb_advert_t 		_to_input_rc;		///< rc inputs from io
	InputType 			_inputType;
	uint16_t	r_raw_rc_values[18];
	uint16_t	r_raw_rc_count;
	/**
	 * Trampoline to the worker task
	 */
	static void		task_main_trampoline(int argc, char *argv[]);
	/**
	 * worker task
	 */
	void			task_main();

	void		controls_tick();
	void		controls_init();
};
namespace
{
F4BY_INPUT	*g_dev = nullptr;
}
F4BY_INPUT::F4BY_INPUT() :
	CDev("f4by_input", F4BY_INPUT_DEVICE_PATH),
	_max_rc_input(0),
	_rc_chan_count(0),
	_rc_last_valid(0),
	_task(-1),
	_task_should_exit(false),
	_perf_chan_count(perf_alloc(PC_COUNT, "io rc #")),
	_to_input_rc(0),
	_inputType(ePPM)
{
	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	_debug_enabled = false;
}
F4BY_INPUT::~F4BY_INPUT()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;
	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}
	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
	g_dev = nullptr;
}

int
F4BY_INPUT::init()
{

	int ret;
	ASSERT(_task == -1);
	/* do regular cdev init */
	ret = CDev::init();
	if (ret != OK)
		return ret;

	/* start the IO interface task */
	_task = task_create("f4by_input", SCHED_PRIORITY_ACTUATOR_OUTPUTS, 2048, (main_t)&F4BY_INPUT::task_main_trampoline, nullptr);
	if (_task < 0) {
		//debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}
void
F4BY_INPUT::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}
void
F4BY_INPUT::task_main()
{
	//log("controls_init");
	controls_init();
	/* lock against the ioctl handler */
	lock();
	/* loop talking to IO */
	while (!_task_should_exit) {
		controls_tick();
		usleep(1000);
	}
	unlock();

	//debug("exiting");
	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

extern "C" int    dsm_init(const char *device);
extern "C" bool   dsm_input(uint16_t *values, uint16_t *num_values, uint16_t *rssi);

extern "C" int    sbus_init(const char *device);
extern "C" bool   sbus_input(uint16_t *values, uint16_t *num_values, uint16_t *rssi, uint16_t max_channels);

static bool
ppm_input(uint16_t *values, uint16_t *num_values)
{
	bool result = false;

	/* avoid racing with PPM updates */
	irqstate_t state = irqsave();

	/*
	 * If we have received a new PPM frame within the last 200ms, accept it
	 * and then invalidate it.
	 */
	if (hrt_elapsed_time(&ppm_last_valid_decode) < 200000) {

		/* PPM data exists, copy it */
		*num_values = ppm_decoded_channels;
		if (*num_values > 16)
			*num_values = 16;

		for (unsigned i = 0; i < *num_values; i++)
			values[i] = ppm_buffer[i];

		/* clear validity */
		ppm_last_valid_decode = 0;

		/* good if we got any channels */
		result = (*num_values > 0);
	}

	irqrestore(state);

	return result;
}

void F4BY_INPUT::controls_tick()
{
	uint16_t rssi;

	struct rc_input_values rc_in;

	memset(&rc_in, 0, sizeof(rc_in));
	rc_in.input_source = 0;
	
	bool updated = false;
	
	if(_inputType == eDSM)
	{
		//perf_begin(c_gather_dsm);
		updated = dsm_input(r_raw_rc_values, &r_raw_rc_count, &rssi);
	}
	else if(_inputType == eSBUS)
	{
		//perf_begin(c_gather_sbus);
		updated = sbus_input(r_raw_rc_values, &r_raw_rc_count, &rssi, 18 /* XXX this should be INPUT channels, once untangled */);
		//perf_end(c_gather_sbus);
	}
	else
	{
		//perf_begin(c_gather_ppm);
		updated = ppm_input(r_raw_rc_values, &r_raw_rc_count);
		if (updated) {

			/* XXX sample RSSI properly here */
			rssi = 255;
		}
		//perf_end(c_gather_ppm);
	}
	if(updated)
	{
		rc_in.channel_count = r_raw_rc_count;
		if (rc_in.channel_count > 8) 
		{
			rc_in.channel_count = 8;
		}

		for (uint8_t i = 0; i < rc_in.channel_count; i++) 
		{
			rc_in.values[i] = r_raw_rc_values[i];
		}
		rc_in.rssi = rssi;
		rc_in.timestamp_publication = hrt_absolute_time();
		rc_in.timestamp_last_signal = hrt_absolute_time();
		/* lazily advertise on first publication */
		if (_to_input_rc == 0) {
			_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_in);

		} else {
			orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_in);
		}
		r_raw_rc_count = 0;
		r_raw_rc_values[18] = {0};
	}
}

#define IN2_INPUT (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN8)
#define IN4_INPUT (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)

#define IN1_OUTPUT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN9)
#define IN3_OUTPUT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)


void F4BY_INPUT::controls_init()
{
	/**
	1-2 DSM
	2-3 SBUS
	3-4 PPMSUM
	PIN3 - USART6_RX
	*/
	
	stm32_configgpio(IN2_INPUT);
	stm32_configgpio(IN1_OUTPUT);
	
	uint8_t i;
	uint8_t test = 0x5a; 
	uint8_t dsm = 0;
	uint8_t sbus = 0;
	uint8_t sppm = 0; 
	//r_raw_rc_values[18] = {0};
	r_raw_rc_count = 0;


	for(i = 0;i<32;i++)
	{  
		if ((test>>(i&0x07))&0x01) 
		{
			stm32_gpiowrite(IN1_OUTPUT,1);
			while (!stm32_gpioread(IN1_OUTPUT));
			if (!stm32_gpioread(IN2_INPUT)) dsm++;  
		} 
		else 
		{
			stm32_gpiowrite(IN1_OUTPUT,0);
			while (stm32_gpioread(IN1_OUTPUT));
			if (stm32_gpioread(IN2_INPUT)) dsm++;
		}
	}
	
	if(!dsm)
	{
		//log("DSM input");
		_inputType = eDSM;
		dsm_init("/dev/ttyS5");
		return;
	}
	
	
	stm32_configgpio(IN3_OUTPUT);
	
	for(i = 0;i<32;i++)
	{  
		if ((test>>(i&0x07))&0x01) 
		{
			stm32_gpiowrite(IN3_OUTPUT,1);
			while (!stm32_gpioread(IN3_OUTPUT));
			if (!stm32_gpioread(IN2_INPUT)) sbus++;
			
		} 
		else 
		{
			stm32_gpiowrite(IN3_OUTPUT,0);
			while (stm32_gpioread(IN3_OUTPUT));
			if (stm32_gpioread(IN2_INPUT)) sbus++;
		}
	}
	
	if(!sbus)
	{
		//log("S.BUS input");
		_inputType = eSBUS;
		sbus_init("/dev/ttyS5");
		return;
	}
	
	unregister_driver("/dev/ttyS5");
	stm32_configgpio(IN4_INPUT);
	
	for(i = 0;i<32;i++)
	{  
		if ((test>>(i&0x07))&0x01) 
		{
			stm32_gpiowrite(IN3_OUTPUT,1);
			while (!stm32_gpioread(IN3_OUTPUT));
			if (!stm32_gpioread(IN4_INPUT)) sppm++;
		} 
		else 
		{
			stm32_gpiowrite(IN3_OUTPUT,0);
			while (stm32_gpioread(IN3_OUTPUT));
			if (stm32_gpioread(IN4_INPUT)) sppm++;
		}
	}
	
	if(!sppm)
	{
		//log("PPMSUM input");
		_inputType = ePPMSUM;
		rc_init(true);
		return;
	}else 
	{
		//log("PPM input");
		rc_init(false);
	}
}

extern "C" __EXPORT int f4by_input_main(int argc, char *argv[]);
namespace
{
void
start(int argc, char *argv[])
{
	if (g_dev != nullptr)
		errx(0, "already loaded");
	/* create the driver - it will set g_dev */
	(void)new F4BY_INPUT();
	if (g_dev == nullptr) {
		errx(1, "driver alloc failed");
	}
	if (OK != g_dev->init()) {
		delete g_dev;
		g_dev = nullptr;
		errx(1, "driver init failed");
	}
	exit(0);
}
} /* namespace */

int
f4by_input_main(int argc, char *argv[])
{
	/* check for sufficient number of arguments */
	if (argc < 2)
		goto out;
	if (!strcmp(argv[1], "start"))
		start(argc - 1, argv + 1);
	/* commands below here require a started driver */
	if (g_dev == nullptr)
		errx(1, "not started");
	if (!strcmp(argv[1], "stop")) {
		/* stop the driver */
		delete g_dev;
		g_dev = nullptr;
		exit(0);
	}

out:
	errx(1, "need a command, try 'start', 'stop'");
}
