/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file pwm_input.c
 *
 * Timer used for reading in PWM signals, with an emphasis on
 * reading encoders such as the US Digital MA3. These are good
 * for vane-type angle of attack and sideslip sensors.
 *
 * This can use any general or advanced STM32 timer.
 *
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <board_config.h>
#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/subsystem_info.h>

#ifdef PWMIN_TIMER

#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

/* PWMIN configuration */
#if PWMIN_TIMER == 1
# define PWMIN_TIMER_BASE	STM32_TIM1_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM1_CLKIN
# define PWMIN_TIMER_LIMIT	UINT16_MAX
# if CONFIG_STM32_TIM1 
#  error must not set CONFIG_STM32_TIM1=y and PWMIN_TIMER=1
# endif
#elif PWMIN_TIMER == 2
# define PWMIN_TIMER_BASE	STM32_TIM2_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM2EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM2
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM2_CLKIN
# define PWMIN_TIMER_LIMIT	UINT32_MAX
# if CONFIG_STM32_TIM2
#  error must not set CONFIG_STM32_TIM2=y and PWMIN_TIMER=2
# endif
#elif PWMIN_TIMER == 3
# define PWMIN_TIMER_BASE	STM32_TIM3_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM3EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM3
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
# define PWMIN_TIMER_LIMIT 	UINT16_MAX
# if CONFIG_STM32_TIM3
#  error must not set CONFIG_STM32_TIM3=y and PWMIN_TIMER=3
# endif
#elif PWMIN_TIMER == 4
# define PWMIN_TIMER_BASE	STM32_TIM4_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM4EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM4
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
# define PWMIN_TIMER_LIMIT 	UINT16_MAX
# if CONFIG_STM32_TIM4
#  error must not set CONFIG_STM32_TIM4=y and PWMIN_TIMER=4
# endif
#elif PWMIN_TIMER == 5
# define PWMIN_TIMER_BASE	STM32_TIM5_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM5EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM5
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
# define PWMIN_TIMER_LIMIT 	UINT32_MAX
# if CONFIG_STM32_TIM5
#  error must not set CONFIG_STM32_TIM5=y and PWMIN_TIMER=5
# endif
#elif PWMIN_TIMER == 8
# define PWMIN_TIMER_BASE	STM32_TIM8_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM8CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN
# define PWMIN_TIMER_LIMIT	UINT16_MAX
# if CONFIG_STM32_TIM8
#  error must not set CONFIG_STM32_TIM8=y and PWMIN_TIMER=8
# else
#  error PWMIN_TIMER must be 1, 2, 3, 4, 5, 8 or undefined
# endif
#endif

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Maximum period for PWM input.
 * 5ms is valid for the US Digital MA3 encoders.
 */
#define PWMIN_MAX_PERIOD_SECONDS 	0.005

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_GTIM_CR1_OFFSET)
#define rCR2		REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR		REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER		REG(STM32_GTIM_DIER_OFFSET)
#define rSR		REG(STM32_GTIM_SR_OFFSET)
#define rEGR		REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_GTIM_CCER_OFFSET)
#define rCNT		REG(STM32_GTIM_CNT_OFFSET)
#define rPSC		REG(STM32_GTIM_PSC_OFFSET)
#define rARR		REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1		REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR		REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR		REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if PWMIN_TIMER_CHANNEL == 1
 #define rCCR_PWMIN_A		rCCR1			/* compare register for PWMIN */
 #define DIER_PWMIN_A		(GTIM_DIER_CC1IE) 	/* interrupt enable for PWMIN */
 #define SR_INT_PWMIN_A		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
 #define rCCR_PWMIN_B		rCCR2 			/* compare register for PWMIN */
 #define SR_INT_PWMIN_B		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
 #define CCMR1_PWMIN		((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
 #define CCMR2_PWMIN		0
 #define CCER_PWMIN		(GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
 #define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
 #define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
 #define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#elif PWMIN_TIMER_CHANNEL == 2
 #define rCCR_PWMIN_A		rCCR2			/* compare register for PWMIN */
 #define DIER_PWMIN_A		(GTIM_DIER_CC2IE)	/* interrupt enable for PWMIN */
 #define SR_INT_PWMIN_A		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
 #define rCCR_PWMIN_B		rCCR1			/* compare register for PWMIN */
 #define DIER_PWMIN_B		GTIM_DIER_CC1IE		/* interrupt enable for PWMIN */
 #define SR_INT_PWMIN_B		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
 #define CCMR1_PWMIN		((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
 #define CCMR2_PWMIN		0
 #define CCER_PWMIN		(GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
 #define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
 #define SMCR_PWMIN_1		(0x06 << GTIM_SMCR_TS_SHIFT)
 #define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#else
 #error PWMIN_TIMER_CHANNEL must be either 1 and 2.
#endif


/* timer-specific functions */
static void	pwmin_tim_init(void);
static int	pwmin_tim_isr(int irq, void *context);

static void	pwmin_decode(uint16_t status);

void		pwmin_init(void);

/* ORB message setup */
orb_advert_t _pwm_pub = -1;

/*
 * Initialise the timer we are going to use.
 */
static void pwmin_tim_init(void) {
	/* claim our interrupt vector */
	irq_attach(PWMIN_TIMER_VECTOR, pwmin_tim_isr);

	/* Clear no bits, set timer enable bit.*/
	modifyreg32(PWMIN_TIMER_POWER_REG, 0, PWMIN_TIMER_POWER_BIT); 

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMIN_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMIN;
	rCCMR2 = CCMR2_PWMIN;
	rSMCR = SMCR_PWMIN_1;	/* Set up mode */
	rSMCR = SMCR_PWMIN_2;	/* Enable slave mode controller */
	rCCER = CCER_PWMIN;
	rDCR = 0;


	/* 
	 * Calculate the prescaler, rounding up to ensure that 
	 * the timer period definitely exceeds the signal period.
	 */
	double prescaler_fl = ceil((PWMIN_MAX_PERIOD_SECONDS * PWMIN_TIMER_CLOCK) / (PWMIN_TIMER_LIMIT + 1.0));
	uint64_t prescaler_in = (uint64_t) (prescaler_fl);

	if (prescaler_in > (UINT16_MAX + 1)) {
	/*
	 * This is very unusual. For the prescaler to need to be 
	 * over UINT16_MAX there must be either an extremely high clock
	 * speed or an extremely long period. At 168MHz input clock and
	 * using a 16-bit timer, the period would be over 25 seconds.
	 * (normal period for the encoders targeted here is 4.1ms).
	 * As it's so unlikely, the simplest approach is taken: assume
	 * that the period will actually not be as long as claimed.
	 */
		prescaler_in = UINT16_MAX + 1;
	} else if (prescaler_in < 1) {
		prescaler_in = 1;
	}

	/* 
	 * Define the clock speed. We want the highest possible clock 
	 * speed that avoids overflows.
	 */
	rPSC = prescaler_in - 1;

	/* run the full span of the counter */
	rARR = PWMIN_TIMER_LIMIT;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	/* enable interrupts */
	up_enable_irq(PWMIN_TIMER_VECTOR);
}


static void pwmin_decode(uint16_t status)
{
	static uint64_t error_count = 0;
	uint32_t period = rCCR_PWMIN_A;
	uint32_t pulse_width = rCCR_PWMIN_B;

	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PWMIN) {
		error_count++;
		return;
	}

	struct pwm_input_s pwmin_report;
	pwmin_report.timestamp = hrt_absolute_time();
	pwmin_report.error_count = error_count;
	pwmin_report.period = period;
	pwmin_report.pulse_width = pulse_width;

	if (_pwm_pub != -1) {
		/* publish for subscribers */
		orb_publish(ORB_ID(pwm_input), _pwm_pub, &pwmin_report);
	}
	return;
}


/*
 * Handle the compare interupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int pwmin_tim_isr(int irq, void *context)
{
	uint16_t status;

	/* copy interrupt status */
	status = rSR;

	/* ack the interrupts we just read */
	rSR = ~status;

	pwmin_decode(status);
	return OK;
}

/*
 * Initalise the high-resolution timing module.
 */
void pwmin_init(void)
{
	static uint8_t start_number = 0;

	pwmin_tim_init();

	stm32_configgpio(GPIO_PWM_IN);

	struct pwm_input_s zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_pwm_pub = orb_advertise(ORB_ID(pwm_input), &zero_report);

	if (_pwm_pub < 0) {
		warnx("failed to create PWM input object. Did you start uOrb?");
	}

	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_ENCODER
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}
#endif

int pwm_input_main(int argc, char * argv[]) {
#ifdef PWMIN_TIMER
	pwmin_init();
	return 0;
#else
	return 1;
#endif
}


 /* PWMIN_TIMER */
