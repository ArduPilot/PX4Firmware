/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * @file px4fmu_dma_bb.c
 *
 * DMA based Bit Bang driver
 * Author: Siddharth Bharat Purohit
 *
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_dma_bitbang.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

#ifdef DMAMAP_BITBANG

#if   TIMER_BITBANG == 1
# define DMA_TIMER_BASE			STM32_TIM1_BASE
# define DMA_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define DMA_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define DMA_TIMER_VECTOR		STM32_IRQ_TIM1CC
# define DMA_TIMER_CLOCK		STM32_APB2_TIM1_CLKIN
#elif TIMER_BITBANG == 8
# define DMA_TIMER_BASE			STM32_TIM8_BASE
# define DMA_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define DMA_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define DMA_TIMER_VECTOR		STM32_IRQ_TIM8CC
# define DMA_TIMER_CLOCK		STM32_APB2_TIM8_CLKIN
#else
#  error must select Timer 1 or Timer 8 DMA2 doesnot support any other
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(DMA_TIMER_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)


#if TIMER_BITBANG_CH == 1
# define rCCR_DMA		rCCR1				/* compare register */
# define DIER_DMA_INT	GTIM_DIER_CC1IE		/* interrupt enable */
# define DIER_DMA_REQ	GTIM_DIER_CC1DE		/* DMA request enable */
# define SR_INT_DMA		GTIM_SR_CC1IF		/* interrupt status */
#elif TIMER_BITBANG_CH == 2
# define rCCR_DMA		rCCR2				/* compare register */
# define DIER_DMA_INT	GTIM_DIER_CC2IE		/* interrupt enable */
# define DIER_DMA_REQ	GTIM_DIER_CC2DE		/* DMA request enable */
# define SR_INT_DMA		GTIM_SR_CC2IF		/* interrupt status */
#elif TIMER_BITBANG_CH == 3
# define rCCR_DMA		rCCR3				/* compare register */
# define DIER_DMA_INT	GTIM_DIER_CC3IE		/* interrupt enable */
# define DIER_DMA_REQ	GTIM_DIER_CC3DE		/* DMA request enable */
# define SR_INT_DMA		GTIM_SR_CC3IF		/* interrupt status */
#elif TIMER_BITBANG_CH == 4
# define rCCR_DMA		rCCR4				/* compare register */
# define DIER_DMA_INT	GTIM_DIER_CC4IE		/* interrupt enable */
# define DIER_DMA_REQ	GTIM_DIER_CC4DE		/* DMA request enable */
# define SR_INT_DMA		GTIM_SR_CC4IF		/* interrupt status */
#else
# error TIMER_BITBANG_CH must be a value between 1 and 4
#endif

static DMA_HANDLE	tx_dma;

static void		dma_reset(void);
static volatile bool dma_bitbang_initialized = false;

/* GPIO register accessors */
#define GPIO_REG(_x)		(*(volatile uint32_t *)(BITBANG_GPIO_REG_BASE + _x))
#define rBSSR		GPIO_REG(STM32_GPIO_BSRR_OFFSET)

// The rate of timer will be set per following equation
// Timer_Rate = {(16.8*mult + offset)/168} Mhz
// The fastest clock which can be generated would be:
// Clk_fast = Timer_Rate/4 Hz
void dma_bb_init(uint32_t mult, uint32_t offset)
{
	/* clock/power on our timer */
	modifyreg32(DMA_TIMER_POWER_REG, 0, DMA_TIMER_POWER_BIT);

	tx_dma = stm32_dmachannel(DMAMAP_BITBANG);

	/* configure the timer according to the requested factors*/
	rPSC = roundf(16.8f * mult + offset) - 1;

	/* run to only one increment*/
	rARR = 1;

	rCCR_DMA = 1;

	rCR1 = 0;
	rCR2 = 0;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	dma_bitbang_initialized = true;
	printf("[init] Starting DMA bitbang driver \n");
}

/* 
Sends Set or Reset data to GPIO Register popping data from top at every timer compare

 -- "callback" is called after completion the set of requested pulse is transmitted through 
 to GPIO bus, 
 -- with "arg" being the argument to the callback
 -- "dat" contains the list of pin states over time, data is sent to  GPIOx_BSSR register which
 has following behaviour:

 -- Bits 31:16 BRx
		0: No action
		1: Reset the corresponding GPIOx O/P bit
 
 -- Bits 15:0 BSx
		0: No action
		1: Set the corresponding GPIOx O/P bit

Note: If both BSx and BRx are set, BSx has priority.
*/
void dma_bb_send_buff(uint32_t* dat, uint32_t len, dma_callback_t callback, void* arg)
{
	if (!dma_bitbang_initialized) {
		return;
	}
	/* unconfigure the timer request */
	rDIER &= ~DIER_DMA_REQ;
	dma_reset();

	stm32_dmasetup(
		tx_dma,
		(uint32_t)&rBSSR,
		(uint32_t)dat,
		len,
		DMA_SCR_DIR_M2P		 |
		DMA_SCR_MINC		 |
		DMA_SCR_PSIZE_32BITS |
		DMA_SCR_MSIZE_32BITS);
	stm32_dmastart(tx_dma, callback, arg, false);

	/* Enable Timer request */
	rDIER |= DIER_DMA_REQ;
}

static void
dma_reset(void)
{
	/* kill any pending DMA */
	stm32_dmastop(tx_dma);
}

#endif //#ifdef DMAMAP_BITBANG
