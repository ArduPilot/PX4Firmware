#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <drivers/drv_rc.h>
#include <systemlib/ppm_decode.h>

#include <board_config.h>

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

#define PPM_MIN_CHANNEL_VALUE	800		/* shortest valid channel signal */
#define PPM_MAX_CHANNEL_VALUE	2200		/* longest valid channel signal */

#if 1

#define HRT_TIMER_BASE		STM32_TIM3_BASE
#define HRT_TIMER_POWER_REG	STM32_RCC_APB1ENR
#define HRT_TIMER_POWER_BIT   RCC_APB1ENR_TIM3EN
#define HRT_TIMER_VECTOR	STM32_IRQ_TIM3
#define HRT_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN

#define REG(_reg)	(*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))

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

# define rCCR_HRT	rCCR4			/* compare register for HRT */
# define DIER_HRT	GTIM_DIER_CC4IE		/* interrupt enable for HRT */
# define SR_INT_HRT	GTIM_SR_CC4IF		/* interrupt status for HRT */

#  define rCCR_PPM	rCCR4			/* capture register for PPM */
#  define DIER_PPM	GTIM_DIER_CC4IE		/* capture interrupt (non-DMA mode) */
#  define SR_INT_PPM	GTIM_SR_CC4IF		/* capture interrupt (non-DMA mode) */
#  define SR_OVF_PPM	GTIM_SR_CC4OF		/* capture overflow (non-DMA mode) */
#  define CCMR1_PPM	0			/* not on TI1/TI2 */
#  define CCMR2_PPM	0x100			/* on TI3, not on TI4 */
#  define CCER_PPM	(GTIM_CCER_CC4E | GTIM_CCER_CC4P | GTIM_CCER_CC4NP) /* CC2, both edges */
#  define CCER_PPM_FLIP	GTIM_CCER_CC2P

#define GPIO_PPMSUM_IN            (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN9)
#endif

#if 1

#define GPIO_PPM_CH1_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN9)
#define GPIO_PPM_CH2_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN8)
#define GPIO_PPM_CH3_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)
#define GPIO_PPM_CH4_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN6)
#define GPIO_PPM_CH5_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15)
#define GPIO_PPM_CH6_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)
#define GPIO_PPM_CH7_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_PPM_CH8_IN (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)

#define PPM_TIMER3_VECTOR STM32_IRQ_TIM3
#define PPM_TIMER3_POWER_REG STM32_RCC_APB1ENR
#define PPM_TIMER3_POWER_BIT RCC_APB1ENR_TIM3EN
#define PPM_TIMER3_CLOCK STM32_APB1_TIM3_CLKIN

#define PPM_TIMER4_VECTOR STM32_IRQ_TIM4
#define PPM_TIMER4_POWER_REG STM32_RCC_APB1ENR
#define PPM_TIMER4_POWER_BIT RCC_APB1ENR_TIM4EN
#define PPM_TIMER4_CLOCK STM32_APB1_TIM4_CLKIN

#define TIMn_REG(_tim_base, _reg)	(*(volatile uint32_t *)(_tim_base + _reg))

#define TIM3_rCR1     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CR1_OFFSET)
#define TIM3_rCR2     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CR2_OFFSET)
#define TIM3_rSMCR    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_SMCR_OFFSET)
#define TIM3_rDIER    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_DIER_OFFSET)
#define TIM3_rSR      	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_SR_OFFSET)
#define TIM3_rEGR     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_EGR_OFFSET)
#define TIM3_rCCMR1   	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCMR1_OFFSET)
#define TIM3_rCCMR2   	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCMR2_OFFSET)
#define TIM3_rCCER    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCER_OFFSET)
#define TIM3_rCNT     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CNT_OFFSET)
#define TIM3_rPSC     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_PSC_OFFSET)
#define TIM3_rARR     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_ARR_OFFSET)
#define TIM3_rCCR1    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCR1_OFFSET)
#define TIM3_rCCR2    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCR2_OFFSET)
#define TIM3_rCCR3    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCR3_OFFSET)
#define TIM3_rCCR4    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_CCR4_OFFSET)
#define TIM3_rDCR     	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_DCR_OFFSET)
#define TIM3_rDMAR    	TIMn_REG(STM32_TIM3_BASE, STM32_GTIM_DMAR_OFFSET)


#define TIM4_rCR1     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CR1_OFFSET)
#define TIM4_rCR2     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CR2_OFFSET)
#define TIM4_rSMCR    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_SMCR_OFFSET)
#define TIM4_rDIER    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_DIER_OFFSET)
#define TIM4_rSR      	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_SR_OFFSET)
#define TIM4_rEGR     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_EGR_OFFSET)
#define TIM4_rCCMR1   	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCMR1_OFFSET)
#define TIM4_rCCMR2   	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCMR2_OFFSET)
#define TIM4_rCCER    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCER_OFFSET)
#define TIM4_rCNT     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CNT_OFFSET)
#define TIM4_rPSC     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_PSC_OFFSET)
#define TIM4_rARR     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_ARR_OFFSET)
#define TIM4_rCCR1    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCR1_OFFSET)
#define TIM4_rCCR2    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCR2_OFFSET)
#define TIM4_rCCR3    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCR3_OFFSET)
#define TIM4_rCCR4    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_CCR4_OFFSET)
#define TIM4_rDCR     	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_DCR_OFFSET)
#define TIM4_rDMAR    	TIMn_REG(STM32_TIM4_BASE, STM32_GTIM_DMAR_OFFSET)

#endif

static int		rc_tim_isr(int irq, void *context);

static bool _startPPMSUM = false;

void rc_init(bool startPPMSUM)
{
	_startPPMSUM = startPPMSUM;
	if(_startPPMSUM)
	{
		irq_attach(HRT_TIMER_VECTOR, rc_tim_isr);

		/* clock/power on our timer */
		modifyreg32(HRT_TIMER_POWER_REG, 0, HRT_TIMER_POWER_BIT);

		/* disable and configure the timer */
		rCR1 = 0;
		rCR2 = 0;
		rSMCR = 0;
		rDIER = DIER_HRT | DIER_PPM;
		rCCER = 0;		/* unlock CCMR* registers */
		rCCMR1 = CCMR1_PPM;
		rCCMR2 = CCMR2_PPM;
		rCCER = CCER_PPM;
		rDCR = 0;

		/* configure the timer to free-run at 1MHz */
		rPSC = (HRT_TIMER_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */

		/* run the full span of the counter */
		rARR = 0xffff;

		/* set an initial capture a little ways off */
		rCCR_HRT = 1000;

		/* generate an update event; reloads the counter, all registers */
		rEGR = GTIM_EGR_UG;

		/* enable the timer */
		rCR1 = GTIM_CR1_CEN;

		/* enable interrupts */
		up_enable_irq(HRT_TIMER_VECTOR);
	
		ppm_input_init(0xffff);
		stm32_configgpio(GPIO_PPMSUM_IN);
	}
	else
	{
		ppm_decoded_channels = 0;
		ppm_last_valid_decode = 0;
	
		irq_attach(PPM_TIMER3_VECTOR, rc_tim_isr);
		irq_attach(PPM_TIMER4_VECTOR, rc_tim_isr);

		/* clock/power on our timer */
		modifyreg32(PPM_TIMER3_POWER_REG, 0, PPM_TIMER3_POWER_BIT);
		modifyreg32(PPM_TIMER4_POWER_REG, 0, PPM_TIMER4_POWER_BIT);

		/* disable and configure the timer */
		TIM3_rCR1 = 0;
		TIM4_rCR1 = 0;
	
		TIM3_rCR2 = 0;
		TIM4_rCR2 = 0;
	
		TIM3_rSMCR = 0;
		TIM4_rSMCR = 0;
	
		TIM3_rDIER = GTIM_DIER_CC1IE | GTIM_DIER_CC2IE | GTIM_DIER_CC3IE | GTIM_DIER_CC4IE;
		TIM4_rDIER = GTIM_DIER_CC1IE | GTIM_DIER_CC2IE | GTIM_DIER_CC3IE | GTIM_DIER_CC4IE;
	
		TIM3_rCCER = 0;		/* unlock CCMR* registers */
		TIM4_rCCER = 0;		/* unlock CCMR* registers */
	
		TIM3_rCCMR1 = 0x101;
		TIM4_rCCMR1 = 0x101;
	
		TIM3_rCCMR2 = 0x101;
		TIM4_rCCMR2 = 0x101;
	
		TIM3_rCCER = GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E 
					/*| GTIM_CCER_CC1P | GTIM_CCER_CC2P | GTIM_CCER_CC3P | GTIM_CCER_CC4P 
					| GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP | GTIM_CCER_CC4NP*/;
	
		TIM4_rCCER = GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E 
					/*| GTIM_CCER_CC1P | GTIM_CCER_CC2P | GTIM_CCER_CC3P | GTIM_CCER_CC4P 
					| GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP | GTIM_CCER_CC4NP*/;
	
		TIM3_rDCR = 0;
		TIM4_rDCR = 0;

		/* configure the timer to free-run at 1MHz */
		TIM3_rPSC = (PPM_TIMER3_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */
		TIM4_rPSC = (PPM_TIMER4_CLOCK / 1000000) - 1;	/* this really only works for whole-MHz clocks */

		/* run the full span of the counter */
		TIM3_rARR = 0xffff;
		TIM4_rARR = 0xffff;

		/* set an initial capture a little ways off */
		TIM3_rCCR2 = 1000;
		TIM4_rCCR2 = 1000;

		/* generate an update event; reloads the counter, all registers */
		TIM3_rEGR = GTIM_EGR_UG;
		TIM4_rEGR = GTIM_EGR_UG;

		/* enable the timer */
		TIM3_rCR1 = GTIM_CR1_CEN;
		TIM4_rCR1 = GTIM_CR1_CEN;

		/* enable interrupts */
		up_enable_irq(PPM_TIMER3_VECTOR);
		up_enable_irq(PPM_TIMER4_VECTOR);
	
		stm32_configgpio(GPIO_PPM_CH1_IN);
		stm32_configgpio(GPIO_PPM_CH2_IN);
		stm32_configgpio(GPIO_PPM_CH3_IN);
		stm32_configgpio(GPIO_PPM_CH4_IN);
		stm32_configgpio(GPIO_PPM_CH5_IN);
		stm32_configgpio(GPIO_PPM_CH6_IN);
		stm32_configgpio(GPIO_PPM_CH7_IN);
		stm32_configgpio(GPIO_PPM_CH8_IN);
	}
}

#define calcWidth(newEdge, oldEdge) ((newEdge < oldEdge) ? newEdge + 0xffff - oldEdge : newEdge-oldEdge) 

#define proceedChannel(_timerCCER, _timerPolarity, _timerCapture, _index) \
			if(_timerCCER & _timerPolarity) \
			{ \
				_timerCCER &= ~_timerPolarity; \
				cTime = _timerCapture; \
				dTime = calcWidth(cTime, ppm_temp_buffer[_index]); \
				if (PPM_MIN_CHANNEL_VALUE < dTime && dTime < PPM_MAX_CHANNEL_VALUE) \
					ppm_buffer[_index] = dTime; \
			} \
			else \
			{ \
				ppm_temp_buffer[_index] = _timerCapture; \
				_timerCCER |= _timerPolarity; \
			} 

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];

static int
rc_tim_isr(int irq, void *context)
{
	if(!_startPPMSUM)
	{
	
		uint32_t status;
		uint16_t cTime;
		uint16_t dTime;

		if(irq == PPM_TIMER3_VECTOR)
		{
			/* copy interrupt status */
			status = TIM3_rSR;
			/* ack the interrupts we just read */
			TIM3_rSR = ~status;

		
			if(status & GTIM_SR_CC4IF) /* input capture channel4? */
			{					
				proceedChannel(TIM3_rCCER, GTIM_CCER_CC4P, TIM3_rCCR4, 0)
			}
			if(status & GTIM_SR_CC3IF) /* input capture channel3? */
			{
				proceedChannel(TIM3_rCCER, GTIM_CCER_CC3P, TIM3_rCCR3, 1)
			}
			if(status & GTIM_SR_CC1IF) /* input capture channel1? */
			{
				proceedChannel(TIM3_rCCER, GTIM_CCER_CC1P, TIM3_rCCR1, 2)
			
				if(!(TIM3_rCCER & GTIM_CCER_CC1P)) //failing
				{
					ppm_decoded_channels = 8;
					ppm_last_valid_decode = hrt_absolute_time();
				}			
			}
			if(status & GTIM_SR_CC2IF) /* input capture channel2? */
			{					
				proceedChannel(TIM3_rCCER, GTIM_CCER_CC2P, TIM3_rCCR2, 3)
			}		
		}
		else
		{
			status = TIM4_rSR;
			TIM4_rSR = ~status;
			if(status & GTIM_SR_CC3IF) /* input capture channel3? */
			{
				proceedChannel(TIM4_rCCER, GTIM_CCER_CC3P, TIM4_rCCR3, 4)
			}
			if(status & GTIM_SR_CC4IF) /* input capture channel4? */
			{
				proceedChannel(TIM4_rCCER, GTIM_CCER_CC4P, TIM4_rCCR4, 5)
			}
			if(status & GTIM_SR_CC1IF) /* input capture channel1? */
			{
				proceedChannel(TIM4_rCCER, GTIM_CCER_CC1P, TIM4_rCCR1, 6)
			}
			if(status & GTIM_SR_CC2IF) /* input capture channel2? */
			{
				proceedChannel(TIM4_rCCER, GTIM_CCER_CC2P, TIM4_rCCR2, 7)
			}
		
		}
	}
	else
	{
		uint32_t status;
		/* copy interrupt status */
		status = rSR;
		/* ack the interrupts we just read */
		rSR = ~status;
		/* was this a PPM edge? */
		if (status & (SR_INT_PPM | SR_OVF_PPM)) {
			/* if required, flip edge sensitivity */
			ppm_input_decode(status & SR_OVF_PPM, rCCR_PPM);
		}
	}
	return OK;
}