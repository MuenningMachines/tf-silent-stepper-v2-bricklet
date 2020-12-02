/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * timer_irq.c: CCU8 timer with IRQ
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "timer_irq.h"

#include "xmc_ccu8.h"
#include "xmc_scu.h"
#include "xmc_gpio.h"

void timer_irq_init(void) {
	XMC_CCU8_SetModuleClock(CCU80, XMC_CCU8_CLOCK_SCU);
	XMC_CCU8_Init(CCU80, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
	XMC_CCU8_StartPrescaler(CCU80);
	XMC_CCU8_EnableClock(CCU80, 0);

	XMC_CCU8_SLICE_COMPARE_CONFIG_t ccu8_slice_config = {
		.timer_mode          = XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
		.monoshot            = XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
		.shadow_xfer_clear   = false,
		.dither_timer_period = false,
		.dither_duty_cycle   = false,
		.prescaler_mode      = XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
		.mcm_ch1_enable      = false,
		.mcm_ch2_enable      = false,
		.slice_status        = XMC_CCU8_SLICE_STATUS_CHANNEL_1,
		.passive_level_out0  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
		.passive_level_out1  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
		.passive_level_out2  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
		.passive_level_out3  = XMC_CCU8_SLICE_OUTPUT_PASSIVE_LEVEL_LOW, 
		.asymmetric_pwm      = false,
		.selector_out0       = XMC_CCU8_SOURCE_OUT0_ST1,
		.selector_out1       = XMC_CCU8_SOURCE_OUT1_ST1,
		.selector_out2       = XMC_CCU8_SOURCE_OUT2_ST2,
		.selector_out3       = XMC_CCU8_SOURCE_OUT3_ST2,
		.prescaler_initval   = 0,
		.float_limit         = 0,
		.dither_limit        = 0,
		.timer_concatenation = false
	};
	XMC_CCU8_SLICE_CompareInit(CCU80_CC80, &ccu8_slice_config);

	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC80, 48000-1);
	XMC_CCU8_SLICE_SetTimerCompareMatch(CCU80_CC80, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, 0);
	XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_0 | XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_0);

	XMC_CCU8_SLICE_SetInterruptNode(CCU80_CC80, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_0);
	XMC_CCU8_SLICE_EnableEvent(CCU80_CC80, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);

	XMC_CCU8_SLICE_ClearTimer(CCU80_CC80);

	NVIC_SetPriority(IRQ1_IRQn, 0);
	XMC_SCU_SetInterruptControl(IRQ1_IRQn, XMC_SCU_IRQCTRL_CCU80_SR0_IRQ1);
	NVIC_EnableIRQ(IRQ1_IRQn);
}

void timer_irq_tick(void) {
	// Nothing
}