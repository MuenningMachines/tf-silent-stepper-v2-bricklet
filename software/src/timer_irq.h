/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * timer_irq.h: CCU8 timer with IRQ
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

#ifndef TIMER_IRQ_H
#define TIMER_IRQ_H

#include "xmc_ccu8.h"

#include <stdint.h>

#define timer_irq IRQ_Hdlr_1

void timer_irq_init(void);
void timer_irq_tick(void);

static inline void timer_irq_stop(void) {
	XMC_CCU8_SLICE_StopTimer(CCU80_CC80);
}

static inline void timer_irq_start(void) {
	XMC_CCU8_SLICE_StartTimer(CCU80_CC80);
}

static inline void timer_irq_new_time(const uint8_t prescaler, const const uint16_t period) {
    CCU80_CC80->PSC = prescaler;
	XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU80_CC80, period-1);
	XMC_CCU8_EnableShadowTransfer(CCU80, XMC_CCU8_SHADOW_TRANSFER_SLICE_0 | XMC_CCU8_SHADOW_TRANSFER_PRESCALER_SLICE_0);
}

#endif