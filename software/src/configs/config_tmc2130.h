/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config_tmc2130.h: Configuration for TMC2130
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

#ifndef CONFIG_TMC2130_H
#define CONFIG_TMC2130_H

#define TMC2130_CLK_PIN                P1_0
#define TMC2130_CLK_SLICE              0
#define TMC2130_VREF_PIN               P1_5
#define TMC2130_STEP_PIN               P1_4
#define TMC2130_ENABLE_PIN             P2_1
#define TMC2130_SW_3V3_PIN             P2_10
#define TMC2130_DIR_PIN                P2_11
#define TMC2130_ERROR_LED_PIN          P0_9

#define TMC2130_SPI_BAUDRATE           2000000
#define TMC2130_USIC_CHANNEL           USIC1_CH1
#define TMC2130_USIC_SPI               XMC_SPI1_CH1

#define TMC2130_RX_FIFO_SIZE           XMC_USIC_CH_FIFO_SIZE_32WORDS
#define TMC2130_RX_FIFO_POINTER        0
#define TMC2130_TX_FIFO_SIZE           XMC_USIC_CH_FIFO_SIZE_32WORDS
#define TMC2130_TX_FIFO_POINTER        32

#define TMC2130_CLOCK_PASSIVE_LEVEL    XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_1_DELAY_DISABLED
#define TMC2130_CLOCK_OUTPUT           XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK
#define TMC2130_SLAVE                  XMC_SPI_CH_SLAVE_SELECT_0

#define TMC2130_SCLK_PORT              XMC_GPIO_PORT0
#define TMC2130_SCLK_PIN               3
#define TMC2130_SCLK_PIN_MODE          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8 | P0_3_AF_U1C1_SCLKOUT)

#define TMC2130_SELECT_PORT            XMC_GPIO_PORT0
#define TMC2130_SELECT_PIN             4
#define TMC2130_SELECT_PIN_MODE        (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8 | P0_4_AF_U1C1_SELO0)

#define TMC2130_MOSI_PORT              XMC_GPIO_PORT0
#define TMC2130_MOSI_PIN               0
#define TMC2130_MOSI_PIN_MODE          (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9 | P0_0_AF_U1C1_DOUT0)

#define TMC2130_MISO_PORT              XMC_GPIO_PORT0
#define TMC2130_MISO_PIN               1
#define TMC2130_MISO_INPUT             XMC_USIC_CH_INPUT_DX0
#define TMC2130_MISO_SOURCE            0b001 // DX0B

#endif
