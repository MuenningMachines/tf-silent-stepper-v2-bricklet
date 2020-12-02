/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * tmc2130.c: Driver for TMC2130
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

#include "tmc2130.h"

#include "configs/config_tmc2130.h"
#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

#include "bricklib2/utility/util_definitions.h"
#include "bricklib2/hal/ccu4_pwm/ccu4_pwm.h"
#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/hal/spi_fifo/spi_fifo.h"

#include "configs/config_tmc2130.h"
#include "communication.h"
#include "stepper.h"
#include "voltage.h"

#include "xmc_gpio.h"
#include "xmc_ccu4.h"
#include "xmc_ccu8.h"

TMC2130 tmc2130;
CoopTask tmc2130_task;

const XMC_GPIO_CONFIG_t input_pullup_config = {
	.mode             = XMC_GPIO_MODE_INPUT_PULL_UP,
	.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
};

const XMC_GPIO_CONFIG_t input_default_config = {
	.mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
	.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
};

const XMC_GPIO_CONFIG_t output_high_config = {
	.mode         = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
	.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH
};

const XMC_GPIO_CONFIG_t output_low_config = {
	.mode         = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
	.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW
};

// Everything that is different on the wire compared to the register
// we put in here as a backup for the API getters
TMC2130HighLevel tmc2130_high_level = {
	.standstill_current = 200,
	.motor_run_current = 800,
	.standstill_delay_time = 0,
	.power_down_time = 1000,
	.stealth_threshold = 500,
	.coolstep_threshold = 500,
	.classic_threshold = 1000
};

// Unused registers

// Read Only
TMC2130RegIOEN tmc2130_reg_ioen; // Reads state of pins, we can get this without reading a register
TMC2130RegMSCNT tmc2130_reg_mscnt; // Not needed if we don't use MSLUT
TMC2130RegMSCURACT tmc2130_reg_mscuract; // Not needed if we don't use MSLUT
TMC2130RegLOST_STEPS tmc2130_reg_lost_steps; // Not used in API (needs dcStep)

// Write Only
TMC2130RegVDCMIN tmc2130_reg_vdcmin = { // Not used in API
	.bit = {
		.velocity = 0
	}
};

TMC2130RegMSLUT tmc2130_reg_mslut[TMC2130_REG_MSLUT_NUM] = { // Not used in API
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } },
	{ .bit = { .table_entry = 0 } }
};

TMC2130RegMSLUTSEL tmc2130_reg_mslutsel = { // Not used in API
	.bit = {
		.w0 = 0,
		.w1 = 0,
		.w2 = 0,
		.w3 = 0,
		.x1 = 0,
		.x2 = 0,
		.x3 = 0,
	}
};

TMC2130RegMSLUTSTART tmc2130_reg_mslutstart = { // Not used in API
	.bit = {
		.start_sin = 0,
		.start_sin90 = 247
	}
};

TMC2130RegDCCTRL tmc2130_reg_dcctrl = { // Can only be used together with dcStep
	.bit = {
		.dc_time = 0,
		.dc_sg = 0
	}
};

TMC2130RegENCM_CTRL tmc2130_reg_encm_ctrl = { // Unnecessary
	.bit = {
		.inv = 0,
		.maxspeed = 0
	}
};

// Read/Write
TMC2130RegXDIRECT tmc2130_reg_xdirect = { // Not used in API
	.bit = {
		.coil_a = 0,
		.coil_b = 0
	}
};



// Used registers

// Read Only
TMC2130RegGSTAT tmc2130_reg_gstat; // Read + Clear upon read
TMC2130RegTSTEP tmc2130_reg_tstep; // Only needed if decide to use internal clock
TMC2130RegDRV_STATUS tmc2130_reg_drv_status;
TMC2130RegPWM_SCALE tmc2130_reg_pwm_scale;  // Can be used to detect motor stalling


// Write Only
TMC2130RegIHOLD_IRUN tmc2130_reg_ihold_run = { // velocity_based_mode_configuration
	.bit = {
		.ihold = 8,
		.irun = 31,
		.ihold_delay = 0
	}
};

TMC2130RegTPOWERDOWN tmc2130_reg_tpowerdown = { // velocity_based_mode_configuration
	.bit = {
		.delay = 48 // ~1 second
	}
};

TMC2130RegTPWMTHRS tmc2130_reg_tpwmthrs = { // velocity_based_mode_configuration
	.bit = {
		.velocity = TMC2130_CLOCK_FREQUENCY/(500*256)
	}
};

TMC2130RegTCOOLTHRS tmc2130_reg_tcoolthrs = { // velocity_based_mode_configuration
	.bit = {
		.velocity = TMC2130_CLOCK_FREQUENCY/(500*256)
	}
};

TMC2130RegTHIGH tmc2130_reg_thigh = { // velocity_based_mode_configuration
	.bit = {
		.velocity = TMC2130_CLOCK_FREQUENCY/(1000*256)
	}
};

TMC2130RegCOOLCONF tmc2130_reg_coolconf = {
	.bit = {
		.semin = 2, // coolstep_configuration
		.seup = 0, // coolstep_configuration
		.semax = 10, // coolstep_configuration
		.sedn = 0,  // coolstep_configuration
		.seimin = 0, // coolstep_configuration
		.sgt = 0, // stallguard_configuration
		.sfilt = 0 // stallguard_configuration
	}
};

TMC2130RegPWMCONF tmc2130_reg_pwmconf = { // stealth_configuration
	.bit = {
		.pwm_ampl = 0x80,
		.pwm_grad = 4,
		.pwm_freq = 1, // We have a fixed external clock of 12.8MHz and with pwm_freq=1 we are about at the optimum of 40kHz PWM frequency
		.pwm_autoscale = 1, // automatic current control enabled
		.pwm_symmetric = 0,
		.freewheel = 0
	}
};

// Read/Write
TMC2130RegGCONF tmc2130_reg_gconf = {
	.bit = {
		.i_scale_analog         = 1, // Fixed
		.internal_rsense        = 0, // Fixed
		.en_pwm_mode            = 1, // stealth_configuration (Change only in stand still)
		.enc_commutation        = 0, // Fixed
		.shaft                  = 0, // Fixed ???
		.diag0_error            = 0, // Fixed
		.diag0_otpw             = 0, // Fixed
		.diag0_stall            = 0, // Fixed
		.diag1_stall            = 0, // Fixed
		.diag1_index            = 0, // Fixed
		.diag1_diag1_onstate    = 0, // Fixed
		.diag1_steps_skipped    = 0, // Fixed
		.diag0_int_pushpull     = 0, // Fixed
		.diag1_pushpull         = 0, // Fixed
		.small_hysteresis       = 0, // velocity_based_mode_configuration
		.stop_enable            = 0, // Fixed
		.direct_mode            = 0, // Fixed
		.test_mode              = 0  // Fixed, never change
	}
};

TMC2130RegCHOPCONF tmc2130_reg_chopconf = {
	.bit = {
		.toff = 4, // spreadcycle_configuration
		.hstrt = 0, // spreadcycle_configuration
		.hend = 0, // spreadcycle_configuration
		.fd3 = 0, // ???
		.disfdcc = 0, // spreadcycle_configuration
		.rndtf = 0, // misc_configuration
		.chm = 0, // spreadcycle_configuration
		.tbl = 1, // spreadcycle_configuration
		.vsense = 0, // Always 0
		.vhighfs = 0, // Always 0 (is only used for dcStep)
		.vhighchm = 0, // velocity_based_mode_configuration
		.sync = 0, // chopsync_configuration
		.mres = 0, // step_configuration
		.intpol = 1, // step_configuration
		.dedge = 1, // Always 1.
		.diss2g = 0 // misc_configuration
	}
};

uint32_t tmc2130_task_register_read(const uint8_t reg) {
	while(tmc2130.spi_communication_in_progress) {
		coop_task_yield();
	}
	tmc2130.spi_communication_in_progress = true;

	uint8_t tmp[5] = {
		TMC2130_SPI_READ | reg, 
		0, 0, 0, 0
	};

	// First read sets pointer
	uint8_t tmp_read1[5] = {0};
	if(!spi_fifo_coop_transceive(&tmc2130.spi_fifo, 5, tmp, tmp_read1)) {
		// TODO: Reset SPI here in case of error?
		tmc2130.last_status = 0xFF;
		tmc2130.spi_communication_in_progress = false;
		return 0xFFFFFFFF;
	}

	// Second read gets actual data
	uint8_t tmp_read2[5] = {0};
	if(!spi_fifo_coop_transceive(&tmc2130.spi_fifo, 5, tmp, tmp_read2)) {
		// TODO: Reset SPI here in case of error?
		tmc2130.last_status = 0xFF;
		tmc2130.spi_communication_in_progress = false;
		return 0xFFFFFFFF;
	}

	tmc2130.last_status = tmp_read2[0];

	tmc2130.spi_communication_in_progress = false;
	return (tmp_read2[1] << 24) | (tmp_read2[2] << 16) | (tmp_read2[3] << 8) | (tmp_read2[4] << 0);
}

void tmc2130_task_register_write(const uint8_t reg, const uint32_t data) {
	while(tmc2130.spi_communication_in_progress) {
		coop_task_yield();
	}
	tmc2130.spi_communication_in_progress = true;

	uint8_t tmp[5] = {
		TMC2130_SPI_WRITE | reg,
		(data >> 24) & 0xFF,
		(data >> 16) & 0xFF,
		(data >>  8) & 0xFF,
		(data >>  0) & 0xFF,
	};

	if(!spi_fifo_coop_transceive(&tmc2130.spi_fifo, 5, tmp, tmp)) {
		// TODO: Reset SPI here in case of error?
		tmc2130.last_status = 0xFF;
		tmc2130.spi_communication_in_progress = false;
		return;
	}

	tmc2130.last_status = tmp[0];
	tmc2130.spi_communication_in_progress = false;
}


void tmc2130_task_register_read_by_bit(uint32_t register_bit) {
	tmc2130.current_read_bit = register_bit;
	uint32_t *read_address = NULL;

	uint8_t reg = 0;
	switch(register_bit) {
		case TMC2130_REG_GSTAT_BIT:      reg = TMC2130_REG_GSTAT;      read_address = &tmc2130_reg_gstat.reg;      break;
		case TMC2130_REG_TSTEP_BIT:      reg = TMC2130_REG_TSTEP;      read_address = &tmc2130_reg_tstep.reg;      break;
		case TMC2130_REG_DRV_STATUS_BIT: reg = TMC2130_REG_DRV_STATUS; read_address = &tmc2130_reg_drv_status.reg; break;
		case TMC2130_REG_PWM_SCALE_BIT:  reg = TMC2130_REG_PWM_SCALE;  read_address = &tmc2130_reg_pwm_scale.reg;  break;
		default: return;
	}

	uint32_t value = tmc2130_task_register_read(reg);
	*read_address = value;
}

void tmc2130_task_register_write_by_bit(uint32_t register_bit) {
	switch(register_bit) {
		case TMC2130_REG_GCONF_BIT:      tmc2130_task_register_write(TMC2130_REG_GCONF,      tmc2130_reg_gconf.reg);      break;
		case TMC2130_REG_IHOLD_IRUN_BIT: tmc2130_task_register_write(TMC2130_REG_IHOLD_IRUN, tmc2130_reg_ihold_run.reg);  break;
		case TMC2130_REG_TPOWERDOWN_BIT: tmc2130_task_register_write(TMC2130_REG_TPOWERDOWN, tmc2130_reg_tpowerdown.reg); break;
		case TMC2130_REG_TPWMTHRS_BIT:   tmc2130_task_register_write(TMC2130_REG_TPWMTHRS,   tmc2130_reg_tpwmthrs.reg);   break;
		case TMC2130_REG_TCOOLTHRS_BIT:  tmc2130_task_register_write(TMC2130_REG_TCOOLTHRS,  tmc2130_reg_tcoolthrs.reg);  break;
		case TMC2130_REG_THIGH_BIT:      tmc2130_task_register_write(TMC2130_REG_THIGH,      tmc2130_reg_thigh.reg);      break;
		case TMC2130_REG_XDIRECT_BIT:    tmc2130_task_register_write(TMC2130_REG_XDIRECT,    tmc2130_reg_xdirect.reg);    break;
		case TMC2130_REG_VDCMIN_BIT:     tmc2130_task_register_write(TMC2130_REG_VDCMIN,     tmc2130_reg_vdcmin.reg);     break;
		case TMC2130_REG_MSLUT0_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT0,     tmc2130_reg_mslut[0].reg);   break;
		case TMC2130_REG_MSLUT1_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT1,     tmc2130_reg_mslut[1].reg);   break;
		case TMC2130_REG_MSLUT2_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT2,     tmc2130_reg_mslut[2].reg);   break;
		case TMC2130_REG_MSLUT3_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT3,     tmc2130_reg_mslut[3].reg);   break;
		case TMC2130_REG_MSLUT4_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT4,     tmc2130_reg_mslut[4].reg);   break;
		case TMC2130_REG_MSLUT5_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT5,     tmc2130_reg_mslut[5].reg);   break;
		case TMC2130_REG_MSLUT6_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT6,     tmc2130_reg_mslut[6].reg);   break;
		case TMC2130_REG_MSLUT7_BIT:     tmc2130_task_register_write(TMC2130_REG_MSLUT7,     tmc2130_reg_mslut[7].reg);   break;
		case TMC2130_REG_MSLUTSEL_BIT:   tmc2130_task_register_write(TMC2130_REG_MSLUTSEL,   tmc2130_reg_mslutsel.reg);   break;
		case TMC2130_REG_MSLUTSTART_BIT: tmc2130_task_register_write(TMC2130_REG_MSLUTSTART, tmc2130_reg_mslutstart.reg); break;
		case TMC2130_REG_CHOPCONF_BIT:   tmc2130_task_register_write(TMC2130_REG_CHOPCONF,   tmc2130_reg_chopconf.reg);   break;
		case TMC2130_REG_COOLCONF_BIT:   tmc2130_task_register_write(TMC2130_REG_COOLCONF,   tmc2130_reg_coolconf.reg);   break;
		case TMC2130_REG_DCCTRL_BIT:     tmc2130_task_register_write(TMC2130_REG_DCCTRL,     tmc2130_reg_dcctrl.reg);     break;
		case TMC2130_REG_PWMCONF_BIT:    tmc2130_task_register_write(TMC2130_REG_PWMCONF,    tmc2130_reg_pwmconf.reg);    break;
		case TMC2130_REG_ENCM_CTRL_BIT:  tmc2130_task_register_write(TMC2130_REG_ENCM_CTRL,  tmc2130_reg_encm_ctrl.reg);  break;
	}
}


void tmc2130_update_read_registers(void) {
	if(tmc2130.is_active) {
		tmc2130.register_to_read_mask = TMC2130_REG_DRV_STATUS_BIT | TMC2130_REG_PWM_SCALE_BIT;
	}
}

static void tmc2130_init_spi(void) {
	tmc2130.spi_fifo.channel             = TMC2130_USIC_SPI;
	tmc2130.spi_fifo.baudrate            = TMC2130_SPI_BAUDRATE;

	tmc2130.spi_fifo.rx_fifo_size        = TMC2130_RX_FIFO_SIZE;
	tmc2130.spi_fifo.rx_fifo_pointer     = TMC2130_RX_FIFO_POINTER;
	tmc2130.spi_fifo.tx_fifo_size        = TMC2130_TX_FIFO_SIZE;
	tmc2130.spi_fifo.tx_fifo_pointer     = TMC2130_TX_FIFO_POINTER;

	tmc2130.spi_fifo.slave               = TMC2130_SLAVE;
	tmc2130.spi_fifo.clock_output        = TMC2130_CLOCK_OUTPUT;
	tmc2130.spi_fifo.clock_passive_level = TMC2130_CLOCK_PASSIVE_LEVEL;

	tmc2130.spi_fifo.sclk_pin            = TMC2130_SCLK_PIN;
	tmc2130.spi_fifo.sclk_port           = TMC2130_SCLK_PORT;
	tmc2130.spi_fifo.sclk_pin_mode       = TMC2130_SCLK_PIN_MODE;

	tmc2130.spi_fifo.select_pin          = TMC2130_SELECT_PIN;
	tmc2130.spi_fifo.select_port         = TMC2130_SELECT_PORT;
	tmc2130.spi_fifo.select_pin_mode     = TMC2130_SELECT_PIN_MODE;

	tmc2130.spi_fifo.mosi_pin            = TMC2130_MOSI_PIN;
	tmc2130.spi_fifo.mosi_port           = TMC2130_MOSI_PORT;
	tmc2130.spi_fifo.mosi_pin_mode       = TMC2130_MOSI_PIN_MODE;

	tmc2130.spi_fifo.miso_pin            = TMC2130_MISO_PIN;
	tmc2130.spi_fifo.miso_port           = TMC2130_MISO_PORT;
	tmc2130.spi_fifo.miso_input          = TMC2130_MISO_INPUT;
	tmc2130.spi_fifo.miso_source         = TMC2130_MISO_SOURCE;

	spi_fifo_init(&tmc2130.spi_fifo);

	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_IHOLD_IRUN, tmc2130_reg_ihold_run.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_TPOWERDOWN, tmc2130_reg_tpowerdown.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_TPWMTHRS,   tmc2130_reg_tpwmthrs.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_TCOOLTHRS,  tmc2130_reg_tcoolthrs.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_THIGH,      tmc2130_reg_thigh.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_COOLCONF,   tmc2130_reg_coolconf.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_PWMCONF,    tmc2130_reg_pwmconf.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_ENCM_CTRL,  tmc2130_reg_encm_ctrl.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_GCONF,      tmc2130_reg_gconf.reg);
	system_timer_sleep_us(10);
	tmc2130_task_register_write(TMC2130_REG_CHOPCONF,   tmc2130_reg_chopconf.reg);
	system_timer_sleep_us(10);

	// Make sure that motor is enabled after active changed from false to true
	if(!tmc2130.pin_enable_output_save) {
		stepper_enable();
	}
}

void tmc2130_init_vref(void) {
	const XMC_CCU4_SLICE_COMPARE_CONFIG_t compare_config = {
		.timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
		.monoshot            = false,
		.shadow_xfer_clear   = 0,
		.dither_timer_period = 0,
		.dither_duty_cycle   = 0,
		.prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
		.mcm_enable          = 0,
		.prescaler_initval   = 0,
		.float_limit         = 0,
		.dither_limit        = 0,
		.passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
		.timer_concatenation = 0
	};

	const XMC_GPIO_CONFIG_t gpio_out_config	= {
		.mode                = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9,
		.input_hysteresis    = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
		.output_level        = XMC_GPIO_OUTPUT_LEVEL_LOW,
	};

    XMC_CCU4_Init(CCU41, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_StartPrescaler(CCU41);
    XMC_CCU4_SLICE_CompareInit(CCU41_CC41, &compare_config);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU41_CC41, 960-1);


    XMC_GPIO_Init(TMC2130_VREF_PIN, &gpio_out_config);

    XMC_CCU4_EnableClock(CCU41, 1);
    XMC_CCU4_SLICE_StartTimer(CCU41_CC41);
}

void tmc2130_set_output_current(const uint16_t current) {
	const uint16_t new_current = BETWEEN(STEPPER_VREF_MIN_CURRENT,
	                                     current,
										 STEPPER_VREF_MAX_CURRENT);


	const uint16_t scaled_value = SCALE(new_current, 0, 1640, 960, 0);
    XMC_CCU4_SLICE_SetTimerCompareMatch(CCU41_CC41, scaled_value);
    XMC_CCU4_EnableShadowTransfer(CCU41, XMC_CCU4_SHADOW_TRANSFER_SLICE_1 | XMC_CCU4_SHADOW_TRANSFER_PRESCALER_SLICE_1);

	stepper.output_current = new_current;
}

void tmc2130_activate(void) {
	XMC_GPIO_Init(TMC2130_SW_3V3_PIN, &output_high_config);

	// The clock runs at 12MHz here, it was 12.8MHz
	// on the Silent Stepper Brick.
	// TODO: Make sure that nothing in the code depends on the old 12.8MHz value still!
	ccu4_pwm_init(TMC2130_CLK_PIN, TMC2130_CLK_SLICE, 8 - 1);
	ccu4_pwm_set_duty_cycle(TMC2130_CLK_SLICE, 4);

	tmc2130_init_vref();
	tmc2130_init_spi();

	XMC_GPIO_Init(TMC2130_ENABLE_PIN, &output_high_config);

	if(tmc2130.pin_step_output_save) {
		XMC_GPIO_Init(TMC2130_STEP_PIN, &output_high_config);
	} else {
		XMC_GPIO_Init(TMC2130_STEP_PIN, &output_low_config);
	}

	if(tmc2130.pin_direction_output_save) {
		XMC_GPIO_Init(TMC2130_DIR_PIN, &output_high_config);
	} else {
		XMC_GPIO_Init(TMC2130_DIR_PIN, &output_low_config);
	}

	XMC_GPIO_Init(TMC2130_SW_3V3_PIN, &output_high_config);

	tmc2130_set_output_current(stepper.output_current);

	tmc2130.is_active = true;
}

void tmc2130_deactivate(void) {
	tmc2130.is_active = false;
	XMC_GPIO_SetOutputLow(TMC2130_SW_3V3_PIN);

	tmc2130.pin_enable_output_save    = XMC_GPIO_GetInput(TMC2130_ENABLE_PIN);
	tmc2130.pin_step_output_save      = XMC_GPIO_GetInput(TMC2130_STEP_PIN);
	tmc2130.pin_direction_output_save = XMC_GPIO_GetInput(TMC2130_DIR_PIN);

	XMC_GPIO_Init(TMC2130_ENABLE_PIN, &input_default_config);
	XMC_GPIO_Init(TMC2130_STEP_PIN,   &input_default_config);
	XMC_GPIO_Init(TMC2130_DIR_PIN,    &input_default_config);
	XMC_GPIO_Init(TMC2130_VREF_PIN,   &input_default_config);
	XMC_GPIO_Init(TMC2130_CLK_PIN,    &input_default_config);
	XMC_GPIO_Init(TMC2130_SW_3V3_PIN, &input_default_config);
	XMC_GPIO_Init(TMC2130_MISO_PORT,   TMC2130_MISO_PIN,   &input_default_config);
	XMC_GPIO_Init(TMC2130_MOSI_PORT,   TMC2130_MOSI_PIN,   &input_default_config);
	XMC_GPIO_Init(TMC2130_SCLK_PORT,   TMC2130_SCLK_PIN,   &input_default_config);
	XMC_GPIO_Init(TMC2130_SELECT_PORT, TMC2130_SELECT_PIN, &input_default_config);

	stepper_disable();
}

void tmc2130_set_active(const bool active) {
	if(active == tmc2130.is_active) {
		return;
	}

	if(active) {
		tmc2130_activate();
	} else {
		tmc2130_deactivate();
	}
}

static void tmc2130_task_handle_error_led(void) {
	static uint32_t last_time = 0;

	if(tmc2130.error_led_flicker_state.config == LED_FLICKER_CONFIG_HEARTBEAT) {
		led_flicker_tick(&tmc2130.error_led_flicker_state, system_timer_get_ms(), TMC2130_ERROR_LED_PIN);
	} else if(tmc2130.error_led_flicker_state.config == SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR) {
		uint32_t error = 0;
		if(voltage.value < stepper.minimum_voltage) {
			error = 500;
		}

		if(tmc2130_reg_drv_status.bit.otpw) {
			error = 125;
		}

		if(tmc2130_reg_drv_status.bit.s2gb || tmc2130_reg_drv_status.bit.s2ga || tmc2130_reg_drv_status.bit.ot) {
			error = 1;
		}

		if(error == 0) {
			XMC_GPIO_SetOutputHigh(TMC2130_ERROR_LED_PIN);
		} else if (error == 1) {
			XMC_GPIO_SetOutputHigh(TMC2130_ERROR_LED_PIN);
		} else {
			if(system_timer_is_time_elapsed_ms(last_time, error)) {
				XMC_GPIO_ToggleOutput(TMC2130_ERROR_LED_PIN);
				last_time = system_timer_get_ms();
			}
		}
	}
}

void tmc2130_task_tick(void) {
	while(true) {
		tmc2130_task_handle_error_led();

		// Power TMC2130 if input voltage is connected and above voltage minimum
		if(stepper.minimum_voltage < voltage.value) {
			stepper.minimum_voltage_cb_done = false;
			tmc2130_set_active(true);
		} else {
			tmc2130_set_active(false);
		}

		if(!tmc2130.is_active) {
			coop_task_yield();
			continue;
		}

		// Update read registers every 10ms
		if(system_timer_is_time_elapsed_ms(tmc2130.last_read_register_update, 10)) {
			tmc2130_update_read_registers();
			tmc2130.last_read_register_update = system_timer_get_ms();
		}

		if(tmc2130.register_to_write_mask != 0) {
			for(uint8_t i = 0; i < TMC2130_NUM_REGS_TO_WRITE; i++) {
				const uint32_t bit = 1 << i;
				if(tmc2130.register_to_write_mask & bit) {
					tmc2130_task_register_write_by_bit(bit);
					tmc2130.register_to_write_mask &= ~bit;
					coop_task_yield();
				}
			}
		}
		if(tmc2130.register_to_read_mask != 0) {
			for(uint8_t i = 0; i < TMC2130_NUM_REGS_TO_READ; i++) {
				const uint32_t bit = 1 << i;
				if(tmc2130.register_to_read_mask & bit) {
					tmc2130_task_register_read_by_bit(bit);
					tmc2130.register_to_read_mask &= ~bit;
					coop_task_yield();
				}
			}
		}

		coop_task_yield();
	}
}

void tmc2130_tick(void) {
	coop_task_tick(&tmc2130_task);
}

void tmc2130_init(void) {
	memset(&tmc2130, 0, sizeof(TMC2130));

	// Set minimum voltage cb to done, we don't want to send UnderVoltage callbacks on startup
	stepper.minimum_voltage_cb_done = true;
	tmc2130_deactivate();

	XMC_GPIO_Init(TMC2130_ERROR_LED_PIN, &output_high_config);
	tmc2130.error_led_flicker_state.config = SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR;

	coop_task_init(&tmc2130_task, tmc2130_task_tick);
}