/*
 * channel_control.c
 *
 *  Created on: Jan 16, 2022
 *      Author: Ben Abbott
 */

#include "channel_control.h"
#include "channel.h"
#include "config.h"
#include "base_types.h"
#include <stdlib.h>

U8 num_channels;
Channel** channels;

void init_channels() {
	channels = (Channel**)pvPortMalloc(sizeof(Channel*)*NUM_CHANNELS);
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		channels[i] = init_channel();
	}
}

/**
 * Sets the diagnostic pins on switches.  After this function is called,
 * any ADC measurements made before the minimum wait time specified in the
 * switch datasheet have to be discarded.
 */
void set_all_channel_diagnostics(DiagnosticState state) {
	HAL_GPIO_WritePin(SEL1_PORT, SEL1_PIN, state | SEL1_MASK);
	HAL_GPIO_WritePin(SEL2_PORT, SEL2_PIN, state | SEL2_MASK);
	HAL_GPIO_WritePin(DIA_EN_PORT, DIA_EN_PIN, state | EN_MASK);
}


U8 get_channel_supply_voltage(U8 channel) {
	return 0;
}

U8 get_channel_temperature(U8 channel) {
	return 0;
}

U8 get_channel_current(U8 channel) {
	return 0;
}

boolean get_channel_hw_fault(U8 channel) {
	return 0;
}

boolean get_channel_hw_open_load(U8 channel) {
	return 0;
}

void clear_all_channel_hw_faults() {

}
