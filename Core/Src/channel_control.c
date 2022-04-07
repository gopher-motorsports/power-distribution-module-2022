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
#include "freertos.h"
#include "stalloc.h"

extern U16 FUSE_PEAK_CURRENT[];
extern U16 FUSE_PEAK_TIME[];
extern U16 FUSE_FAST_BLOW_RATING[];
extern U16 FUSE_RATING[];

void init_channels(Channel* channels, DiagnosticState* diagnostic_state, U32* time_micros) {
	// TODO: Make MAX_FUSE_RETRIES and FUSE_RETRY_DELAY arrays
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		init_channel(&channels[i], i, time_micros, diagnostic_state, MAX_FUSE_RETRIES,
				FUSE_PEAK_CURRENT[i] * FUSE_PEAK_TIME[i], FUSE_FAST_BLOW_RATING[i],
				FUSE_RATING[i], FUSE_RETRY_DELAY_MS);
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
	HAL_GPIO_WritePin(DIA_EN_PORT, DIA_EN_PIN, state | DIA_EN_MASK);
}

U16 get_battery_voltage() {
	U32 average = 0;
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		average += CHANNELS[i].readings.voltage;
	}
	average /= NUM_CHANNELS;
	return average;
}

U16 get_channel_supply_voltage(Channel* channels, U8 id) {
	return channels[id].readings.voltage;
}

U16 get_channel_temperature(Channel* channels, U8 id) {
	return channels[id].readings.temp;
}

U16 get_channel_current(Channel* channels, U8 id) {
	return channels[id].readings.current;
}

boolean get_channel_fault(Channel* channels, U8 id) {
	return channels[id].fuse.state != FUSE_ACTIVE;
}

/**
 * Checks hardware open load protection to see if there is a load connected
 * to the switch
 * Note: This only updates when the switch is turned off
 *
 */
boolean get_channel_open_load(Channel* channels, U8 id) {
	return channels[id].hw_switch.no_load;
}

// TODO: Stick fault??
void clear_all_channel_hw_faults() {

}
