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

extern boolean POPULATED_CHANNELS[];
extern U16 FUSE_PEAK_CURRENT[];
extern U16 FUSE_PEAK_TIME_MS[];
extern U16 FUSE_FAST_BLOW_RATING[];
extern U16 FUSE_RATING[];
extern GPIO_TypeDef* SWITCH_EN_PORTS[];
extern U16 SWITCH_EN_PINS[];
extern S16 FUSE_RETRIES[];
extern U16 FUSE_RETRY_DELAY_MS[];

void init_channels(Channel* channels, DiagnosticState* diagnostic_state, volatile U32* time_micros) {
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		init_channel(
				&channels[i],
				i,
				time_micros,
				diagnostic_state,
				POPULATED_CHANNELS[i],
				FUSE_RETRIES[i],
				FUSE_PEAK_CURRENT[i] * FUSE_PEAK_TIME_MS[i],
				FUSE_FAST_BLOW_RATING[i],
				FUSE_RATING[i],
				FUSE_RETRY_DELAY_MS[i],
				SWITCH_EN_PINS[i],
				SWITCH_EN_PORTS[i]);
	}
}

void init_dsc(DiagnosticStateController* dsc, U16 voltage_period, U16 temp_period) {
	dsc->state = DIAGNOSTIC_CURRENT;
	for(U8 i = 0; i < DIAGNOSTIC_QUEUE_LENGTH; i++) {
		dsc->queue[i] = DIAGNOSTIC_CURRENT;
	}
	dsc->voltage_period = voltage_period;
	dsc->temp_period = temp_period;
	dsc->voltage_last = 0;
	dsc->temp_last = 0;
}

void update_channels(Channel* channels, U8 num_channels) {
	for(U8 i = 0; i < num_channels; i++) {
		update_channel(&channels[i]);
	}
}

/**
 * Sets the diagnostic pins on switches.  After this function is called,
 * any ADC measurements made before the minimum wait time specified in the
 * switch datasheet have to be discarded.
 */
void set_diagnostics_raw(Channel* channels, DiagnosticState state) {
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		update_channel_timing(&channels[i], T_SNS_DIA_SW);
	}
	HAL_GPIO_WritePin(SEL1_PORT, SEL1_PIN, state & SEL1_MASK);
	HAL_GPIO_WritePin(SEL2_PORT, SEL2_PIN, state & SEL2_MASK);
	HAL_GPIO_WritePin(DIA_EN_PORT, DIA_EN_PIN, state & DIA_EN_MASK);
}



/**
 * Updates the diagnostic state controller to add a desired state to the queue.
 * The state will be switched to within several loop iterations after the function is called
 */
void set_diagnostics(DiagnosticStateController* dsc, DiagnosticState state) {
	// Push the desired state the to the queue
	for(U8 i = 0; i < DIAGNOSTIC_QUEUE_LENGTH; i++) {
		if(dsc->queue[i] == state) {
			return;
		}
		if(dsc->queue[i] == DIAGNOSTIC_CURRENT) {
			dsc->queue[i] = state;
			return;
		}
	}
}

/*
 * Updates the queue in the DiagnosticStateController.
 * This function should be called before update_diagnostics!
 */
void update_diagnostics_queue(DiagnosticStateController* dsc) {

	U32 tick = HAL_GetTick();

	// If either of the timers are over, add their state to the queue
	if(tick - dsc->voltage_last >= dsc->voltage_period) {
		set_diagnostics(dsc, DIAGNOSTIC_VOLTAGE);
		dsc->voltage_last = tick;
	}
	if(tick - dsc->temp_last >= dsc->temp_period) {
		set_diagnostics(dsc, DIAGNOSTIC_TEMP);
		dsc->temp_last = tick;
	}
}

void update_diagnostics(Channel* channels, DiagnosticStateController* dsc) {
	// If we were just measuring current, advance the queue
	if(dsc->state == DIAGNOSTIC_CURRENT) {
		dsc->state = dsc->queue[0];
		// Move everything up one in the queue
		for(U8 i = 0; i < DIAGNOSTIC_QUEUE_LENGTH - 1; i++) {
			dsc->queue[i] = dsc->queue[i+1];
		}
		dsc->queue[DIAGNOSTIC_QUEUE_LENGTH-1] = DIAGNOSTIC_CURRENT;
		set_diagnostics_raw(channels, dsc->state);
	} else {
		// If we weren't measuring current, switch the mode to current so that
		// we don't go more than one tick without measuring current
		dsc->state = DIAGNOSTIC_CURRENT;
		set_diagnostics_raw(channels, dsc->state);
	}
}

/**
 * Returns the average battery voltage from the
 */
U16 check_battery_voltage(Channel* channels) {
	U32 average = 0;
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		average += channels[i].readings.voltage;
	}
	average /= NUM_CHANNELS;
	return average;
}

U16 check_channel_supply_voltage(Channel* channels, U8 id) {
	return channels[id].readings.voltage;
}

U16 check_channel_temperature(Channel* channels, U8 id) {
	return channels[id].readings.temp;
}

U16 check_channel_current(Channel* channels, U8 id) {
	return channels[id].readings.current;
}

boolean check_channel_fault(Channel* channels, U8 id) {
	return channels[id].fuse.state != FUSE_ACTIVE;
}

/**
 * Checks hardware open load protection to see if there is a load connected
 * to the switch
 * Note: This only updates when the switch is turned off
 *
 */
boolean check_channel_open_load(Channel* channels, U8 id) {
	return channels[id].hw_switch.no_load;
}
