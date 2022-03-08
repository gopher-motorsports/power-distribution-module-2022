/*
 * channel.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_CHANNEL_H_
#define INC_CHANNEL_H_

#include "config.h"
#include "ring_buffer.h"
#include "base_types.h"
#include "channel_control.h"

typedef enum {
	FUSE_ACTIVE, FUSE_AUTO_RETRY, FUSE_DISABLED
} FuseState;

typedef enum {
	NO_HW_FAULT, OVERCURRENT_FAULT, OPEN_LOAD_FAULT
} HWFault;

typedef enum {
	SWITCH_ENABLED, SWITCH_DISABLED
} SwitchState;

typedef struct {
	// The channel number, or "id"
	U8 id;
	// If the channel is desired to be on/off (in software), independent of
	// the actual physical state of the switch
	SwitchState set_switch;
	// If the switch is enabled or not
	SwitchState switch_state;
	// The hardware fault state of the switch attached to this channel
	HWFault hw_fault;
	// Current state of the fuse
	FuseState fuse_state;
	// The number of times this channel has been disabled and re-switch_state
	U8 retries;
	// The amount of time that has passed since the channel has been disbaled
	U16 retry_timer_ms;
	// Check if this channel is disabled due to brownout protection
	boolean brn_out_disabled;
	// The number of times this module has been turned on and off recently
	// due to brownout protection
	U8 recent_retries;
	// The primary buffer used for storing ADC measurements
	U16RingBuffer* adc_buffer;
	// The primary buffer for storing recent averaged ADC measurements
	U16RingBuffer* filtered_adc_buffer;
	// The current integrator for software overcurrent
	double current_integrator;
	// The most recent current measurement in milliamperes
	U16 switch_current;
	// The present temperature of the high-side switch
	double switch_temp;
	// The present voltage on the supply side of the switch
	double switch_voltage;
} Channel;

// TODO: Functions
// Function to initalize channel
// Function to run on DMA interrupt
// Function to run every tick that updates both downstream buffers, and
// updates all states, checks for hardware faults, updates fuse state, updates
// max continuous current, etc.
// Function for calculating max contintuous current?
// Function for enabling/disabling
// Function for endabling/disabling on brownout

Channel* init_channel(U8 id);
void channel_adc_interrupt(Channel* channel, boolean first_half, boolean valid);
FuseState update_channel(Channel* channel, DiagnosticState diagnostic_state);
void set_channel(Channel* channel, SwitchState on);
void free_channel(Channel* channel);

#endif /* INC_CHANNEL_H_ */
