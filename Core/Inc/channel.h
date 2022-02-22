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

typedef enum {
	ACTIVE, AUTO_RETRY, DISABLED
} FuseState;

typedef enum {
	NONE, OVERCURRENT, OPEN_LOAD
} HWFault;

typedef struct {
	// If the channel is desired to be on/off (in software), independent of
	// the actual physical state of the switch
	boolean channel_on;
	// Current state of the fuse
	FuseState state;
	// If a hardware fault has been detected (hardware protection tripped)
	boolean hw_fault;
	// Check if this channel is disabled due to brownout protection
	boolean brn_out_disabled;
	// The number of times this channel has been disabled and re-enabled
	U8 retries;
	// The number of times this module has been turned on and off recently
	// due to brownout protection
	U8 recent_retries;
	// Maximum continuous current recorded on this channel (in mA)
	U16 max_cont_current;
	// The primary buffer used for storing ADC measurements
	U16RingBuffer* primary_adc_buffer;
	// The primary buffer for storing recent current measurements, and loading
	// them into the fuse_integrator buffer
	U16RingBuffer* primary_current_buffer;
	// The secondary buffer, used for software overcurrent protection
	U16RingBuffer* secondary_current_buffer;
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

Channel* init_channel();
FuseState update_channel(Channel* channel, U16 current);
void set_channel(boolean on);
void free_channel(Channel* channel);

#endif /* INC_CHANNEL_H_ */
