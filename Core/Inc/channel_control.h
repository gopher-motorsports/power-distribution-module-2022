/*
 * channel_control.h
 *
 *  Created on: Jan 20, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_CHANNEL_CONTROL_H_
#define INC_CHANNEL_CONTROL_H_

#include "base_types.h"
#include "channel.h"

void init_channels(Channel* channels, DiagnosticState* diagnostic_state, volatile U32* time_micros);
void update_channels(Channel* channels, U8 num_channels);
void set_all_channel_diagnostics(Channel* channels, DiagnosticState state);
U16 get_channel_supply_voltage(Channel* channels, U8 id);
U16 get_channel_temperature(Channel* channels, U8 id);
U16 get_channel_current(Channel* channels, U8 id);
boolean get_channel_hw_fault(Channel* channels, U8 id);
boolean get_channel_hw_open_load(Channel* channels, U8 id);
void clear_all_channel_hw_faults();

#endif /* INC_CHANNEL_CONTROL_H_ */
