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


typedef struct {
	DiagnosticState state;
	DiagnosticState queue[2];
	U16 voltage_period;
	U16 temp_period;
	U16 voltage_timer;
	U16 temp_timer;
} DiagnosticStateController;

void init_channels(Channel* channels, DiagnosticState* diagnostic_state, volatile U32* time_micros);
void init_dsc(DiagnosticStateController* dsc, U16 voltage_period, U16 temp_period);
void update_channels(Channel* channels, U8 num_channels);
void set_diagnostics_raw(Channel* channels, DiagnosticState state);
void set_diagnostics(DiagnosticStateController* dsc, DiagnosticState state);
void update_diagnostics_queue(DiagnosticStateController* dsc);
void update_diagnostics(Channel* channels, DiagnosticStateController* dsc);
U16 check_battery_voltage(Channel* channels);
U16 check_channel_supply_voltage(Channel* channels, U8 id);
U16 check_channel_temperature(Channel* channels, U8 id);
U16 check_channel_current(Channel* channels, U8 id);
boolean check_channel_fault(Channel* channels, U8 id);
boolean check_channel_open_load(Channel* channels, U8 id);

#endif /* INC_CHANNEL_CONTROL_H_ */
