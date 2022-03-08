/*
 * channel_control.h
 *
 *  Created on: Jan 20, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_CHANNEL_CONTROL_H_
#define INC_CHANNEL_CONTROL_H_

#include "base_types.h"

#define SEL1_MASK   0x01
#define SEL2_MASK   0x02
#define DIA_EN_MASK 0x04

typedef enum {
	DIAGNOSTICS_DISABLED = 0b000,
	DIAGNOSTICS_CURRENT  = 0b100,
	DIAGNOSTICS_TEMP     = 0b101,
	DIAGNOSTICS_VOLTAGE  = 0b111,
	DIAGNOSTICS_OL_STB   = 0b100,
} DiagnosticState;

void set_all_channel_diagnostics(DiagnosticState state);
U8 get_channel_supply_voltage(U8 channel);
U8 get_channel_temperature(U8 channel);
U8 get_channel_current(U8 channel);
boolean get_channel_hw_fault(U8 channel);
boolean get_channel_hw_open_load(U8 channel);
void clear_all_channel_hw_faults();

#endif /* INC_CHANNEL_CONTROL_H_ */
