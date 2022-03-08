/*
 * channel.c
 *
 *  Created on: Feb 12, 2022
 *      Author: Ben Abbott
 */

#include "config.h"
#include "channel.h"
#include "channel_control.h"
#include "freertos.h"
#include <math.h>

extern U16 FUSE_RATING_MILLIAMPERE[];
extern GPIO_TypeDef* SWITCH_EN_PORTS[];
extern U16 SWITCH_EN_PINS[];

Channel* init_channel(U8 id) {
	Channel* channel = (Channel*)pvPortMalloc(sizeof(Channel));
	channel->id = id;
	channel->set_switch = SWITCH_DISABLED;
	channel->switch_state = SWITCH_DISABLED;
	channel->fuse_state = FUSE_DISABLED;
	channel->retry_timer_ms = 0;
	channel->hw_fault = FALSE;
	channel->brn_out_disabled = FALSE;
	channel->retries = 0;
	channel->recent_retries = 0;
	channel->adc_buffer = init_ring_buffer(CHANNEL_ADC_BUFFER_SIZE);
	channel->filtered_adc_buffer = init_ring_buffer(CHANNEL_FILTERED_ADC_BUFFER_SIZE);
	channel->current_integrator = 0.0;
	return channel;
}



void channel_adc_interrupt(Channel* channel, boolean first_half, boolean valid) {
	// TODO: This will currently malloc a bunch of memory that will fill up the heap and never get freed
	if(valid) {
		if(first_half) {
			ring_buffer_append(channel->filtered_adc_buffer, ring_buffer_first_half_average(channel->adc_buffer));
		} else {
			ring_buffer_append(channel->filtered_adc_buffer, ring_buffer_second_half_average(channel->adc_buffer));
		}
	}
}

FuseState update_channel(Channel* channel, DiagnosticState diagnostic_state) {
	// TODO: Detect if any of the pins (i.e. SEL1, SEL2, DIA_EN, or ADC pins are
	// disconnected using deduction, cross checking measurements such as voltage)

	// TODO: Get out of open load fault somehow

	if(channel->fuse_state == FUSE_AUTO_RETRY) {
		channel->retry_timer_ms++;

		if(channel->retry_timer_ms >= FUSE_RETRY_DELAY_MS) {
			channel->set_switch = SWITCH_ENABLED;
		}
	}

	boolean adc_maxed = FALSE;

	for(U16 i = 0; i < channel->filtered_adc_buffer->item_count; i++) {
		if(ring_buffer_get(channel->filtered_adc_buffer, i) == MAX_ADC_READING) {
			adc_maxed = TRUE;
		}
	}

	// Equation to calculate SNS current (in milliamperes) from ADC reading is
	// Isns = 1000*(MAX_ADC_VOLTAGE(ADC/MAX_ADC_READING))/SNS_SHUNT_RESISTANCE_OHMS
	U16 adc_reading = ring_buffer_passive_average(channel->filtered_adc_buffer);
	double i_sns = (1000.0*MAX_ADC_VOLTAGE*adc_reading)/(MAX_ADC_READING*SNS_SHUNT_RESISTANCE_OHMS);

	if(diagnostic_state == DIAGNOSTICS_CURRENT && channel->switch_state) {
		if(i_sns >= SNS_FAULT_HIGH_CURRENT_MILLIAMPERES || adc_maxed) {
			channel->hw_fault = OVERCURRENT_FAULT;
		}
		channel->current_integrator *= FUSE_DECAY_CONSTANT;
		// Iswitch=Isns*CURRENT_SENSE_RATIO
		channel->switch_current = CURRENT_SENSE_RATIO*i_sns;
		channel->current_integrator += fmax(CURRENT_SENSE_RATIO*i_sns - FUSE_RATING_MILLIAMPERE[channel->id], 0);
		// Normalize the integrator based on it's rating, then check if it's above the 50% sum limit
		if(channel->current_integrator*FUSE_INTEGRATOR_LIMIT_SCALAR > 0.5*FUSE_RATING_MILLIAMPERE[channel->id]) { // @suppress("Avoid magic numbers")
			channel->set_switch = SWITCH_DISABLED;
			if(channel->retries < MAX_FUSE_RETRIES) {
				channel->fuse_state = FUSE_AUTO_RETRY;
			} else {
				channel->fuse_state = FUSE_DISABLED;
			}
			channel->retries++;
		}
	}

	if(diagnostic_state == DIAGNOSTICS_TEMP) {
		// TODO: Overtemp detection
		// Equation for switch temperature is
		// Temp = (Isns-0.85)*TEMPERATURE_SENSE_RATIO+25C
		channel->switch_temp = (i_sns-0.85)*TEMPERATURE_SENSE_RATIO+25; // @suppress("Avoid magic numbers")
	}

	if(diagnostic_state == DIAGNOSTICS_VOLTAGE) {
		// Equation for switch voltage on supply side is
		// Voltage=Isns*VOLTAGE_SENSE_RATIO
		channel->switch_voltage = i_sns*VOLTAGE_SENSE_RATIO;
	}

	if(diagnostic_state == DIAGNOSTICS_OL_STB && channel->set_switch &&
			!channel->switch_state && channel->hw_fault != OVERCURRENT_FAULT)
	{
		if(i_sns >= SNS_FAULT_HIGH_CURRENT_MILLIAMPERES || adc_maxed) {
			channel->hw_fault = OPEN_LOAD_FAULT;
		} else {
			HAL_GPIO_WritePin(SWITCH_EN_PORTS[channel->id], SWITCH_EN_PINS[channel->id], SET);
			channel->switch_state = SWITCH_ENABLED;
		}
	}

	if(!channel->set_switch && channel->switch_state) {
		HAL_GPIO_WritePin(SWITCH_EN_PORTS[channel->id], SWITCH_EN_PINS[channel->id], RESET);
		channel->switch_state = SWITCH_DISABLED;
	}

	return channel->fuse_state;
}

void set_channel(Channel* channel, SwitchState on) {
	channel->set_switch = on;
}

void free_channel(Channel* channel) {

}
