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
#include "base_types.h"
#include <math.h>

void init_channel(
		Channel* channel,
		U8 id,
		volatile U32* time_micros,
		DiagnosticState* diagnostic_state,
		boolean populated,
		U16 max_retries,
		U32 max_integrator,
		U16 max_current,
		U16 rating,
		U16 retry_delay,
		U16 en_pin,
		GPIO_TypeDef* en_port) {
	channel->id = id;

	channel->diagnostic_state = diagnostic_state;

	channel->populated = populated;

	channel->hw_switch.set = populated ? SWITCH_ENABLED : SWITCH_DISABLED;
	channel->hw_switch.state = SWITCH_DISABLED;
	channel->hw_switch.i_sns = 0;
	channel->hw_switch.no_load = TRUE;
	channel->hw_switch.en_pin = en_pin;
	channel->hw_switch.en_port = en_port;

	channel->timing.time_micros = time_micros;
	channel->timing.micros_last = 0;
	channel->timing.micros_left = 0;
	channel->timing.invalid = FALSE;

	channel->fuse.state = FUSE_ACTIVE;
	channel->fuse.retries = 0;
	channel->fuse.max_retries = max_retries;
	channel->fuse.retry_timer_ms = 0;
	channel->fuse.retry_delay_ms = retry_delay;
	channel->fuse.integrator = 0;
	channel->fuse.max_integrator = max_integrator;
	channel->fuse.max_current = max_current;
	channel->fuse.rating = rating;

	channel->brownout.disabled = FALSE;
	channel->brownout.recent_retries = 0;

	init_ring_buffer_static(CHANNEL_FILTERED_BUFFER_SIZE,
			&channel->adc.filtered_adc_buffer,
			channel->adc.filtered_adc_buffer_array);
	channel->adc.adc_reading = 0;

	channel->readings.current = 0;
	channel->readings.temp = 0.0;
	channel->readings.voltage = 0;
}

void channel_adc_interrupt(Channel* channel, U16 measurement) {
	if(!channel->timing.invalid) {
		update_channel_fuse_interrupt(channel, measurement);
		ring_buffer_append(&channel->adc.filtered_adc_buffer, measurement);
	}
}


/*
 * This function runs in the ADC interrupt and checks for overcurrent
 */
void update_channel_fuse_interrupt(Channel* channel, U16 measurement) {
	if(*channel->diagnostic_state == DIAGNOSTIC_CURRENT) {
		// If the switch is enabled and we're in current mode
		if(channel->hw_switch.state == SWITCH_ENABLED) {
			// Check switch fast blow
			if(measurement >= ( MAX_ADC_READING - MAX_ADC_MARGIN ) ||
				calculate_i_sns(measurement)*CURRENT_SENSE_RATIO >= channel->fuse.max_current) {
				channel_blow_fuse(channel);
			}
		}
	}
}

void update_channel(Channel* channel) {

	update_channel_fuse(channel);

	update_channel_read_buffer(channel);

	update_channel_switch(channel);

}


void update_channel_fuse(Channel* channel) {
	if(channel->fuse.state == FUSE_AUTO_RETRY) {
		channel->fuse.retry_timer_ms += FUSE_UPDATE_PERIOD_MS;

		if(channel->fuse.retry_timer_ms >= channel->fuse.retry_delay_ms) {
			channel->fuse.state = FUSE_ACTIVE;
			set_channel(channel, SWITCH_ENABLED);
		}
	}

	if(*channel->diagnostic_state == DIAGNOSTIC_CURRENT && channel->hw_switch.state) {
		channel->fuse.integrator += channel->readings.current - channel->fuse.rating;

		if(channel->fuse.integrator > channel->fuse.max_integrator) {
			channel_blow_fuse(channel);
		}
	}

}

void channel_blow_fuse(Channel* channel) {
	if(channel->fuse.state != FUSE_ACTIVE) {
		return;
	}
	if(channel->fuse.retries < channel->fuse.max_retries|| channel->fuse.max_retries < 0) {
		channel->fuse.state = FUSE_AUTO_RETRY;
	} else {
		channel->fuse.state = FUSE_DISABLED;
	}
	if(channel->fuse.retries >= 0) {
		channel->fuse.retries++;
	}
	set_channel(channel, SWITCH_DISABLED);
	channel->fuse.integrator = 0;
	update_channel_switch(channel);
}

/**
 * Reads data from ADC filtered buffer
 * @param channel The channel to update
 * @return A boolean representing if the ADC was maxed out
 */
boolean update_channel_read_buffer(Channel* channel) {
	// Delay for a couple of microseconds so we can do stuff with the buffer
	update_channel_timing(channel, CHANNEL_UPDATE_DELAY_US);

	// Store the ADC reading
	channel->adc.adc_reading = ring_buffer_passive_average(&channel->adc.filtered_adc_buffer);

	// Clear the buffer
	ring_buffer_clear(&channel->adc.filtered_adc_buffer);

	channel->hw_switch.i_sns = calculate_i_sns(channel->adc.adc_reading);

	if(channel->adc.filtered_adc_buffer.item_count == 0) {
		channel->readings.current = 0;
		channel->readings.temp = 0;
		channel->readings.voltage = 0;
		return FALSE;
	}

	if(*channel->diagnostic_state == DIAGNOSTIC_CURRENT) {
		if(channel->hw_switch.state) {
			// Iswitch=Isns*CURRENT_SENSE_RATIO
			channel->readings.current = CURRENT_SENSE_RATIO*channel->hw_switch.i_sns/MICROAMPS_PER_MILLIAMP;
		} else {
			channel->readings.current = 0;
		}
	}

	if(*channel->diagnostic_state == DIAGNOSTIC_TEMP) {
		// Equation for switch temperature is
		// Temp = (Isns-0.85)*TEMPERATURE_SENSE_RATIO+25C
		channel->readings.temp = (channel->hw_switch.i_sns/MICROAMPS_PER_MILLIAMP-0.85)*TEMPERATURE_SENSE_RATIO+25; // @suppress("Avoid magic numbers")
	}

	if(*channel->diagnostic_state == DIAGNOSTIC_VOLTAGE) {
		// Equation for switch voltage on supply side is
		// Voltage=Isns*VOLTAGE_SENSE_RATIO
		channel->readings.voltage = channel->hw_switch.i_sns/MICROAMPS_PER_MILLIAMP*VOLTAGE_SENSE_RATIO;
	}

	return channel->adc.adc_reading >= ( MAX_ADC_READING - MAX_ADC_MARGIN );
}

U16 calculate_i_sns(U16 adc_reading) {
	// Equation to calculate SNS current (in milliamperes) from ADC reading is
	// Isns = (1000mA/A)*(MAX_ADC_VOLTAGE(ADC/MAX_ADC_READING))/SNS_SHUNT_RESISTANCE_OHMS
	return (MICROAMPS_PER_AMP*MAX_ADC_VOLTAGE*adc_reading)/(MAX_ADC_READING*SNS_SHUNT_RESISTANCE_OHMS);
}

void update_channel_switch(Channel* channel) {

	update_channel_timing(channel, 0);

	update_hw_open_load(channel);

	// Make sure we checked open load before we enable the switch
	if(!channel->hw_switch.state && channel->hw_switch.set && !channel->hw_switch.no_load) {
		//HAL_GPIO_WritePin(channel->hw_switch.en_port, channel->hw_switch.en_pin, SET);
		channel->hw_switch.state = SWITCH_ENABLED;
		update_channel_timing(channel, T_SNS_EN_ON);
	} else if(channel->hw_switch.state && !channel->hw_switch.set) {
		HAL_GPIO_WritePin(channel->hw_switch.en_port, channel->hw_switch.en_pin, RESET);
		channel->hw_switch.state = SWITCH_DISABLED;
		update_channel_timing(channel, T_SNS_EN_OFF);
	}
}

/*
 * Updates the timing of a channel
 * @param channel The channel to update
 * @param delay_micros A transition delay in microseconds
 */
void update_channel_timing(Channel* channel, U32 delay_micros) {
	// Set the timing to invalid so that the interrupt doesn't mess with
	// anything while we're updating stuff
	channel->timing.invalid = TRUE;

	// time_micros - time in micros
	U32 time_micros = *channel->timing.time_micros;
	U32* micros_left = &channel->timing.micros_left;
	U32* micros_last = &channel->timing.micros_last;

	// Add a little bit of extra time in case an interrupt gets called or something
	delay_micros += T_SNS_MARGIN;

	boolean was_invalid = *micros_left > 0;

	*micros_left -= time_micros - *micros_last;
	*micros_left = (*micros_left > delay_micros) ? *micros_left : delay_micros;
	*micros_last = time_micros;
	*micros_left = ((S32)*micros_left) < 0 ? 0 : *micros_left;

	if(was_invalid && *micros_left == 0) {
		ring_buffer_clear(&channel->adc.filtered_adc_buffer);
	}

	channel->timing.invalid = *micros_left > 0;
}

boolean update_hw_open_load(Channel* channel) {
	if(*channel->diagnostic_state == DIAGNOSTIC_OL_STB && !channel->hw_switch.state) {
		channel->hw_switch.no_load = channel->hw_switch.i_sns >= SNS_FAULT_HIGH_CURRENT_MICROAMPERES
				|| channel->adc.adc_reading >= ( MAX_ADC_READING - MAX_ADC_MARGIN )
				|| channel->adc.filtered_adc_buffer.item_count == 0;
		return TRUE;
	} else {
		return FALSE;
	}
}


void set_channel(Channel* channel, SwitchState on) {
	channel->hw_switch.set = on;
}
