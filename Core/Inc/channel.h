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

#define SEL1_MASK       0x01
#define SEL2_MASK       0x02
#define DIA_EN_MASK     0x04

typedef enum {
	DIAGNOSTIC_DISABLED   = 0b000,
	DIAGNOSTIC_CURRENT    = 0b100,
	DIAGNOSTIC_TEMP       = 0b101,
	DIAGNOSTIC_VOLTAGE    = 0b111,
	// DIAGNOSTICS_CURRENT and DIAGNOSTICS_OL_STB are the same,
	// they are determined by switch enable
	DIAGNOSTIC_OL_STB     = 0b100
} DiagnosticState;

typedef enum {
	FUSE_ACTIVE, FUSE_AUTO_RETRY, FUSE_DISABLED
} FuseState;

typedef enum {
	SWITCH_DISABLED, SWITCH_ENABLED
} SwitchState;

typedef struct {
	// The channel number, or "id"
	U8 id;

	// A pointer to the global diagnostic state
	DiagnosticState* diagnostic_state;

	// A boolean representing if there's something attached to the switch
	boolean populated;

	// Switch Stuff

	struct Switch {
		// If the channel is desired to be on/off (in software), independent of
		// the actual physical state of the switch
		SwitchState set;
		// If the switch is enabled or not
		SwitchState state;
		// SNS Current
		U16 i_sns;
		// If there is no load connected to the switch
		boolean no_load;
		// Switch enable pin
		U16 en_pin;
		// Switch enable port
		GPIO_TypeDef* en_port;
	} hw_switch;



	struct Timing {
		// A pointer to a integer representing the time in microseconds
		volatile U32* time_micros;
		// Last recorded time in microseconds
		U32 micros_last;
		// The time left in microseconds before transitioning is done
		U32 micros_left;
		// If ADC measurements are valid or not
		boolean invalid;
	} timing;


	// Software Fuse Stuff

	struct Fuse {
		// Current state of the fuse
		FuseState state;
		// The number of times this channel has been disabled and re-switch_state
		U16 retries;
		// The maximum number of retries allowed
		U16 max_retries;

		// The amount of time that has passed since the channel has been disbaled
		U16 retry_timer_ms;
		// The amount of time to wait to retry the channel after overcurrent
		U16 retry_delay_ms;
		// The current integrator for software overcurrent
		U32 integrator;
		// Maximum integrator value
		U32 max_integrator;

		// Maximum current value
		U16 max_current;
		// Fuse rating
		U16 rating;
	} fuse;

	// Brownout Stuff

	struct BrownoutProtetion {
		// Check if this channel is disabled due to brownout protection
		boolean disabled;
		// The number of times this module has been turned on and off recently
		// due to brownout protection
		U8 recent_retries;
	} brownout;

	// ADC Stuff

	struct ChannelADC {

		// The buffer for storing recent averaged ADC measurements
		U16RingBuffer filtered_adc_buffer;

		// U16 Array for U16RingBuffer
		U16 filtered_adc_buffer_array[CHANNEL_FILTERED_BUFFER_SIZE];

		// Averaged ADC reading
		U16 adc_reading;

	} adc;

	// Variables for storing recent measurements

	struct ChannelReadings {
		// The most recent current measurement in milliamperes
		U16 current;
		// The temperature of the high-side switch in degrees Celsius
		float temp;
		// The voltage on the supply side of the switch
		U16 voltage;
	} readings;

} Channel;

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
		GPIO_TypeDef* en_port);
void channel_adc_interrupt(Channel* channel, U16 measurement);
void update_channel_fuse_interrupt(Channel* channel, U16 measurement);
void update_channel(Channel* channel);
void update_channel_fuse(Channel* channel);
void channel_blow_fuse(Channel* channel);
boolean update_channel_read_buffer(Channel* channel);
void update_channel_switch(Channel* channel);
void update_channel_timing(Channel* channel, U32 delay_micros);
boolean update_hw_open_load(Channel* channel);
void set_channel(Channel* channel, SwitchState on);
U16 calculate_i_sns(U16 adc_reading);

#endif /* INC_CHANNEL_H_ */
