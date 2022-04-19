/*
 * led.c
 *
 *  Created on: Apr 16, 2022
 *      Author: Ben Abbott
 */

#include "led.h"
#include "config.h"

/**
 * Initializes a new LED struct with the given values and sets the GPIO pin
 *
 * @param led A pointer to the LED struct to configure
 * @param pin The GPIO pin number of the led
 * @param port The GPIO port of the led
 */
void init_led(LED* led, U16 pin, GPIO_TypeDef* port) {
	led->pin = pin;
	led->port = port;
	led->on = FALSE;
	HAL_GPIO_WritePin(led->port, led->pin, led->on);
}

/**
 * Initializes a pattern on the given led struct
 *
 * @param led The led to initialize
 * @param time_us A pointer to an integer that represents the time in microseconds
 * @param pattern_ms An array that stores the deltas between the pattern in milliseconds
 * @param pattern_len The length of pattern_ms; must be even or will be rounded down
 */
void init_led_pattern(LED* led, volatile U32* time_us, U32* pattern_ms, U16 pattern_len) {
	led->time_us = time_us;
	led->last_time_us = *time_us;
	led->pattern_ms = pattern_ms;
	led->pattern_timer_us = 0;
	// Round pattern_len down to the nearest even number
	led->pattern_len = pattern_len & (-2);
	led->pattern_index = 0;
}



/**
 * Sets a pattern on the given led struct
 *
 * @param led The LED to change
 * @param pattern_ms An array that stores the deltas between the pattern in milliseconds
 * @param pattern_len The length of pattern_ms; must be even or will be rounded down
 */
void set_led_pattern(LED* led, U32* pattern_ms, U16 pattern_len) {
	led->pattern_ms = pattern_ms;
	led->pattern_timer_us = 0;
	// Round pattern_len down to the nearest even number
	led->pattern_len = pattern_len & (-2);
	led->pattern_index = 0;
}

/**
 * Sets an LED to a certain state
 *
 * @param led The LED to set
 * @param on The state the led should be set to
 */
void set_led(LED* led, boolean on) {
	led->on = on;
	HAL_GPIO_WritePin(led->port, led->pin, led->on);
}

void update_led(LED* led) {
	U32 delta_us = (*led->time_us)-led->last_time_us;
	led->last_time_us += delta_us;
	led->pattern_timer_us += delta_us;
	U32 pattern_timer_ms = led->pattern_timer_us / MICROSECONDS_PER_MILLISECOND;
	while(pattern_timer_ms >= led->pattern_ms[led->pattern_index]) {
		pattern_timer_ms -= led->pattern_ms[led->pattern_index];
		led->pattern_index++;
		led->pattern_index %= led->pattern_len;
		led->on = !led->on;
		HAL_GPIO_WritePin(led->port, led->pin, led->on);
	}
}
