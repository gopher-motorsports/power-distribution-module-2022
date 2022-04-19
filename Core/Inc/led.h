/*
 * led.h
 *
 *  Created on: Apr 16, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "base_types.h"
#include "stm32f756xx.h"

typedef struct {
	U16 pin;
	GPIO_TypeDef* port;
	volatile U32* time_us;
	U32 last_time_us;
	U32* pattern_ms;
	U16 pattern_timer_us;
	U16 pattern_len;
	U16 pattern_index;
	boolean on;
} LED;

void init_led(LED* led, U16 pin, GPIO_TypeDef* port);
void init_led_pattern(LED* led, volatile U32* time_us, U32* pattern_ms, U16 pattern_len);
void set_led_pattern(LED* led, U32* pattern_ms, U16 pattern_len);
void set_led(LED* led, boolean on);
void update_led(LED* led);

#endif /* INC_LED_H_ */
