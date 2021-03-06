/*
 * config.c
 *
 *  Created on: Feb 27, 2022
 *      Author: Ben Abbott
 */

#include "config.h"
#include "ring_buffer.h"


// ===========================================
// || GPIO Pin/Port Configuration Constants ||
// ===========================================


// ADC Inputs from diagnostic pins on switches

const U16 ADC_INPUT_PINS[] = { ADC_INPUT0_Pin, ADC_INPUT1_Pin, ADC_INPUT2_Pin, ADC_INPUT3_Pin,
		ADC_INPUT4_Pin, ADC_INPUT5_Pin, ADC_INPUT6_Pin, ADC_INPUT7_Pin, ADC_INPUT8_Pin, ADC_INPUT9_Pin,
		ADC_INPUT10_Pin, ADC_INPUT11_Pin, ADC_INPUT12_Pin, ADC_INPUT13_Pin, ADC_INPUT14_Pin, ADC_INPUT15_Pin,
		ADC_INPUT16_Pin, ADC_INPUT17_Pin, ADC_INPUT18_Pin, ADC_INPUT19_Pin };

const GPIO_TypeDef* ADC_INPUT_PORTS[] = { ADC_INPUT0_GPIO_Port, ADC_INPUT1_GPIO_Port, ADC_INPUT2_GPIO_Port, ADC_INPUT3_GPIO_Port,
		ADC_INPUT4_GPIO_Port, ADC_INPUT5_GPIO_Port, ADC_INPUT6_GPIO_Port, ADC_INPUT7_GPIO_Port, ADC_INPUT8_GPIO_Port, ADC_INPUT9_GPIO_Port,
		ADC_INPUT10_GPIO_Port, ADC_INPUT11_GPIO_Port, ADC_INPUT12_GPIO_Port, ADC_INPUT13_GPIO_Port, ADC_INPUT14_GPIO_Port, ADC_INPUT15_GPIO_Port,
		ADC_INPUT16_GPIO_Port, ADC_INPUT17_GPIO_Port, ADC_INPUT18_GPIO_Port, ADC_INPUT19_GPIO_Port };

const boolean ADC1_CHANNELS[] = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
		TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE };

const boolean ADC3_CHANNELS[] = { TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE,
		FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE };

// Switch enable pins

const U16 SWITCH_EN_PINS[] = { EN0_Pin, EN1_Pin, EN2_Pin, EN3_Pin, EN4_Pin, EN5_Pin, EN6_Pin, EN7_Pin, EN8_Pin, EN9_Pin,
		EN10_Pin, EN11_Pin, EN12_Pin, EN13_Pin, EN14_Pin, EN15_Pin, EN16_Pin, EN17_Pin, EN18_Pin, EN19_Pin };

const GPIO_TypeDef* SWITCH_EN_PORTS[] = { EN0_GPIO_Port, EN1_GPIO_Port, EN2_GPIO_Port, EN3_GPIO_Port, EN4_GPIO_Port, EN5_GPIO_Port,
		EN6_GPIO_Port, EN7_GPIO_Port, EN8_GPIO_Port, EN9_GPIO_Port, EN10_GPIO_Port, EN11_GPIO_Port, EN12_GPIO_Port, EN13_GPIO_Port,
		EN14_GPIO_Port, EN15_GPIO_Port, EN16_GPIO_Port, EN17_GPIO_Port, EN18_GPIO_Port, EN19_GPIO_Port };

// The rating of each of the fuses in milliamperes
const U16 FUSE_RATING[]           = { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
// The peak current in milliamperes
const U16 FUSE_PEAK_CURRENT[]     = { 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000 };
// The peak current duration in milliseconds
const U16 FUSE_PEAK_TIME_MS[]     = { 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500 };
// The fuses's fast-blow rating (the amount of peak current it can take) in millamperes
const U16 FUSE_FAST_BLOW_RATING[] = { 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000 };
// The amount of retries allow for each channel's fuse
const S16 FUSE_RETRIES[]          = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
// The delay before each channel is retried after a software fuse is blown in milliseconds
const U16 FUSE_RETRY_DELAY_MS[]   = { 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250 };
// TODO: Implement warnings with open load
const boolean POPULATED_CHANNELS[] = { TRUE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
		FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE };
