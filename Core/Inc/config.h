/*
 * config.h
 *
 *  Created on: Jan 9, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "base_types.h"
#include "main.h"

// ===========================================
// || GPIO Pin/Port Configuration Constants ||
// ===========================================


// ADC Inputs from diagnostic pins on switches

static const U16 ADC_INPUT_PINS[] = { ADC_INPUT0_Pin, ADC_INPUT1_Pin, ADC_INPUT2_Pin, ADC_INPUT3_Pin,
		ADC_INPUT4_Pin, ADC_INPUT5_Pin, ADC_INPUT6_Pin, ADC_INPUT7_Pin, ADC_INPUT8_Pin, ADC_INPUT9_Pin,
		ADC_INPUT10_Pin, ADC_INPUT11_Pin, ADC_INPUT12_Pin, ADC_INPUT13_Pin, ADC_INPUT14_Pin, ADC_INPUT15_Pin,
		ADC_INPUT16_Pin, ADC_INPUT17_Pin, ADC_INPUT18_Pin, ADC_INPUT19_Pin };

static const GPIO_TypeDef* ADC_INPUT_PORTS[] = { ADC_INPUT0_GPIO_Port, ADC_INPUT1_GPIO_Port, ADC_INPUT2_GPIO_Port, ADC_INPUT3_GPIO_Port,
		ADC_INPUT4_GPIO_Port, ADC_INPUT5_GPIO_Port, ADC_INPUT6_GPIO_Port, ADC_INPUT7_GPIO_Port, ADC_INPUT8_GPIO_Port, ADC_INPUT9_GPIO_Port,
		ADC_INPUT10_GPIO_Port, ADC_INPUT11_GPIO_Port, ADC_INPUT12_GPIO_Port, ADC_INPUT13_GPIO_Port, ADC_INPUT14_GPIO_Port, ADC_INPUT15_GPIO_Port,
		ADC_INPUT16_GPIO_Port, ADC_INPUT17_GPIO_Port, ADC_INPUT18_GPIO_Port, ADC_INPUT19_GPIO_Port };

static const boolean ADC1_PINS[] = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
		TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE };

static const boolean ADC3_PINS[] = { TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE,
		FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE };

// Switch enable pins

// TODO: Add back EN10 and EN14 pins

static const U16 SWITCH_EN_PINS[] = { EN0_Pin, EN1_Pin, EN2_Pin, EN3_Pin, EN4_Pin, EN5_Pin, EN6_Pin, EN7_Pin, EN8_Pin, EN9_Pin,
		/*EN10_Pin, */EN11_Pin, EN12_Pin, EN13_Pin,/* EN14_Pin, */EN15_Pin, EN16_Pin, EN17_Pin, EN18_Pin, EN19_Pin };

static const GPIO_TypeDef* SWITCH_EN_PORTS[] = { EN0_GPIO_Port, EN1_GPIO_Port, EN2_GPIO_Port, EN3_GPIO_Port, EN4_GPIO_Port, EN5_GPIO_Port,
		EN6_GPIO_Port, EN7_GPIO_Port, EN8_GPIO_Port, EN9_GPIO_Port,/* EN10_GPIO_Port, */EN11_GPIO_Port, EN12_GPIO_Port, EN13_GPIO_Port,
		/*EN14_GPIO_Port, */EN15_GPIO_Port, EN16_GPIO_Port, EN17_GPIO_Port, EN18_GPIO_Port, EN19_GPIO_Port };

// Digital I/O on 14 pin I/O connector

static const U16 DIO_PINS[] = { DIO0_Pin, DIO1_Pin, DIO2_Pin, DIO3_Pin, DIO4_Pin, DIO5_Pin };

static const GPIO_TypeDef* DIO_PORTS[] = { DIO0_GPIO_Port, DIO1_GPIO_Port, DIO2_GPIO_Port, DIO3_GPIO_Port, DIO4_GPIO_Port, DIO5_GPIO_Port };

// Latch, SEL1, SEL2, and DIA_EN pins

#define LATCH_PIN LATCH_Pin
#define LATCH_PORT LATCH_GPIO_Port

#define SEL1_PIN SEL1_Pin
#define SEL1_PORT SEL1_GPIO_Port

#define SEL2_PIN SEL2_Pin
#define SEL2_PORT SEL2_GPIO_Port

#define DIA_EN_PIN DIA_EN_Pin
#define DIA_EN_PORT DIA_EN_GPIO_Port

// I2C SCL and SCK, along with a shake interrupt pin

#define SCL_PIN SCL_Pin
#define SCL_PORT SCL_GPIO_Port

#define SDA_PIN SDA_Pin
#define SDA_PORT SDA_GPIO_Port

#define SHAKE_INT_PIN SHAKE_INT_Pin
#define SHAKE_INT_PORT SHAKE_INT_GPIO_Port


// =======================================
// || GopherCAN Configuration Constants ||
// =======================================

// Used to define what GopherCAN module this board is
#define GCAN_MODULE_ID PDM_ID


// ===================================
// || Other Configuration Constants ||
// ===================================

// The size of the ADC buffer for each channel
#define CHANNEL_ADC_BUFFER_SIZE 700

// The size of each channel's primary current buffer
#define PRIMARY_CURRENT_BUFFER_SIZE 1000

// The number of pins on each of the ADCs
#define ADC1_NUM_PINS 10
#define ADC3_NUM_PINS 10

// Voltage and temperature read rates, current is logged
// on all other task calls
#define VOLTAGE_RATE_HZ 10
#define TEMP_RATE_HZ 10


// How long the fuse integrates current measurements
#define FUSE_INTEGRATION_LENGTH_MS 5000
#define FUSE_SAMPLE_PERIOD_MS 20
#define FUSE_CURRENT_AVERAGE_SAMPLES 10
#define FUSE_AVERAGE_RATING_AMPERE 10

#define DEFAULT_FUSE_STATE ACTIVE

#define MAX_FUSE_RETRIES 10
#define FUSE_RETRY_DELAY_MS 10

#define CONTINUOUS_CURRENT_TIME_MS 1000

#define NUM_CHANNELS 20

#define ADC1_BUF_LEN CHANNEL_ADC_BUFFER_SIZE*ADC1_NUM_PINS
#define ADC3_BUF_LEN CHANNEL_ADC_BUFFER_SIZE*ADC1_NUM_PINS

#define FUSE_INTEGRATOR_LIMIT FUSE_AVERAGE_RATING_AMPERE*FUSE_INTEGRATION_LENGTH_MS

#define VOLTAGE_PERIOD_TICKS configTICK_RATE_HZ/VOLTAGE_RATE_HZ
#define TEMP_PERIOD_TICKS configTICK_RATE_HZ/TEMP_RATE_HZ


#endif /* INC_CONFIG_H_ */
