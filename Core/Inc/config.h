/*
 * config.h
 *
 *  Created on: Jan 9, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "base_types.h"
#include <math.h>
#include "main.h"

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


// ======================
// || Switch Constants ||
// ======================

// The ratio between current flowing through the switch and current measured
// on the SNS pin
#define CURRENT_SENSE_RATIO 4600

// The temperature change in degrees Celsius per mA output by the SNS pin
#define TEMPERATURE_SENSE_RATIO 89.2857143

// The change in supply voltage per mA output by the SNS pin
#define VOLTAGE_SENSE_RATIO 11.5340254

// The value of the resistor that connects the SNS pin to ground
#define SNS_SHUNT_RESISTANCE_OHMS 500

// The value of the SNS pin that indicates a fault
#define SNS_FAULT_HIGH_CURRENT_MILLIAMPERES 6

// From https://www.ti.com/lit/ds/symlink/tps1ha08-q1.pdf

#define T_SNS_EN_ON  180
#define T_SNS_EN_OFF 700
#define T_SNS_DIA_ON 70
#define T_SNS_DIA_SW 60

// ===================================
// || Other Configuration Constants ||
// ===================================

// Maximum ADC reading
#define MAX_ADC_READING 4095

// Maximum ADC Voltage
#define MAX_ADC_VOLTAGE 3.3

// Voltage and temperature read rates, current is logged
// on all other task calls
#define VOLTAGE_RATE_HZ 10
#define TEMP_RATE_HZ 10

// How long the fuse integrates current measurements
#define FUSE_SAMPLE_PERIOD_MS 1

#define MAX_FUSE_RETRIES 10
#define FUSE_RETRY_DELAY_MS 50

#define NUM_CHANNELS 20

#define ADC1_NUM_PINS 10
#define ADC3_NUM_PINS 10

#define ADC1_BUF_LEN CHANNEL_ADC_BUFFER_SIZE*ADC1_NUM_PINS
#define ADC3_BUF_LEN CHANNEL_ADC_BUFFER_SIZE*ADC3_NUM_PINS

#define VOLTAGE_PERIOD_TICKS configTICK_RATE_HZ/VOLTAGE_RATE_HZ
#define TEMP_PERIOD_TICKS configTICK_RATE_HZ/TEMP_RATE_HZ


// The size of the ADC buffer for each channel
#define CHANNEL_ADC_BUFFER_SIZE 100

// The size of each channel's filtered ADC buffer
// Ideally, this buffer should take just about one tick to fill up, so that by
// the next tick all previous measurements have been overwritten and there is
// no bleed-over from one tick to the next
#define CHANNEL_FILTERED_BUFFER_SIZE 1000

#define MILLIAMPS_PER_AMP 1000

#endif /* INC_CONFIG_H_ */
