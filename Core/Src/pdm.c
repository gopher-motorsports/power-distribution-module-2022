/*
 * pdm.c
 *
 *  Created on: Jan 16, 2022
 *      Author: Ben Abbott
 */

#include "pdm.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "cmsis_os.h"
#include "channel.h"
#include "channel_control.h"
#include "led.h"


// TODO: Priority 2: UART Errors
// TODO: Priority 2: Program error codes on LED
// TODO: Priority 3: Implement logging
// TODO: Priority 3: Verify and update all documentation
// TODO: Priority 3: Go over all header files and prune code
// TODO: Priority 3: Try to increase ADC sample time to decrease noise instead of filtering so much
// TODO: Priority 4: Implement fault transmission over CAN to driver display
// TODO: Priority 4: Implement processor temperature sensing via internal sensor?
// TODO: Priority 4: Implement board condition sensing (I2C)?
// TODO: Priority 5: Implement software open-load protection
// TODO: Priority 5: Detect switch faults/warnings (low voltage, high temp, etc.)
// TODO: Priority 5: Detect external overrides of manually controlled pins (i.e. fuel pump, spark, etc.)
// TODO: Priority 5: Detect if any of the pins (i.e. SEL1, SEL2, DIA_EN, or ADC pins are
// TODO: Priority 5: Sticky faults?
// disconnected using deduction, cross checking measurements such as voltage)

// ==============================
// || STATIC MEMORY ALLOCATION ||
// ==============================

volatile U16 ADC1_BUF[ADC1_BUF_LEN*2];
volatile U16 ADC3_BUF[ADC3_BUF_LEN*2];
U16RingBuffer ADC1_RING_BUF_1;
U16RingBuffer ADC3_RING_BUF_1;
U16RingBuffer ADC1_RING_BUF_2;
U16RingBuffer ADC3_RING_BUF_2;
Channel CHANNELS[NUM_CHANNELS];

LED FAULT_LED;
LED STATUS_LED;
LED ADC_LED;

UART_HandleTypeDef* huart;

DiagnosticStateController DSC;

ADC_HandleTypeDef* hadc1_ptr;
ADC_HandleTypeDef* hadc3_ptr;
extern boolean ADC1_CHANNELS[];
extern boolean ADC3_CHANNELS[];

TIM_HandleTypeDef* tim;

void init_pdm(TIM_HandleTypeDef* tim_ptr, ADC_HandleTypeDef* adc1_ptr, ADC_HandleTypeDef* adc3_ptr, UART_HandleTypeDef* huart_ptr) {
	hadc1_ptr = adc1_ptr;
	hadc3_ptr = adc3_ptr;
	tim = tim_ptr;
	huart = huart_ptr;
	init_dsc(&DSC, VOLTAGE_PERIOD_TICKS, TEMP_PERIOD_TICKS);
	init_channels(CHANNELS, &DSC.state, &tim->Instance->CNT);
	init_adc_dma(hadc1_ptr, ADC1_BUF, ADC1_BUF_LEN*2);
	init_adc_dma(hadc3_ptr, ADC3_BUF, ADC3_BUF_LEN*2);
	init_ring_buffer_static(ADC1_BUF_LEN, &ADC1_RING_BUF_1, ADC1_BUF);
	init_ring_buffer_static(ADC3_BUF_LEN, &ADC3_RING_BUF_1, ADC3_BUF);
	init_ring_buffer_static(ADC1_BUF_LEN, &ADC1_RING_BUF_2, ADC1_BUF+ADC1_BUF_LEN);
	init_ring_buffer_static(ADC3_BUF_LEN, &ADC3_RING_BUF_2, ADC3_BUF+ADC3_BUF_LEN);
	init_led_pattern(&FAULT_LED, &tim->Instance->CNT, 0, 0);
	init_led_pattern(&STATUS_LED, &tim->Instance->CNT, 0, 0);
	HAL_TIM_Base_Start(tim);
}


void init_leds() {
	init_led(&FAULT_LED, FAULT_LED_Pin, FAULT_LED_GPIO_Port);
	init_led(&STATUS_LED, STATUS_LED_Pin, FAULT_LED_GPIO_Port);
	init_led(&ADC_LED, ADC_LED_Pin, ADC_LED_GPIO_Port);
}

void set_all_leds(boolean fault, boolean status, boolean adc) {
	set_led(&FAULT_LED, fault);
	set_led(&STATUS_LED, status);
	set_led(&ADC_LED, adc);
}


void init_adc_dma(ADC_HandleTypeDef* hadc, volatile U16 buffer[], U16 buffer_size) {
	HAL_ADC_Start_DMA(hadc, (uint32_t*)buffer, buffer_size);
}

void print_uart(UART_HandleTypeDef* huart, char* msg) {
	U8 msg_buf[strlen(msg)+1];
	strcpy((char*)msg_buf, msg);
	HAL_UART_Transmit(huart, msg_buf, strlen((char*)msg_buf), HAL_MAX_DELAY);
}

void main_loop() {
	update_diagnostics_queue(&DSC);
	update_diagnostics(CHANNELS, &DSC);
	update_all_channels();
	if(HAL_GetTick() % 1000 == 0) {
		char msg[80];
		sprintf(msg, "%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d\n",
				CHANNELS[0].adc.adc_reading,
				CHANNELS[1].adc.adc_reading,
				CHANNELS[2].adc.adc_reading,
				CHANNELS[3].adc.adc_reading,
				CHANNELS[4].adc.adc_reading,
				CHANNELS[5].adc.adc_reading,
				CHANNELS[6].adc.adc_reading,
				CHANNELS[7].adc.adc_reading,
				CHANNELS[8].adc.adc_reading,
				CHANNELS[9].adc.adc_reading);
		print_uart(huart, msg);
	}
}

/*
 * The function called during the ADC interrupt
 * @param hadc The ADC handle to the ADC
 * @param first_half If the first half of the buffer is filled
 */
void adc_interrupt(ADC_HandleTypeDef* hadc, boolean first_half) {
	U8 adc_input_rank = 0;
	boolean* adc_channels;
	U16RingBuffer* buf;
	if(hadc == hadc1_ptr) {
		adc_channels = ADC1_CHANNELS;
		if(first_half) {
			buf = &ADC1_RING_BUF_1;
		} else {
			buf = &ADC1_RING_BUF_2;
		}
	} else if (hadc == hadc3_ptr) {
		adc_channels = ADC3_CHANNELS;
		if(first_half) {
			buf = &ADC3_RING_BUF_1;
		} else {
			buf = &ADC3_RING_BUF_2;
		}
	}
	for(U8 i = 0; i < NUM_CHANNELS; i++) {
		if(adc_channels[i]) {
			channel_adc_interrupt(&CHANNELS[i], ring_buffer_ith_average(buf, adc_input_rank, NUM_CHANNELS));
			adc_input_rank++;
		}
	}
}

/*
 * Periodic function that runs and updates all channels
 */
void update_all_channels() {
	update_channels(CHANNELS, NUM_CHANNELS);
}

U16 get_battery_voltage() {
	return check_battery_voltage(CHANNELS);
}

U16 get_channel_supply_voltage(U8 id) {
	if(check_valid_channel(id)) {
		return check_channel_supply_voltage(CHANNELS, id);
	}
	return -1;
}

U16 get_channel_temperature(U8 id) {
	if(check_valid_channel(id)) {
		return check_channel_temperature(CHANNELS, id);
	}
	return -1;
}

U16 get_channel_current(U8 id) {
	if(check_valid_channel(id)) {
		return check_channel_current(CHANNELS, id);
	}
	return -1;
}

boolean get_channel_fault(Channel* channels, U8 id) {
	if(check_valid_channel(id)) {
		return check_channel_fault(CHANNELS, id);
	}
	return -1;
}

/*
 * Checks if a module is disabled due to brownout protection
 */
U8 check_brown_out(U8 id) {
	return 0;
}

/*
 * Returns a number representing how many modules are disabled due to brownout protection
 */
U8 check_brown_out_status() {
	return 0;
}

/*
 * Returns a boolean indicating if there is no load on the specified channel
 */
boolean get_channel_open_load(U8 id) {
	return check_channel_open_load(CHANNELS, id);
}

/*
 * Returns a boolean representing if a number is a valid channel
 */
boolean check_valid_channel(U8 id) {
	return  0 < id && id < NUM_CHANNELS;
}


