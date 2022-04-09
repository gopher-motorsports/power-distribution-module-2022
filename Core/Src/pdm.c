/*
 * pdm.c
 *
 *  Created on: Jan 16, 2022
 *      Author: Ben Abbott
 */

#include "pdm.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "cmsis_os.h"
#include "stalloc.h"
#include "channel.h"
#include "channel_control.h"

// TODO: Create 3 overlapping circular buffers with 1 main buffer and two
//       other buffers that point to the first and second half of the buffer
//       to create a ping pong buffer
// TODO: Verify and update all documentation
// TODO: Detect external overrides of manually controlled pins (i.e. fuel pump, spark, etc.)
// TODO: Try to increase ADC sample time to decrease noise instead of filtering so much
// TODO: Add more header files and update/verify header files and prototypes
// TODO: Implement hardware timer functionality for diagnostic state switching
//       (discarding old ADC measurements) and also have some sort of queue
// TODO: Implement fault transmission over CAN to driver display
// TODO: Implement board condition sensing (I2C)?
// TODO: Implement processor temperature sensing via internal sensor?
// TODO: Determine error codes on LED
// TODO: Figure out LED pins and configure
// TODO: Implement logging
// TODO: Go over all header files and prune code

extern Channel CHANNELS[];
extern U16 ADC1_BUF[];
extern U16 ADC3_BUF[];

DiagnosticState diagnostic_state;

void init_pdm(TIM_HandleTypeDef* tim, ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc3) {
	init_channels(CHANNELS, &diagnostic_state, &tim->Instance->CNT);
	init_adc_dma(hadc1, ADC1_BUF, ADC1_BUF_LEN);
	init_adc_dma(hadc3, ADC3_BUF, ADC3_BUF_LEN);
}

void init_adc_dma(ADC_HandleTypeDef* hadc, U16 buffer[], U16 buffer_size) {
	HAL_ADC_Start_DMA(hadc, (uint32_t*)buffer, buffer_size);
}

void print_uart(UART_HandleTypeDef huart, char* msg) {
	U8 msg_buf[strlen(msg)+1];
	strcpy((char*)msg_buf, msg);
	HAL_UART_Transmit(&huart, msg_buf, strlen((char*)msg_buf), HAL_MAX_DELAY);
}

void main_loop() {
	// TODO: ADd logic to switch to voltage/temp state every so often
	update_all_channels();
}

/*
 * Periodic function that runs and updates all channels
 */
void update_all_channels() {
	update_channels(CHANNELS, NUM_CHANNELS);
}

/*
 * Returns an enum representing the state of the fuse
 */
FuseState check_fuse(U8 channel) {
	return 0;
}

/*
 * Updates the fuse on a specified channel
 */
void update_fuse(U8 channel) {

}

/*
 * Returns a boolean representing if there is a hardware fault on that channel
 */
boolean check_hw_fault(U8 channel) {
	return 0;
}

/*
 *
 *
 */
U8 check_brown_out(U8 channel) {
	return 0;
}

/*
 * A periodic function called that attempts to prevent brownouts on the PDM
 */
void brown_out_prevention(double current) {

}

/*
 * Returns a number representing how many modules are disabled due to brownout protection
 */
U8 check_brown_out_status() {
	return 0;
}

/*
 * A periodic function called that attempts to detect open loads on the PDM
 */
void open_load_protection(U8 channel) {

}

/*
 * Returns a boolean indicating if there is no load on the specified channel
 */
boolean check_open_load(U8 channel) {
	return 0;
}

/*
 * Returns a boolean representing if a number is a valid channel
 */
boolean check_valid_channel(U8 num) {
	return  0 < num && num < NUM_CHANNELS;
}


