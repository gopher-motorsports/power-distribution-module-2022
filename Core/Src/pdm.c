/*
 * pdm.c
 *
 *  Created on: Jan 16, 2022
 *      Author: Ben Abbott
 */

#include "pdm.h"
#include "main.h"
#include "config.h"
#include "channel.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// TODO: Add volatile modifier to DMA ADC array
// TODO: Look for maxed out ADC in channel interrupt to detect faults that overlap
//       between different ticks
// TODO: Create 3 overlapping circular buffers with 1 main buffer and two
//       other buffers that point to the first and second half of the buffer
//       to create a ping pong buffer
// TODO: Verify and update all documentation
// TODO: Statically allocate memory and delete free methods
// TODO: Deal with integrator logic when hardware fault triggers
// TODO: Detect external overrides of manually controlled pins (i.e. fuel pump, spark, etc.)
// TODO: Try to increase ADC sample time to decrease noise instead of filtering so much
// TODO: Document functions/stuff
// TODO: Add more header files and update/verify header files and prototypes
// TODO: Add malloc null checks and fault handling
// TODO: Implement channel utility functions/structs
// TODO: Implement software overcurrent protection
// TODO: Implement hardware timer functionality for diagnostic state switching
//       (discarding old ADC measurements) and also have some sort of queue
// TODO: Implement fault transmission over CAN to driver display
// TODO: Implement cooling model
// TODO: Implement board condition sensing (I2C)
// TODO: Implement processor temperature sensing via internal sensor
// TODO: Determine error codes on LED
// TODO: Figure out LED pins and configure
// TODO: Implement logging
// TODO: Think more about fault detection (and monitoring methods)
// TODO: Detect switch fault if voltage is too low on switch input
// TODO: Detect switch fault if temp is too high
// TODO: Hardware open-load on startup and whenever turning on channels
// TODO: Implement software and hardware open-load protection
// TODO: Write data filter algorithm
// TODO: Go over header files and prune code

U16RingBuffer* init_adc_dma(ADC_HandleTypeDef* hadc, U16 buffer_size) {
	U16RingBuffer* adc_buf = init_ring_buffer(buffer_size);
	if(adc_buf->array == NULL) {
		return NULL;
	}
	HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buf->array, buffer_size);
	return adc_buf;
}

void print_uart(UART_HandleTypeDef huart, char* msg) {
	U8 msg_buf[strlen(msg)+1];
	strcpy((char*)msg_buf, msg);
	HAL_UART_Transmit(&huart, msg_buf, strlen((char*)msg_buf), HAL_MAX_DELAY);
}

void init_pdm() {
}

// TODO: Rename
void main_loop(UART_HandleTypeDef huart3, ADC_HandleTypeDef hadc1) {
	osDelay(500);
}

/*
 * Periodic function that runs and updates all safety/protection systems
 * Returns a boolean representing if there's a problem
 */
boolean update_all_channels() {
	return 0;
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
 * This function filters the current of a channel after adding a new measurement to it
 */
U16 filter_current(U16RingBuffer* buffer, U16 measurement) {
	ring_buffer_append(buffer, measurement);
	return ring_buffer_passive_average(buffer);
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
 * Returns a boolean indicating if there is an open load on the specified channel
 */
boolean check_open_load(U8 channel) {
	return 0;
}

/*
 * Returns a boolean representing if a number is a valid channel
 */
boolean check_valid_channel(U8 num) {
	return 0;
}


