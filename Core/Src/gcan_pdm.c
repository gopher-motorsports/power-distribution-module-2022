/*
 * gcan_pdm.c
 *
 *  Created on: Feb 8, 2022
 *      Author: Ben Abbott
 */


#include "gcan_pdm.h"
#include "config.h"
#include "GopherCAN.h"


// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

/**
 * GopherCAN Initialization
 */
void init_gcan(CAN_HandleTypeDef* hcan_ptr)
{

	hcan = hcan_ptr;

	// Initialize CAN
	if (init_can(hcan, GCAN_MODULE_ID, BXTYPE_MASTER))
	{
		// An error has occurred; stay here
		// TODO: Handle error
		while (1);
	}

	// Enable all of the variables in GopherCAN for testing
	set_all_params_state(TRUE);

//	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
//	// will be run whenever this can command is sent to the module
//	if (add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL))
//	{
//		// an error has occurred
//		while (1);
//	}
}

/**
 * This loop will handle CAN RX software task and CAN TX hardware task. Should be
 * called every 1ms or as often as received messages should be handled
 */
void gcan_process_buffer()
{
	// Handle each RX message in the buffer
	service_can_rx_buffer();

	// Handle the transmission hardware for each CAN bus
	service_can_tx_hardware(hcan);
}
