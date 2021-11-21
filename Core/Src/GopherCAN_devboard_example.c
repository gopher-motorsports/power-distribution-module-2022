// GopherCAN_devboard_example.c
//  TODO DOCS - when the project becomes more fleshed out add a quick comment
//  explaining the purpose of this file

#include "GopherCAN_devboard_example.h"
#include "main.h"

// the HAL_CAN struct. This example only works for a single CAN bus
CAN_HandleTypeDef* example_hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID PDM_ID


// some global variables for examples
U8 last_button_state = 0;


// the CAN callback function used in this example
static void change_led_state(U8 sender, void* UNUSED_LOCAL_PARAM, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3);


// init
//  What needs to happen on startup in order to run GopherCAN
void init(CAN_HandleTypeDef* hcan_ptr)
{
	example_hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(example_hcan, THIS_MODULE_ID, BXTYPE_MASTER))
	{
		// an error has occurred, stay here
		while (1);
	}

	// enable all of the variables in GopherCAN for testing
	set_all_params_state(TRUE);

	// Set the function pointer of SET_LED_STATE. This means the function change_led_state()
	// will be run whenever this can command is sent to the module
	if (add_custom_can_func(SET_LED_STATE, &change_led_state, TRUE, NULL))
	{
		// an error has occurred
		while (1);
	}
}


// can_buffer_handling_loop
//  This loop will handle CAN RX software task and CAN TX hardware task. Should be
//  called every 1ms or as often as received messages should be handled
void can_buffer_handling_loop()
{
	// handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// an error has occurred
	}

	// handle the transmission hardware for each CAN bus
	service_can_tx_hardware(example_hcan);
}


// main_loop
//  another loop. This includes logic for sending a CAN command. Designed to be
//  called every 10ms
void main_loop()
{
	U8 button_state;

	// If the button is pressed send a can command to another to change the LED state
	// To on or off depending on the button
	button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

	// Logic to only send one message per change in button state
	if (button_state != last_button_state)
	{
		last_button_state = button_state;

		if (send_can_command(PRIO_HIGH, ALL_MODULES_ID, SET_LED_STATE,
				button_state, button_state, button_state, button_state))
		{
			// error sending command
		}
	}
}


// can_callback_function example

// change_led_state
//  a custom function that will change the state of the LED specified
//  by parameter to remote_param. In this case parameter is a U16*, but
//  any data type can be pointed to, as long as it is configured and casted
//  correctly
static void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
{
	HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, !!remote_param);
	return;
}

// end of GopherCAN_example.c
