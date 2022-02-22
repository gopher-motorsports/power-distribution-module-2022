/*
 * logging.h
 *
 *  Created on: Jan 16, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include "GopherCAN.h"

U16 log_supply_voltage(U8 channel);
U16 log_temperature(U8 channel);
U16 log_current(U8 channel);
U16 log_fuse_state(U8 channel);
U16 log_hw_fault(U8 channel);
U16 log_open_load(U8 channel);
U16 log_board_temp();
U16 log_board_humidity();
U16 log_board_accel();

#endif /* INC_LOGGING_H_ */
