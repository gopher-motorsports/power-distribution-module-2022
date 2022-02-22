/*
 * gcan_pdm.h
 *
 *  Created on: Feb 8, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_GCAN_PDM_H_
#define INC_GCAN_PDM_H_

#include "GopherCAN.h"

void init_gcan(CAN_HandleTypeDef* hcan_ptr);
void gcan_process_buffer();

#endif /* INC_GCAN_PDM_H_ */
