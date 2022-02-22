/*
 * pdm.h
 *
 *  Created on: Feb 8, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_PDM_H_
#define INC_PDM_H_

#include "config.h"
#include "channel.h"
#include "ring_buffer.h"

U16RingBuffer* init_adc_dma(ADC_HandleTypeDef* hadc1, U16 buffer_size);
void init_pdm();
void main_loop();
boolean update_all_channels();
FuseState check_fuse(U8 channel);
void update_fuse(U8 channel);
boolean check_hw_fault(U8 channel);
U8 check_brown_out(U8 channel);
U16 filter_current(U16RingBuffer* buffer, U16 measurement);
void brown_out_prevention(double current);
U8 check_brown_out_status();
void open_load_protection(U8 channel);
boolean check_open_load(U8 channel);
boolean check_valid_channel(U8 num);

#endif /* INC_PDM_H_ */
