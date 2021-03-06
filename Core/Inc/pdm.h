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

void init_adc_dma(ADC_HandleTypeDef* hadc, volatile U16 buffer[], U16 buffer_size);
void init_pdm(TIM_HandleTypeDef* tim_ptr, ADC_HandleTypeDef* adc1_ptr, ADC_HandleTypeDef* adc3_ptr, UART_HandleTypeDef* huart_ptr);
void init_leds();
void set_all_leds(boolean fault, boolean status, boolean adc);
void main_loop();
void adc_interrupt(ADC_HandleTypeDef* hadc, boolean first_half);
void update_all_channels();
FuseState check_fuse(U8 channel);
void update_fuse(U8 channel);
void print_uart(UART_HandleTypeDef* huart, char* msg);
boolean check_hw_fault(U8 channel);
U8 check_brown_out(U8 channel);
U16 filter_current(U16RingBuffer* buffer, U16 measurement);
void brown_out_prevention(double current);
U8 check_brown_out_status();
void open_load_protection(U8 channel);
boolean check_open_load(U8 channel);
boolean check_valid_channel(U8 num);

#endif /* INC_PDM_H_ */
