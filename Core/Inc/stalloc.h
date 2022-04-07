/*
 * stalloc.h
 *
 *  Created on: Mar 17, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_STALLOC_H_
#define INC_STALLOC_H_

#include "base_types.h"

// ==============================
// || STATIC MEMORY ALLOCATION ||
// ==============================

volatile U16 ADC1_BUF[ADC1_BUF_LEN];
volatile U16 ADC3_BUF[ADC3_BUF_LEN];
Channel CHANNELS[NUM_CHANNELS];

#endif /* INC_STALLOC_H_ */
