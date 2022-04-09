/*
 * stalloc.c
 *
 *  Created on: Apr 9, 2022
 *      Author: Ben Abbott
 */

#include "stalloc.h"
#include "channel.h"

// ==============================
// || STATIC MEMORY ALLOCATION ||
// ==============================

volatile U16 ADC1_BUF[ADC1_BUF_LEN*2];
volatile U16 ADC3_BUF[ADC3_BUF_LEN*2];
Channel CHANNELS[NUM_CHANNELS];
