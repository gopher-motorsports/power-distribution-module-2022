/*
 * channel.c
 *
 *  Created on: Feb 12, 2022
 *      Author: Ben Abbott
 */

#include "channel.h"
#include <stdlib.h>

Channel* init_channel() {
	Channel* channel = (Channel*)pvPortMalloc(sizeof(Channel));
	channel->state = DEFAULT_FUSE_STATE;
	channel->hw_fault = FALSE;
	channel->brn_out_disabled = FALSE;
	channel->retries = 0;
	channel->recent_retries = 0;
	channel->max_cont_current = 0;
	channel->primary_adc_buffer = init_ring_buffer(CHANNEL_ADC_BUFFER_SIZE);
	channel->primary_current_buffer = init_ring_buffer(PRIMARY_CURRENT_BUFFER_SIZE);
	channel->secondary_current_buffer = init_ring_buffer((int)FUSE_INTEGRATION_LENGTH_MS/(int)FUSE_SAMPLE_PERIOD_MS);
}

FuseState update_channel(Channel* channel, U16 current) {

}

void set_channel(boolean on) {

}


void free_channel(Channel* channel) {

}
