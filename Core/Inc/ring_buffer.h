/*
 * ring_buffer.h
 *
 *  Created on: Feb 9, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include "base_types.h"

// A struct representing a circular or ring buffer
typedef struct {
	U16 head;
	U16 size;
	U16 item_count;
	U32 sum;
	U16* array;
} U16RingBuffer;

U16RingBuffer* init_ring_buffer(U16 size);
void ring_buffer_append(U16RingBuffer* buffer, U16 element);
U16 ring_buffer_passive_average(U16RingBuffer* buffer);
U16 ring_buffer_entire_average(U16RingBuffer* buffer);
U16 ring_buffer_range_average(U16RingBuffer* buffer, U16 start, U16 stop);
U16 ring_buffer_get(U16RingBuffer* buffer, U16 index);
U16RingBuffer* ring_buffer_ith(U16RingBuffer* buffer, U16 offset, U16 step);
U16RingBuffer* ring_buffer_slice(U16RingBuffer* buffer, U16 start, U16 end);
U16RingBuffer* ring_buffer_first_half(U16RingBuffer* buffer);
U16RingBuffer* ring_buffer_second_half(U16RingBuffer* buffer);
void ring_buffer_clear(U16RingBuffer* buffer);
void free_ring_buffer(U16RingBuffer* buffer);

#endif /* INC_RING_BUFFER_H_ */
