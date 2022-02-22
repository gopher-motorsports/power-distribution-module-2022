/*
 * ring_buffer.c
 *
 *  Created on: Feb 9, 2022
 *      Author: Ben Abbott
 */

#include "ring_buffer.h"

/**
 * Allocates and initializes a new ring buffer
 * @param size The size of the the new buffer
 */
U16RingBuffer* init_ring_buffer(U16 size) {
	U16RingBuffer* buffer = pvPortMalloc(sizeof(U16RingBuffer));
	buffer->head = 0;
	buffer->item_count = 0;
	buffer->size = size;
	buffer->sum = 0;
	buffer->array = pvPortMalloc(sizeof(U16)*size);
	return buffer;
}

/**
 * Appends a new element to a ring buffer
 *
 * @param buffer The buffer to append the new item to
 * @param element The element to append
 */
void ring_buffer_append(U16RingBuffer* buffer, U16 element) {
	if(buffer->item_count < buffer->size) {
		buffer->array[buffer->item_count] = element;
		buffer->sum += element;
		buffer->item_count++;
	} else {
		buffer->sum -= buffer->array[buffer->head];
		buffer->sum += element;
		buffer->array[buffer->head] = element;
		buffer->head++;
		buffer->head %= buffer->size;
	}
}

/**
 * Calculates the average assuming the internal array of the ring
 * buffer has not been manually modified (runs in O(1))
 *
 * @param buffer The buffer on which to calculate the average
 * @return An integer representing the average of entries in the buffer
 */
U16 ring_buffer_passive_average(U16RingBuffer* buffer) {
	return (buffer->sum / buffer->item_count);
}

/*
 * Calculates the average of a ring buffer.  Works even if the internal array
 * of the buffer has been modified manually (runs in O(n)).
 *
 * @param buffer The buffer on which to calculate the average
 * @return The average of the entire array in teh buffer
 * @note This function will calculate the average on the entire buffer,
 * meainng the entire array of the buffer, whether or not it has been filled
 * using ring_buffer_append.
 */
U16 ring_buffer_entire_average(U16RingBuffer* buffer) {
	return ring_buffer_range_average(buffer, 0, buffer->size);
}

/**
 * Calculates the average of a ring buffer over a specified range.  Works even
 * if the internal array of the buffer has been modified manually (runs in O(n)).
 *
 * @param buffer The buffer on which to calculate the average
 * @param start The start of the range on which to calculate the average
 * @param stop The end of the range on which to calculate the average
 * @return The average value in the specified range in the ring buffer
 */
U16 ring_buffer_range_average(U16RingBuffer* buffer, U16 start, U16 stop) {
	U32 sum = 0;
	for(U16 i = start; i < stop; i++) {
		sum += ring_buffer_get(buffer, i);
	}
	return (sum / buffer->size);
}

/**
 * Retrivies an element in a ring buffer
 *
 * @param buffer The relevant buffer
 * @param index The index of the item to retrieve
 * @return The item at the specified index
 */
U16 ring_buffer_get(U16RingBuffer* buffer, U16 index) {
	return buffer->array[(index + buffer->head) % buffer->size];
}

/**
 * Creates a new ring buffer with every i-th element of the provided ring
 * buffer.  For example, create a new ring buffer with every 2nd or every 3rd
 * element of another ring buffer.  In addition, an offset can be specified.
 * The copying of elements will start at that index, so if every 2nd element
 * is requested with an offset of 5, the 5th, 7th, 9th,... elements will be
 * in the new ring buffer.
 *
 * @param buffer The provided buffer
 * @param offset The index of the first item to be in the new buffer
 * @param step The difference in indices between any two consecutive elements
 * in the new buffer
 * @return A newly allocated ring buffer with the aforementioned properties
 */
U16RingBuffer* ring_buffer_ith(U16RingBuffer* buffer, U16 offset, U16 step) {
	U16 new_buffer_size = (buffer->size - offset)/step;
	if(new_buffer_size < 0) {
		return 0;
	}
	U16RingBuffer* new_buffer = init_ring_buffer(new_buffer_size);
	for(U16 i = offset; i < buffer->size; i += step) {
		ring_buffer_append(new_buffer, ring_buffer_get(buffer, i));
	}
	return new_buffer;
}


/**
 * Slices a buffer (creates a new buffer that is a subset of another buffer)
 * based on the start and end indices given.  The given range excludes the
 * ending index, so if the sliced buffer should include the last element in
 * the original buffer, it's ending index should be its size.
 *
 * @param buffer The buffer to create a subset of
 * @param start The index at which to begin the new buffer
 * @param end The index at which to end the new buffer
 * @return A sliced buffer
 */
U16RingBuffer* ring_buffer_slice(U16RingBuffer* buffer, U16 start, U16 end) {
	if(end - start < 0) {
		return 0;
	}
	U16RingBuffer* new_buffer = init_ring_buffer(end-start);
	for(U16 i = start; i < end; i++) {
		ring_buffer_append(new_buffer, ring_buffer_get(buffer, i));
	}
	return new_buffer;
}


/**
 * Slices the first half of a buffer (inclusive of the middle element)
 *
 * @param buffer The buffer to slice
 * @ return The first half of the specified buffer
 */
U16RingBuffer* ring_buffer_first_half(U16RingBuffer* buffer) {
	return ring_buffer_slice(buffer, 0, (buffer->size/2) + (buffer->size % 2));
}

/**
 * Slices the second half of a buffer (exclusive of the middle element)
 *
 * @param buffer The buffer to slice
 * @ return The first half of the specified buffer
 */
U16RingBuffer* ring_buffer_second_half(U16RingBuffer* buffer) {
	return ring_buffer_slice(buffer, (buffer->size/2) + (buffer->size % 2), buffer->size);
}


/**
 * Clears a ring buffer, removing all elements.  This method does not free the
 * ring buffer from memory, so it is recommended to utilize free_ring_buffer
 * to remove any ring buffers from memory that are no longer needed.
 *
 * @param buffer The buffer to clear
 */
void ring_buffer_clear(U16RingBuffer* buffer) {
	for(U16 i = 0; i < buffer->item_count; i++) {
		buffer->array[i] = 0;
	}
	buffer->item_count = 0;
	buffer->head = 0;
	buffer->sum = 0;
}

/**
 * Frees a ring buffer from memory
 *
 * @param buffer The buffer to free
 */
void free_ring_buffer(U16RingBuffer* buffer) {
	vPortFree(buffer->array);
	vPortFree(buffer);
}
