/*
 * ring_buffer.c
 *
 *  Created on: Feb 9, 2022
 *      Author: Ben Abbott
 */

#include "ring_buffer.h"
#include "freertos.h"

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
 * Initializes a new ring buffer at a given memory address
 * @param size The size of the the new buffer
 * @param buffer A pointer to the buffer to initialize
 * @param array A pointer to the memory address to the buffer array
 */
void init_ring_buffer_static(U16 size, U16RingBuffer* buffer, volatile U16* array) {
	buffer->head = 0;
	buffer->item_count = 0;
	buffer->size = size;
	buffer->sum = 0;
	buffer->array = array;
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
 * Appends all elements in one ring buffer to the end of another ring buffer
 *
 * @param buffer1 The buffer to append the items to that will be modified
 * @param buffer2 The buffer from which the items will be appended
 */
void ring_buffer_append_buffer(U16RingBuffer* buffer1, U16RingBuffer* buffer2) {
	for(U16 i = 0; i < buffer2->size; i++) {
		ring_buffer_append(buffer1, ring_buffer_get(buffer2, i));
	}
}

/**
 * Calculates the average assuming the internal array of the ring
 * buffer has not been manually modified (runs in O(1))
 *
 * @param buffer The buffer on which to calculate the average
 * @return An integer representing the average of entries in the buffer, 0 if empty
 */
U16 ring_buffer_passive_average(U16RingBuffer* buffer) {
	if(buffer->item_count == 0) {
		return 0;
	}
	return (buffer->sum / buffer->item_count);
}

/*
 * Calculates the average of a ring buffer.  Works even if the internal array
 * of the buffer has been modified manually (runs in O(n)).
 *
 * @param buffer The buffer on which to calculate the average
 * @return The average of the entire array in teh buffer
 * @note This function will calculate the average on the entire buffer,
 * meaning the entire array of the buffer, whether or not it has been filled
 * using ring_buffer_append.
 */
U16 ring_buffer_entire_average(U16RingBuffer* buffer) {
	return ring_buffer_range_average(buffer, 0, buffer->size);
}


/**
 * Averages the first half of a buffer (inclusive of the middle element)
 *
 * @param buffer The buffer to average
 * @return The average of the first half of the specified buffer
 */
U16 ring_buffer_first_half_average(U16RingBuffer* buffer) {
	return ring_buffer_range_average(buffer, 0, (buffer->size/2) + (buffer->size % 2));
}

/**
 * Averages the second half of a buffer (exclusive of the middle element)
 *
 * @param buffer The buffer to average
 * @return The average of the second half of the specified buffer
 */
U16 ring_buffer_second_half_average(U16RingBuffer* buffer) {
	return ring_buffer_range_average(buffer, (buffer->size/2) + (buffer->size % 2), buffer->size);
}

/**
 * Averages every i-th element in the buffer, starting at the i-th element
 *
 * @param buffer The buffer to average
 * @param offset The index of the first item to average
 * @param step The amount ot step each iteration
 *
 */
U16 ring_buffer_ith_average(U16RingBuffer* buffer, U16 offset, U16 step) {
	U32 sum = 0;
	U16 count = 0;
	for(U16 i = offset; i < buffer->size; i += step) {
		sum += ring_buffer_get(buffer, i);
		count++;
	}
	return sum /= count;
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
 * Retrieves an element in a ring buffer
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
	U16 new_buffer_size = (((buffer->size - offset) + step - 1)/step);
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
 * Clears a ring buffer, removing all elements.  This method does not free the
 * ring buffer from memory, so it is recommended to utilize free_ring_buffer
 * to remove any ring buffers from memory that are no longer needed.
 *
 * @param buffer The buffer to clear
 */
void ring_buffer_clear(U16RingBuffer* buffer) {
	buffer->item_count = 0;
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
