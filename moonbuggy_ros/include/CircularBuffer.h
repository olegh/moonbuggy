//
// Created by x0leg on 2/2/16.
//

#ifndef ROS_ADAPTER_CIRCULARBUFFER_H
#define ROS_ADAPTER_CIRCULARBUFFER_H

#include <stdint.h>
#include <stddef.h>

/* Opaque buffer element type.  This would be defined by the application. */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ElemType;

/* Circular buffer object */
typedef struct {
    size_t size;
    /* maximum number of elements           */
    volatile size_t start;
    /* index of oldest element              */
    volatile size_t end;
    /* index at which to write new element  */
    volatile size_t wr_counter;
    volatile size_t rw_counter;
    size_t neg_size;
    uint8_t *elems;  /* vector of elements                   */
} CircularBuffer;

#define cbGetLength( cb ) ((cb).wr_counter - (cb).rw_counter)

#define cbGetSpace( cb ) ((cb).size - ((cb).wr_counter - (cb).rw_counter))

#define cbGetReadPtr( cb ) (&(cb).elems[cb.start])

#define getWritePtr( cb ) (&(cb).elems[cb.end])

#define confirmRead( cb, readBytes ) do{ (cb).rw_counter += readBytes;\
                                         (cb).start = ((cb).start + readBytes) % (cb).size;}while(0)

#define confirmRead_if_size_pow_2( cb, readBytes ) do{ (cb).rw_counter += readBytes;\
                                   (cb).start = ((cb).start + readBytes) & (cb).neg_size;}while(0)

#define confirmWrite( cb, writeBytes ) do{ (cb).wr_counter += writeBytes;\
                                            (cb).end = ((cb).end + writeBytes) % (cb).size;}while(0)

#define confirmWrite_if_size_pow_2( cb, writeBytes ) do{ (cb).wr_counter += writeBytes;\
                                    (cb).end = ((cb).end + writeBytes) & (cb).neg_size;}while(0)

#define cbInit(cb, vector, size_) do{ \
    (cb).size = (size_); \
    (cb).wr_counter = 0; \
    (cb).rw_counter = 0; \
    (cb).elems = (vector); \
    (cb).start = 0; \
    (cb).end = 0; \
    (cb).neg_size = ~(size_); \
}while(0)

#endif //ROS_ADAPTER_CIRCULARBUFFER_H
