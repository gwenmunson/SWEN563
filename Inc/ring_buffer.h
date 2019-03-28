/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ring_buffer_H
#define __ring_buffer_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"
#include "stdbool.h"

#define RB_SIZE 25
typedef struct {
	int inp;	// input index to ring buffer
	int outp;	// output index to ring buffer
	unsigned int buf[RB_SIZE];	// the buffer of things 
} RING_BUFFER_t;

bool rb_full(RING_BUFFER_t *rb);
bool rb_empty(RING_BUFFER_t *rb);
bool rb_add(RING_BUFFER_t *rb, unsigned int ch);
bool rb_push(RING_BUFFER_t *rb, unsigned int item);
unsigned int rb_remove(RING_BUFFER_t *rb);
unsigned int rb_peek(RING_BUFFER_t *rb);
unsigned int rb_length(RING_BUFFER_t *rb);

#endif /* __ring_buffer_H */
