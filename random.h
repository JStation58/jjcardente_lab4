/*
 * random.h
 *
 *  Created on: Mar 27, 2025
 *      Author: jason
 */

#ifndef RANDOM_H_
#define RANDOM_H_

#include <stdint.h>
#include <stdbool.h>
#include "grlib/grlib.h"

void signal_init();
void init_ADC1();
void init_Grid(tContext * sContextAdr);
void plot_data(tContext * sContextAdr, volatile int16_t data[128]);
int fifo_put(int data);
int fifo_get(int * data);
int Trigger(void);
void init_Measure(tContext * sContextAdr);
void init_CPU_Measure(void);
uint32_t cpu_load_count(void);

void Time_Scale(int tSet);
void init_ADC_Timer(void);
void init_DMA(void);
int32_t getADCBufferIndex(void);

#endif /* RANDOM_H_ */
