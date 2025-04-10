/*
 * random.h
 *
 *  Created on: Mar 27, 2025
 *      Author: jason
 */

#ifndef RANDOM_H_
#define RANDOM_H_


void signal_init();
void init_ADC1();
void init_Grid(void * sContextAdr, void * rectAdr);
void plot_data(void * sContextAdr, volatile uint16_t data[128], void * rectAdr);
int fifo_put(int data);
int fifo_get(int * data);
int Trigger(void);
void init_Measure(void * sContextAdr);
void init_CPU_Measure(void);
uint32_t cpu_load_count(void);
void Time_Scale(int tSet);
void init_Measure(void * sContextAdr);
void init_ADC_Timer(void);

#endif /* RANDOM_H_ */
