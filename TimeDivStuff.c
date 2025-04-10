/*
 * TimeDivStuff.c
 *
 *  Created on: Apr 9, 2025
 *      Author: jason
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "buttons.h"
#include "inc/tm4c1294ncpdt.h"

extern uint32_t gSystemClock;
float tscales[11] = {5e-3, 2.5e-3, 1e-3, 500e-6, 250e-6, 100e-6, 50e-6, 25e-6, 10e-6, 5e-6, 2.5e-6}; //FILL IN WITH MAX COUNTS FOR EVERY SETTING LATER


void init_ADC_Timer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerDisable(TIMER2_BASE, TIMER_BOTH);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, tscales[10] * 120000000);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
    TimerEnable(TIMER2_BASE, TIMER_BOTH);

}

void Time_Scale(volatile int tSet) {

    if (tSet == 11) {
        ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    } else {
        ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0);
        TimerDisable(TIMER2_BASE, TIMER_BOTH);
        TimerLoadSet(TIMER2_BASE, TIMER_A, tscales[tSet] * 120000000);
        TimerEnable(TIMER2_BASE, TIMER_BOTH);
    }
    ADCSequenceEnable(ADC1_BASE, 0);

}


//TimerControlTrigger() can be used to set the timer as the trigger for the ADC



/* SOME BRIEF MATHEMATICS
 *
 * 20 pixels per division, meaning 20 samples per division
 *
 *
 * 100 ms / 20 Samples = 1 sample every 5 ms
 * 50 ms / 20 samples = 1 sample every 2.5 ms
 * 20 ms / 20 samples = 1 sample per ms
 * 10 ms / 20 samples = 1 sample every 500 us
 * 5 ms / 20 samples = 1 sample every 250 us
 * 2 ms / 20 samples = 1 sample every 100 us
 * 1 ms / 20 samples = 1 sample every 50 us
 * 500 us / 20 samples = 1 sample every 25 us
 * 200 us / 20 samples = 1 sample every 10 us
 * 100 us / 20 samples = 1 sample every 5 us
 * 50 us / 20 samples = 1 sample every 2.5 us
 * This is just a further test
 *
 * clock frequency / (1 / these times) = Max count
 * clock frequency = 120 MHz
 *
 * */
