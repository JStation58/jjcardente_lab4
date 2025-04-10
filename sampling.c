/*
 * sampling.c
 *
 *  Created on: Mar 27, 2025
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

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
// index wrapping macro
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
// latest sample index
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines

void init_ADC1(){
    // initialize ADC1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
    // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    // choose ADC1 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger
    //NOTE: ADC_TRIGGER_TIMER specifies the trigger from a timer
    // in the 0th step, sample channel 3 (AIN3)
    // enable interrupt, and make it the end of sequence
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    // enable the sequence. it is now sampling
    ADCSequenceEnable(ADC1_BASE, 0);
    // enable sequence 0 interrupt in the ADC1 peripheral
    ADCIntEnable(ADC1_BASE, 0);
    IntPrioritySet(INT_ADC1SS0,0); // set ADC1 sequence 0 interrupt priority
    // enable ADC1 sequence 0 interrupt in int. controller
    IntEnable(INT_ADC1SS0);

}

void ADC_ISR(void) {

    // clear ADC1 sequence0 interrupt flag in the ADCISC register
    ADC1_ISC_R |= 1;
    // check for ADC FIFO overflow
    if(ADC1_OSTAT_R & ADC_OSTAT_OV0) {
        gADCErrors++; // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
    }
    gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1);
    // read sample from the ADC1 sequence 0 FIFO
    gADCBuffer[gADCBufferIndex] = ADC1_SSFIFO0_R;

}




