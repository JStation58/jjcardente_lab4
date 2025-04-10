/*
 * graph.c
 *
 *  Created on: Mar 27, 2025
 *      Author: jason
 */


#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "random.h"

#define PIXELS_PER_DIV 20
#define Y_MID 64
#define X_MID 64
#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define ADC_OFFSET 2048
#define ADC_BITS 12
#define VIN_RANGE 3.3

volatile float fVoltsPerDiv[5] = {0.1, 0.2, 0.5, 1, 2};
const char * const gVoltageScaleStr[5] = {"100 mV", "200 mV", "500 mV", "1 V", "2 V"};
const char * const gTimeScaleStr[12] = {"100 ms", "50 ms", "20 ms", "10 ms", "5 ms", "2 ms", "1 ms", "500 us", "200 us", "100 us", "50 us", "20 us"};
extern volatile int voltsPerDiv;
extern volatile int triggerType;
extern float cpu_load;
extern volatile int tSet;

void init_Grid(void * sContextAdr, void * rectAdr) {

    GrContextForegroundSet(sContextAdr, ClrBlack);
    GrRectFill(sContextAdr, rectAdr); // fill screen with black
    GrContextForegroundSet(sContextAdr , ClrBlue);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID - PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID - 2 * PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID - 3 * PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID - 4 * PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID + PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID + 2 * PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID + 3 * PIXELS_PER_DIV);
    GrLineDrawH(sContextAdr, 0, 128, Y_MID + 4 * PIXELS_PER_DIV);
    GrLineDrawV(sContextAdr, X_MID, 0, 128);
    GrLineDrawV(sContextAdr, X_MID - PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID - 2 * PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID - 3 * PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID - 4 * PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID + PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID + 2 * PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID + 3 * PIXELS_PER_DIV, 0, 128);
    GrLineDrawV(sContextAdr, X_MID + 4 * PIXELS_PER_DIV, 0, 128);

}


void plot_data(void * sContextAdr, volatile uint16_t data[128], void * rectAdr) {

    init_Grid(sContextAdr, rectAdr);
    GrContextForegroundSet(sContextAdr, ClrYellow);
    float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[voltsPerDiv]);
    int x;
    int y[2] = {LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(data[0]) - ADC_OFFSET)) ,0};
    if (y[0] > LCD_VERTICAL_MAX) {
        y[0] = LCD_VERTICAL_MAX - 1;
    } else if (y[0] < 0) {
        y[0] = 0;
    }
    for (x = 1; x < 128; x++) {
        y[1] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(data[x]) - ADC_OFFSET));
        if (y[0] > LCD_VERTICAL_MAX) {
           y[0] = LCD_VERTICAL_MAX - 1;
        } else if (y[0] < 0) {
           y[0] = 0;
        }
        GrLineDraw(sContextAdr, (x-1), (y[0]), (x), (y[1]));
        y[0] = y[1];
    }
    init_Measure(sContextAdr);
}

void init_Measure(void * sContextAdr) {

    //Draw Trigger Mode
    GrContextForegroundSet(sContextAdr , ClrWhite);
    if (triggerType == 0) {
        GrStringDraw(sContextAdr, "/ Trigger", 25, 5, 107, 1);
    } else if (triggerType == 1) {
        GrStringDraw(sContextAdr, "\\ Trigger", 25, 5, 107, 1);
    }

    //Draw Volts Per Division
    GrStringDraw(sContextAdr, gVoltageScaleStr[voltsPerDiv] , 25, 15, 10, 1);

    //Draw Volts Per Division
    GrStringDraw(sContextAdr, gTimeScaleStr[tSet] , 25, 80, 10, 1);

    //Display CPU Load
    char cpuMessage[50];
    sprintf(cpuMessage, "CPU Load: %.2f%%", (cpu_load * 100));
    GrStringDraw(sContextAdr, cpuMessage , 25, 5, 117, 1);
}


