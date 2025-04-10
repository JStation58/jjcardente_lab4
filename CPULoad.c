/*
 * CPULoad.c
 *
 *  Created on: Apr 7, 2025
 *      Author: jason
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

uint32_t count_unloaded = 0;

void init_CPU_Measure(void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
      TimerDisable(TIMER3_BASE, TIMER_BOTH);
      TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
      TimerLoadSet(TIMER3_BASE, TIMER_A, 1200000); // 1 sec interval
      count_unloaded = cpu_load_count();

}


uint32_t cpu_load_count(void) {
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}


