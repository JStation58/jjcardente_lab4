/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>

#include "buttons.h" //Line added for the sake of the timer


//Definitions and Inclusions for Clock Signal Generation
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "random.h"
#include "driverlib/fpu.h"
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define LCD_VERTICAL_MAX 128
#define ADC_BUFFER_SIZE 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))
#define LCD_DIMENSION 128
#define ADC_OFFSET 2048
#define PIXELS_PER_DIV 20
#define ADC_BITS 12
#define VIN_RANGE 3.3

volatile float fVoltsPerDiv[5] = {0.1, 0.2, 0.5, 1, 2};

volatile uint16_t Data_Buffer[128];
volatile int16_t Scaled_Buffer[128];

tContext sContext;


extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];
extern volatile int32_t gADCBufferIndex;
volatile int triggerType = 0;
volatile int voltsPerDiv = 3;
float cpu_load = 0.0;
extern uint32_t count_unloaded;
uint32_t count_loaded = 0;
extern volatile int fifo_head;
extern volatile int fifo_tail;
volatile int tSet = 11;

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // hardware initialization goes here

 // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation


    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    ButtonInit(); //Initialize Buttons
    signal_init();
    init_CPU_Measure();
    init_ADC1();
    init_ADC_Timer();

    init_Grid(&sContext);
    init_Measure(&sContext);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void WaveformTask_func(UArg arg1, UArg arg2) {
    IntMasterEnable();
    Semaphore_post(CriticalSem);
    while (true) {
        Semaphore_pend(WaveformSem, BIOS_WAIT_FOREVER);
        int index = Trigger() - LCD_DIMENSION/2;
        int x;
        Semaphore_pend(CriticalSem, BIOS_WAIT_FOREVER);
        for (x = 0; x < LCD_VERTICAL_MAX; x++) {
            Data_Buffer[x] = gADCBuffer[ADC_BUFFER_WRAP(index + x)];
        }
        Semaphore_post(CriticalSem);
        Semaphore_post(ProcessingSem);
    }
}

void ProcessingTask_func(UArg arg1, UArg arg2) {
    //IntMasterEnable();

    while (true) {
        Semaphore_pend(ProcessingSem, BIOS_WAIT_FOREVER);
        float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[voltsPerDiv]);
        int x;
        Semaphore_pend(CriticalSem, BIOS_WAIT_FOREVER);
        for (x = 1; x < 128; x++) {
            Scaled_Buffer[x] = LCD_VERTICAL_MAX/2 - (int)roundf(fScale * ((int)(Data_Buffer[x]) - ADC_OFFSET));
            if (Scaled_Buffer[x] > LCD_VERTICAL_MAX - 1) {
                Scaled_Buffer[x] = LCD_VERTICAL_MAX - 1;
            } else if (Scaled_Buffer[x] < 0) {
                Scaled_Buffer[x] = 4;
            }
        }
        Semaphore_post(CriticalSem);
        Semaphore_post(DisplaySem);
        Semaphore_post(WaveformSem);
    }
}

void DisplayTask_func(UArg arg1, UArg arg2) {
    //IntMasterEnable();

    while (true) {
        Semaphore_pend(DisplaySem, BIOS_WAIT_FOREVER);
        Semaphore_pend(CriticalSem, BIOS_WAIT_FOREVER);
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
        plot_data(&sContext, Scaled_Buffer);
        Semaphore_post(CriticalSem);
        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

/*void clk_func(UArg arg1) {
    Semaphore_post(ButtonSem);
} */

void Button_Task(UArg arg1, UArg arg2) {
    while(true) {
        Semaphore_pend(ButtonSem, BIOS_WAIT_FOREVER);
        uint32_t gpio_buttons =
                    ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0); // EK-TM4C1294XL buttons in positions 0 and 1
                    gpio_buttons |= (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1))<< 1;
                    gpio_buttons |= (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6))>> 3;
                    gpio_buttons |= (~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4));

        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();
        char operation;

        if (presses & 2) { // EK-TM4C1294XL button 3 pressed
            operation = 'g';
            Mailbox_post(mailbox0, &operation, BIOS_WAIT_FOREVER);
        }

        if (presses & 4) { // EK-TM4C1294XL button 3 pressed
            operation = 'v';
            Mailbox_post(mailbox0, &operation, BIOS_WAIT_FOREVER);
        }

        if (presses & 8) { // EK-TM4C1294XL button 4 pressed
            operation = 't';
            Mailbox_post(mailbox0, &operation, BIOS_WAIT_FOREVER);
        }
    }
}

void User_Input(UArg arg1, UArg arg2) {
    while (true) {
        char operation;
        if (Mailbox_pend(mailbox0, &operation, BIOS_WAIT_FOREVER)) {
            Semaphore_pend(CriticalSem, BIOS_WAIT_FOREVER);
            if (operation == 'g') {
                triggerType = triggerType ^ 1;
            } else if (operation == 'v') {
                if (voltsPerDiv == 4) {
                    voltsPerDiv = 0;
                } else {
                    voltsPerDiv++;
                }
            } else if (operation == 't') {
                if (tSet == 11) {
                    tSet = 0;
                } else {
                    tSet++;
                }
            }
            Semaphore_post(CriticalSem);
        }
    }
}


void signal_init() {

    // configure M0PWM2, at GPIO PF2, BoosterPack 1 header C1 pin 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,
    GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}



int Trigger(void) { // search for rising edge trigger

    // Step 1
    int x = gADCBufferIndex - LCD_DIMENSION/2;

    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
        for (; x > x_stop; x--) {
            if (triggerType == 0) {
                if ( (gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET) && (gADCBuffer[ADC_BUFFER_WRAP(x - 1)] < ADC_OFFSET)) //Rising Trigger
                    break;
            } else if (triggerType == 1) {
                if ( (gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET) && (gADCBuffer[ADC_BUFFER_WRAP(x - 1)] > ADC_OFFSET)) //Falling Trigger
                    break;
            }
        }

    // Step 3
    if (x == x_stop) { // for loop ran to the end
        x = gADCBufferIndex - LCD_DIMENSION/2;; // reset x back to how it was initialized
    }
    return x;
}


