#include <stdint.h>

int xscale = 0;
int yscale = 0;

#define FIFO_SIZE 11        // fifot capacity is 1 item fewer
volatile uint32_t fifo[FIFO_SIZE];  // fifo storage array
volatile int fifo_head = 0; // index of the first item in the fifo
volatile int fifo_tail = 0; // index one step past the last item


// put data into the FIFO, skip if full
// returns 1 on success, 0 if FIFO was full
int fifo_put(int data) {

    int new_tail = fifo_tail + 1;
    if (new_tail >= FIFO_SIZE) new_tail = 0; // wrap around
    if (fifo_head != new_tail) {    // if the FIFO is not full
        fifo[fifo_tail] = data;     // store data into the FIFO
        fifo_tail = new_tail;       // advance FIFO tail index
        return 1;                   // success
    }
    return 0;   // full
}

// get data from the FIFO
// returns 1 on success, 0 if FIFO was empty
int fifo_get(int * data) {

    if (fifo_head != fifo_tail) {   // if the FIFO is not empty
        * data = fifo[fifo_head];    // read data from the FIFO
        if (fifo_head >= FIFO_SIZE) fifo_head = 0; // advance FIFO head index
        else fifo_head++; // advance FIFO head index
        return 1;                   // success
    }
    return 0;   // empty
}


