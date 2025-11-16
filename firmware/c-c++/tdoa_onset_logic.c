#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <time.h>

// config const.

// samples captured by dma pre-processing
#define BUFFER_SIZE 4096

// 3 microphones
#define NUM_CHANNELS 3

// sample rate per microphone
#define SAMPLE_RATE_PER_CHANNEL 48000.0f // Hz

// onset algo. const.

// window size for short-time energy (hereinafter ste) calc (samples/channel)
// 64 samples @ 48khz ~ 1.3ms

#define STE_WINDOW_SIZE 64

// factor whereby peak ste must > avg noise ste
#define THRESHOLD_MULTIPLIER 5.0f

// num initial samples for calc. noise floor; must > ste_window_size
#define NOISE_FLOOR_SAMPLES 512

uint16_t capture_buf[BUFFER_SIZE];

int main() {

    return 0;
}