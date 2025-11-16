#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// config const.

// samples captured by dma pre-processing
#define BUFFER_SIZE 6144

// 3 microphones (m1, m2, m3)
#define NUM_CHANNELS 3

// sample rate per microphone
#define SAMPLE_RATE_PER_CHANNEL 48000.0f  // Hz

// onset det. algo. const.

// window size for short-time energy (hereinafter ste) calc (samples/channel)
// 64 samples @ 48khz ~ 1.3ms

#define STE_WINDOW_SIZE 64

// factor whereby peak ste must > avg noise ste
#define THRESHOLD_MULTIPLIER 5.0f

// num initial samples for calc. noise floor; must > ste_window_size
#define NOISE_FLOOR_SAMPLES 512

uint16_t capture_buf[BUFFER_SIZE];

// primary dsp function

int detect_onset_indices(const uint16_t* buffer, int buffer_len, int32_t* n1_out, int32_t* n2_out, int32_t* n3_out) {
    *n1_out = -1;  // pointer to store m1 onset sample index
    *n2_out = -1;  //"
    n3_out = -1;   //"

    float total_noise_ste[NUM_CHANNELS] = {0.0f};
    int num_ste_windows = 0;

    for (int i = 0; i < NOISE_FLOOR_SAMPLES * NUM_CHANNELS; i += NUM_CHANNELS) {
        if (i + STE_WINDOW_SIZE * NUM_CHANNELS >= buffer_len)
            break;  // check boundary

        // calculate ste for m1 (start at buffer[i] step = 3)
        float current_ste1 = 0.0f;
        float current_ste2 = 0.0f;
        float current_ste3 = 0.0f;

        for (int j = 0; j < STE_WINDOW_SIZE; j++) {
            // m1: index i + j * 3
            int32_t sample1 = (int32_t)buffer[i + j * NUM_CHANNELS] - 2048;
            current_ste1 += (float)(sample1 * sample1);

            // m2: index i + 1 + j * 3
            int32_t sample2 = (int32_t)buffer[i + 1 + j * NUM_CHANNELS] - 2048;
            current_ste2 += (float)(sample2 * sample2);

            // m3: index i + 2 + j * 3
            int32_t sample3 = (int32_t)buffer[i + 2 + j * NUM_CHANNELS] - 2048;
            current_ste3 += (float)(sample3 * sample3);
        }
        total_noise_ste[0] += current_ste1;
        total_noise_ste[1] += current_ste2;
        total_noise_ste[2] += current_ste3;

        num_ste_windows++;

        if (num_ste_windows == 0) {
            printf("err: buffer too short to calculate noise floor.\n");
            return 0;
        }

        float avg_noise_ste1 = total_noise_ste[0] / num_ste_windows;
        float avg_noise_ste2 = total_noise_ste[1] / num_ste_windows;
        float avg_noise_ste3 = total_noise_ste[2] / num_ste_windows;

        // define detection threshold
        float threshold1 = avg_noise_ste1 * THRESHOLD_MULTIPLIER;
        float threshold2 = avg_noise_ste2 * THRESHOLD_MULTIPLIER;
        float threshold3 = avg_noise_ste3 * THRESHOLD_MULTIPLIER;

        printf("--- noise floor and threshold ---\n");
        printf("m1 avg noise STE: %.1f | threshold: %.1f\n", avg_noise_ste1, threshold1);
        printf("m2 avg noise STE: %.1f | threshold: %.1f\n", avg_noise_ste2, threshold2);
        printf("m3 avg noise STE: %.1f | threshold: %.1f\n", avg_noise_ste3, threshold3);

        // part 2 search for onset (first ste window that exceeds threshold)

        int start_index = NOISE_FLOOR_SAMPLES * NUM_CHANNELS;  // start search from here

        for (int i = start_index; i < buffer_len - STE_WINDOW_SIZE * NUM_CHANNELS; i += NUM_CHANNELS) {
            // calculate ste of current window for all m
            float current_ste1 = 0.0f;
            float current_ste2 = 0.0f;
            float current_ste3 = 0.0f;
        }

        for (int j = 0; j < STE_WINDOW_SIZE; j++) {
            // m1 (index i + j*3)
            current_ste1 += (float)(((int32_t)buffer[i + j * NUM_CHANNELS] - 2048) *
                                    ((int32_t)buffer[i + j * NUM_CHANNELS] - 2048));

            // m2 (index i + 1 + j*3)
            current_ste2 += (float)(((int32_t)buffer[i + 1 + j * NUM_CHANNELS] - 2048) *
                                    ((int32_t)buffer[i + 1 + j * NUM_CHANNELS] - 2048));

            // m3 (index i + 2 + j*3)
            current_ste3 += (float)(((int32_t)buffer[i + 2 + j * NUM_CHANNELS] - 2048) *
                                    ((int32_t)buffer[i + 2 + j * NUM_CHANNELS] - 2048));
        }

        // check thresholds and record non interleaved sample index ( i / num
        // channels)
        if (*n1_out == -1 && current_ste1 > threshold1) {
            *n1_out = i / NUM_CHANNELS;
        }
        if (*n2_out == -1 && current_ste2 > threshold2) {
            *n2_out = i / NUM_CHANNELS;
        }
        if (*n3_out == -1 && current_ste3 > threshold3) {
            *n3_out = i / NUM_CHANNELS;
        }

        // stop if all onsets found
        if (*n1_out != -1 && *n2_out != -1 && *n3_out != -1) {
            break;
        }
    }
    // part 3 check results

    if (*n1_out == -1 || *n2_out == -1 || *n3_out == -1) {
        printf("err: onset not detected for one or more microphones in buffer.\n");
        return 0;
    }

    return 1;
}

int main() {
    return 0;
}