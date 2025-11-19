/**
 * APSC 140 - Acoustic Location System (TDoA)
 * Platform: Raspberry Pi Pico 2 (RP2350)
 * IMPROVED VERSION: Large Buffer + Calibration + LED + "Sound" Terminology
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

// --- CONFIGURATION ---
#define MIC_PIN_1 26
#define MIC_PIN_2 27
#define MIC_PIN_3 28

// 3 Mics, 48kHz each = 144kHz total ADC speed
#define NUM_CHANNELS 3
#define SAMPLE_RATE_PER_CHANNEL 48000.0f
#define TOTAL_SAMPLE_RATE (SAMPLE_RATE_PER_CHANNEL * NUM_CHANNELS)

// IMPROVEMENT: Buffer increased to ~0.31 seconds
// This prevents cutting off the sound if it happens near the end of a capture
#define BUFFER_SIZE 45000

// Speed of sound (Temperature dependent, approx 20C)
#define SPEED_OF_SOUND 343.0f

// --- GLOBALS ---
uint8_t capture_buf[BUFFER_SIZE];

// Thresholds (Calibrated at startup)
float threshold_mic1 = 0.0f;
float threshold_mic2 = 0.0f;
float threshold_mic3 = 0.0f;

// TDoA Solver Globals
float DELTA_D12_M = 0.0f;
float DELTA_D13_M = 0.0f;

// Geometry Constants (Equilateral Triangle 20cm)
const float MIC1_X = 0.0f;       const float MIC1_Y = 0.11547f;
const float MIC2_X = -0.1f;      const float MIC2_Y = -0.057735f;
const float MIC3_X = 0.1f;       const float MIC3_Y = -0.057735f;

// --- ONSET DETECTION CONSTANTS ---
#define STE_WINDOW_SIZE 64
#define THRESHOLD_MULTIPLIER 5.0f 

// ==========================================
// SECTION 1: CALIBRATION & DETECTION
// ==========================================

// Helper: Calculate Short-Time Energy for a specific window
float get_window_ste(const uint8_t* buffer, int start_index, int channel_offset) {
    float ste = 0.0f;
    for (int j = 0; j < STE_WINDOW_SIZE; j++) {
        int idx = start_index + (j * NUM_CHANNELS) + channel_offset;
        if (idx >= BUFFER_SIZE) break;
        
        int32_t sample = (int32_t)buffer[idx] - 128; // Center around 0
        ste += (float)(sample * sample);
    }
    return ste;
}

// IMPROVEMENT: Separate Calibration Routine
void calibrate_system(int dma_chan, dma_channel_config* c) {
    printf(">>> CALIBRATING... STAY QUIET <<<\n");
    gpio_put(PICO_DEFAULT_LED_PIN, 1); // LED ON during cal
    
    // Capture one full buffer of "silence"
    adc_fifo_drain();
    dma_channel_configure(dma_chan, c, capture_buf, &adc_hw->fifo, BUFFER_SIZE, true);
    adc_select_input(0);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);

    float total_ste[3] = {0};
    int windows = 0;

    // Average the energy across the whole silence buffer
    for (int i = 0; i < BUFFER_SIZE - (STE_WINDOW_SIZE * NUM_CHANNELS); i += (STE_WINDOW_SIZE * NUM_CHANNELS)) {
        total_ste[0] += get_window_ste(capture_buf, i, 0);
        total_ste[1] += get_window_ste(capture_buf, i, 1);
        total_ste[2] += get_window_ste(capture_buf, i, 2);
        windows++;
    }

    float avg_ste_0 = total_ste[0] / windows;
    float avg_ste_1 = total_ste[1] / windows;
    float avg_ste_2 = total_ste[2] / windows;

    // Floor at 50.0 to prevent electrical noise triggers
    if (avg_ste_0 < 50.0f) avg_ste_0 = 50.0f;
    if (avg_ste_1 < 50.0f) avg_ste_1 = 50.0f;
    if (avg_ste_2 < 50.0f) avg_ste_2 = 50.0f;

    threshold_mic1 = avg_ste_0 * THRESHOLD_MULTIPLIER;
    threshold_mic2 = avg_ste_1 * THRESHOLD_MULTIPLIER;
    threshold_mic3 = avg_ste_2 * THRESHOLD_MULTIPLIER;

    printf("CALIBRATION COMPLETE.\n");
    printf("Thresholds: M1:%.0f, M2:%.0f, M3:%.0f\n", threshold_mic1, threshold_mic2, threshold_mic3);
    gpio_put(PICO_DEFAULT_LED_PIN, 0); // LED OFF
    sleep_ms(500);
}

// IMPROVEMENT: Faster scan using pre-calculated thresholds
int scan_buffer_for_sounds(int32_t* n1_out, int32_t* n2_out, int32_t* n3_out) {
    *n1_out = -1; *n2_out = -1; *n3_out = -1;

    // Scan the entire buffer
    for (int i = 0; i < BUFFER_SIZE - (STE_WINDOW_SIZE * NUM_CHANNELS); i += NUM_CHANNELS) {
        
        if (*n1_out == -1) {
            if (get_window_ste(capture_buf, i, 0) > threshold_mic1) *n1_out = i / NUM_CHANNELS;
        }
        if (*n2_out == -1) {
            if (get_window_ste(capture_buf, i, 1) > threshold_mic2) *n2_out = i / NUM_CHANNELS;
        }
        if (*n3_out == -1) {
            if (get_window_ste(capture_buf, i, 2) > threshold_mic3) *n3_out = i / NUM_CHANNELS;
        }

        if (*n1_out != -1 && *n2_out != -1 && *n3_out != -1) return 1;
    }
    
    return (*n1_out != -1 && *n2_out != -1 && *n3_out != -1);
}

// ==========================================
// SECTION 2: TDoA SOLVER
// ==========================================
float calculate_angle_from_tdoa() {
    float V21_x = MIC2_X - MIC1_X; float V21_y = MIC2_Y - MIC1_Y;
    float V31_x = MIC3_X - MIC1_X; float V31_y = MIC3_Y - MIC1_Y;

    float det = V21_x * V31_y - V21_y * V31_x;

    if (fabsf(det) < 1e-9f) return 0.0f;

    float cos_theta = (V31_y * DELTA_D12_M - V21_y * DELTA_D13_M) / det;
    float sin_theta = (V21_x * DELTA_D13_M - V31_x * DELTA_D12_M) / det;

    float mag = sqrtf(cos_theta*cos_theta + sin_theta*sin_theta);
    if (mag > 0) { cos_theta /= mag; sin_theta /= mag; }

    float angle = atan2f(sin_theta, cos_theta) * 180.0f / 3.14159f;

    angle += 180.0f;
    while(angle >= 360.0f) angle -= 360.0f;
    while(angle < 0.0f) angle += 360.0f;

    return angle;
}

// ==========================================
// SECTION 3: MAIN & HARDWARE SETUP
// ==========================================
int main() {
    stdio_init_all();
    
    // IMPROVEMENT: LED Feedback
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    sleep_ms(2000); // Wait for USB Serial
    printf("=== APSC 140: TDoA Sound Detector (Improved) ===\n");
    printf("Initializing ADC/DMA...\n");

    adc_init();
    adc_gpio_init(MIC_PIN_1);
    adc_gpio_init(MIC_PIN_2);
    adc_gpio_init(MIC_PIN_3);
    adc_set_round_robin(0b00000111); 
    adc_fifo_setup(true, true, 1, false, true);
    adc_set_clkdiv(332.33f); 

    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_ADC);        

    // Run Calibration ONCE at startup
    calibrate_system(dma_chan, &c);

    printf("System Ready. Waiting for Sound...\n");

    while (1) {
        // A. LED ON = Listening
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        adc_run(false);
        adc_fifo_drain();
        dma_channel_configure(dma_chan, &c, capture_buf, &adc_hw->fifo, BUFFER_SIZE, true);
        adc_select_input(0);
        adc_run(true);
        dma_channel_wait_for_finish_blocking(dma_chan);
        adc_run(false);

        // B. LED OFF = Processing
        gpio_put(PICO_DEFAULT_LED_PIN, 0);

        int32_t n1, n2, n3;
        if (scan_buffer_for_sounds(&n1, &n2, &n3)) {
            float t1 = (float)n1 / SAMPLE_RATE_PER_CHANNEL;
            float t2 = (float)n2 / SAMPLE_RATE_PER_CHANNEL;
            float t3 = (float)n3 / SAMPLE_RATE_PER_CHANNEL;

            DELTA_D12_M = (t2 - t1) * SPEED_OF_SOUND;
            DELTA_D13_M = (t3 - t1) * SPEED_OF_SOUND;

            float angle = calculate_angle_from_tdoa();

            printf("\n--- SOUND DETECTED ---\n");
            printf("Indices: %d, %d, %d\n", n1, n2, n3);
            printf("Delays:  d12=%.4fm, d13=%.4fm\n", DELTA_D12_M, DELTA_D13_M);
            printf("ANGLE:   %.1f degrees\n", angle);

            // Flash LED to show success
            for(int i=0; i<4; i++) {
                gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(50);
                gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(50);
            }
        }
    }
}