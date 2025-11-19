/**
 * APSC 140 - Acoustic Location System (TDoA)
 * Platform: Raspberry Pi Pico 2 (RP2350)
 * * HARDWARE CONNECTIONS:
 * - Mic 1 (Top):   GP26 (ADC0)
 * - Mic 2 (Right): GP27 (ADC1)
 * - Mic 3 (Left):  GP28 (ADC2)
 * - Ground:        Any GND
 * - VCC:           3.3V
 */

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

// Buffer: 6144 samples total (2048 per mic) -> approx 42ms of audio
// This is short, but enough for a clap. 
#define BUFFER_SIZE 6144 

// Speed of sound (Temperature dependent, approx 20C)
#define SPEED_OF_SOUND 343.0f

// --- GLOBALS ---
// The raw buffer where DMA dumps ADC data
// Format: [M1, M2, M3, M1, M2, M3...]
uint8_t capture_buf[BUFFER_SIZE]; 

// TDoA Solver Globals (from your original code)
float DELTA_D12_M = 0.0f;
float DELTA_D13_M = 0.0f;

// --- TDoA SOLVER CONSTANTS ---
// Geometry: Equilateral Triangle, 20cm side
const float MIC1_X = 0.0f;       const float MIC1_Y = 0.11547f;
const float MIC2_X = -0.1f;      const float MIC2_Y = -0.057735f;
const float MIC3_X = 0.1f;       const float MIC3_Y = -0.057735f;

// --- ONSET DETECTION CONSTANTS ---
#define STE_WINDOW_SIZE 64
#define THRESHOLD_MULTIPLIER 6.0f // Increased slightly for robustness
#define NOISE_FLOOR_SAMPLES 512   // First ~10ms used for noise floor

// ==========================================
// SECTION 1: ONSET DETECTION LOGIC (FIXED)
// ==========================================
// Note: Input changed to uint8_t because Pico ADC FIFO in DMA usually moves bytes
// unless configured for 16-bit. We will use 8-bit for speed/simplicity first, 
// effectively 0-255 range. Center is ~128.

int detect_onset_indices(const uint8_t* buffer, int buffer_len, int32_t* n1_out, int32_t* n2_out, int32_t* n3_out) {
    *n1_out = -1;
    *n2_out = -1;
    *n3_out = -1; // FIXED: Added pointer dereference

    float total_noise_ste[NUM_CHANNELS] = {0.0f};
    int num_ste_windows = 0;

    // 1. Calculate Noise Floor (First portion of buffer)
    for (int i = 0; i < NOISE_FLOOR_SAMPLES * NUM_CHANNELS; i += NUM_CHANNELS) {
        if (i + STE_WINDOW_SIZE * NUM_CHANNELS >= buffer_len) break;

        float current_ste[3] = {0};

        for (int j = 0; j < STE_WINDOW_SIZE; j++) {
            for (int k = 0; k < 3; k++) {
                // Convert 8-bit (0-255) to centered (-128 to 127)
                int32_t sample = (int32_t)buffer[i + k + j * NUM_CHANNELS] - 128;
                current_ste[k] += (float)(sample * sample);
            }
        }
        
        total_noise_ste[0] += current_ste[0];
        total_noise_ste[1] += current_ste[1];
        total_noise_ste[2] += current_ste[2];
        num_ste_windows++;
    }

    if (num_ste_windows == 0) return 0;

    float avg_noise_ste[3];
    float threshold[3];
    
    for(int k=0; k<3; k++) {
        avg_noise_ste[k] = total_noise_ste[k] / num_ste_windows;
        // Ensure a minimum threshold so silence doesn't trigger on electrical noise
        if (avg_noise_ste[k] < 100.0f) avg_noise_ste[k] = 100.0f; 
        threshold[k] = avg_noise_ste[k] * THRESHOLD_MULTIPLIER;
    }

    // 2. Search for Onset
    // Start AFTER the noise floor calculation area
    int start_index = NOISE_FLOOR_SAMPLES * NUM_CHANNELS;

    // FIXED: The curly brace issue from your original code is fixed here
    for (int i = start_index; i < buffer_len - STE_WINDOW_SIZE * NUM_CHANNELS; i += NUM_CHANNELS) {
        
        float current_ste[3] = {0};

        for (int j = 0; j < STE_WINDOW_SIZE; j++) {
            for(int k=0; k<3; k++) {
                int32_t sample = (int32_t)buffer[i + k + j * NUM_CHANNELS] - 128;
                current_ste[k] += (float)(sample * sample);
            }
        }

        // Check thresholds
        // n_out stores the "Frame Index", not the raw array index
        int current_frame = i / NUM_CHANNELS;

        if (*n1_out == -1 && current_ste[0] > threshold[0]) *n1_out = current_frame;
        if (*n2_out == -1 && current_ste[1] > threshold[1]) *n2_out = current_frame;
        if (*n3_out == -1 && current_ste[2] > threshold[2]) *n3_out = current_frame;

        // If all found, exit early
        if (*n1_out != -1 && *n2_out != -1 && *n3_out != -1) return 1;
    }

    // Return 1 only if ALL mics heard the clap
    if (*n1_out != -1 && *n2_out != -1 && *n3_out != -1) return 1;
    return 0;
}


// ==========================================
// SECTION 2: TDoA SOLVER (Condensed)
// ==========================================

float calculate_angle_from_tdoa() {
    // This implements your "100% accurate" algebraic method
    float V21_x = MIC2_X - MIC1_X; float V21_y = MIC2_Y - MIC1_Y;
    float V31_x = MIC3_X - MIC1_X; float V31_y = MIC3_Y - MIC1_Y;
    
    float det = V21_x * V31_y - V21_y * V31_x;
    
    if (fabsf(det) < 1e-9f) return 0.0f;

    // Using globals DELTA_D12_M and DELTA_D13_M
    float cos_theta = (V31_y * DELTA_D12_M - V21_y * DELTA_D13_M) / det;
    float sin_theta = (V21_x * DELTA_D13_M - V31_x * DELTA_D12_M) / det;
    
    // Normalize
    float mag = sqrtf(cos_theta*cos_theta + sin_theta*sin_theta);
    if (mag > 0) { cos_theta /= mag; sin_theta /= mag; }

    float angle = atan2f(sin_theta, cos_theta) * 180.0f / 3.14159f;
    
    // Apply your 180 degree correction
    angle += 180.0f;
    
    // Normalize to 0-360
    while(angle >= 360.0f) angle -= 360.0f;
    while(angle < 0.0f) angle += 360.0f;
    
    return angle;
}

// (I omitted the complex LM distance solver for this "ASAP" file to ensure 
// it compiles small and fast, but the angle solver is here. If angle works, 
// adding distance back is easy).

// ==========================================
// SECTION 3: MAIN & HARDWARE SETUP
// ==========================================

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB Serial
    printf("=== APSC 140: TDoA Clap Detector ===\n");
    printf("Initializing ADC/DMA...\n");

    // 1. Init ADC
    adc_init();
    adc_gpio_init(MIC_PIN_1);
    adc_gpio_init(MIC_PIN_2);
    adc_gpio_init(MIC_PIN_3);

    // 2. Configure Round Robin (Read 0 -> 1 -> 2 -> 0...)
    // Mask 0x07 means ADC 0, 1, and 2 are enabled.
    adc_set_round_robin(0b00000111); 
    
    // 3. Configure ADC FIFO
    adc_fifo_setup(
        true,    // Write to FIFO
        true,    // DMA request enabled
        1,       // DREQ threshold (1 sample)
        false,   // No error bit
        true     // Shift down to 8-bit (0-255) to save space/memory
    );

    // 4. Set Clock Divisor for 144kHz total (48k per channel)
    // Formula: (48MHz / Freq) - 1
    // 48,000,000 / 144,000 = 333.33
    adc_set_clkdiv(332.33f);

    // 5. Init DMA
    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8); // 8-bit transfers
    channel_config_set_read_increment(&c, false); // Read from fixed address (ADC FIFO)
    channel_config_set_write_increment(&c, true); // Write to array
    channel_config_set_dreq(&c, DREQ_ADC);        // Paced by ADC

    printf("System Ready. Waiting for Claps...\n");

    // --- MAIN LOOP ---
    while (1) {
        // A. Drain FIFO (clear old junk)
        adc_run(false);
        adc_fifo_drain();

        // B. Configure and Start DMA
        dma_channel_configure(
            dma_chan,
            &c,
            capture_buf,    // Dest
            &adc_hw->fifo,  // Source
            BUFFER_SIZE,    // Count
            true            // Start immediately
        );

        // C. Start ADC
        adc_select_input(0); // Start at ADC0
        adc_run(true);

        // D. Wait for buffer to fill
        dma_channel_wait_for_finish_blocking(dma_chan);

        // E. Stop ADC
        adc_run(false);

        // F. Process Data
        int32_t n1, n2, n3;
        if (detect_onset_indices(capture_buf, BUFFER_SIZE, &n1, &n2, &n3)) {
            // Calculate Time Differences
            // T = (n_target - n_ref) / SampleRate
            float t1 = (float)n1 / SAMPLE_RATE_PER_CHANNEL;
            float t2 = (float)n2 / SAMPLE_RATE_PER_CHANNEL;
            float t3 = (float)n3 / SAMPLE_RATE_PER_CHANNEL;

            // Calculate Distance Differences (Meters)
            // Using Mic 1 as reference (standard for your solver)
            DELTA_D12_M = (t2 - t1) * SPEED_OF_SOUND;
            DELTA_D13_M = (t3 - t1) * SPEED_OF_SOUND;

            // Solve
            float angle = calculate_angle_from_tdoa();

            // Print Result
            printf("\n--- CLAP DETECTED ---\n");
            printf("Indices: %d, %d, %d\n", n1, n2, n3);
            printf("Delays:  d12=%.4fm, d13=%.4fm\n", DELTA_D12_M, DELTA_D13_M);
            printf("ANGLE:   %.1f degrees\n", angle);
            
            // Small delay so we don't re-trigger on the echo of the same clap
            sleep_ms(500); 
        }

        // No delay needed if no clap; listen immediately again.
    }
}
