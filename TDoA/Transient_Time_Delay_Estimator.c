/*
 * TDoA Time Delay Estimator for Transient Sounds (Claps, Snaps, etc.)
 * 
 * Designed for Raspberry Pi Pico 2 (RP2350) with 3 analog microphones
 * 
 * APPROACH:
 * For transient sounds (claps), we detect the ONSET time at each microphone
 * by finding when the signal amplitude crosses a threshold. The time difference
 * between onsets gives us the TDoA.
 * 
 * ALGORITHM:
 * 1. Capture audio from all 3 mics (sequentially due to ADC mux)
 * 2. Apply envelope detection (running RMS or simple rectification)
 * 3. Detect onset when envelope crosses threshold
 * 4. Calculate time delays between mic pairs
 * 5. Convert to distance differences for TDoA solver
 * 
 * LIMITATIONS (for prototype):
 * - ADC multiplexing introduces time skew between channels
 * - Simple threshold detection (not robust to noise)
 * - For claps/transients only (not continuous sounds)
 * 
 * Next steps for production:
 * - Use I2S/PDM mics for simultaneous sampling
 * - Add GCC-PHAT cross-correlation for better accuracy
 * - Implement noise floor estimation and adaptive thresholding
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// --- Configuration ---
#define SAMPLE_RATE_HZ 48000        // ADC sample rate
#define NUM_MICS 3
#define BUFFER_SIZE 2400            // 50ms at 48kHz
#define ONSET_THRESHOLD 0.15f       // Fraction of max amplitude (0-1)
#define ENVELOPE_WINDOW 20          // Samples for envelope smoothing
#define SPEED_OF_SOUND_MPS 343.0f
#define MAX_DELAY_SAMPLES 35        // ~0.58ms at 48kHz for 20cm baseline

// --- Data Structures ---
typedef struct {
    float samples[BUFFER_SIZE];
    int length;
    int onset_sample;               // Sample index where onset detected
    int onset_found;
    float peak_amplitude;
} MicBuffer;

typedef struct {
    float delta_d12_m;              // Distance difference: mic2 - mic1
    float delta_d13_m;              // Distance difference: mic3 - mic1
    float delay_12_samples;         // Time delay in samples
    float delay_13_samples;
    int valid;                      // 1 if detection succeeded
} TimeDelayResult;

// --- Helper Functions ---

// Simple envelope detector using moving average of absolute values
void calculate_envelope(float *input, float *envelope, int length, int window) {
    for (int i = 0; i < length; i++) {
        float sum = 0.0f;
        int count = 0;
        
        // Average absolute value over window
        for (int j = i - window/2; j <= i + window/2; j++) {
            if (j >= 0 && j < length) {
                sum += fabsf(input[j]);
                count++;
            }
        }
        envelope[i] = sum / count;
    }
}

// Find peak amplitude in signal
float find_peak_amplitude(float *samples, int length) {
    float peak = 0.0f;
    for (int i = 0; i < length; i++) {
        float abs_val = fabsf(samples[i]);
        if (abs_val > peak) {
            peak = abs_val;
        }
    }
    return peak;
}

// Detect onset: first sample where envelope crosses threshold
int detect_onset(float *envelope, int length, float threshold) {
    for (int i = 0; i < length; i++) {
        if (envelope[i] > threshold) {
            return i;
        }
    }
    return -1;  // No onset found
}

// Process a single microphone buffer to find onset
void process_mic_buffer(MicBuffer *mic) {
    // Find peak amplitude
    mic->peak_amplitude = find_peak_amplitude(mic->samples, mic->length);
    
    if (mic->peak_amplitude < 0.001f) {
        // Signal too weak
        mic->onset_found = 0;
        return;
    }
    
    // Calculate envelope
    float envelope[BUFFER_SIZE];
    calculate_envelope(mic->samples, envelope, mic->length, ENVELOPE_WINDOW);
    
    // Detect onset
    float threshold = mic->peak_amplitude * ONSET_THRESHOLD;
    mic->onset_sample = detect_onset(envelope, mic->length, threshold);
    mic->onset_found = (mic->onset_sample >= 0);
}

// Calculate time delays from three mic buffers
TimeDelayResult calculate_time_delays(MicBuffer *mic1, MicBuffer *mic2, MicBuffer *mic3) {
    TimeDelayResult result = {0};
    
    // Check if all onsets were detected
    if (!mic1->onset_found || !mic2->onset_found || !mic3->onset_found) {
        printf("  Error: Onset not detected in all channels\n");
        printf("    Mic1: %s, Mic2: %s, Mic3: %s\n",
               mic1->onset_found ? "OK" : "FAIL",
               mic2->onset_found ? "OK" : "FAIL",
               mic3->onset_found ? "OK" : "FAIL");
        result.valid = 0;
        return result;
    }
    
    // Calculate delays in samples (positive = mic2/3 received sound later)
    result.delay_12_samples = (float)(mic2->onset_sample - mic1->onset_sample);
    result.delay_13_samples = (float)(mic3->onset_sample - mic1->onset_sample);
    
    // Sanity check: delays should be within reasonable range
    if (fabsf(result.delay_12_samples) > MAX_DELAY_SAMPLES ||
        fabsf(result.delay_13_samples) > MAX_DELAY_SAMPLES) {
        printf("  Error: Delays exceed physical limits\n");
        printf("    Delay12: %.1f samples (max: %d)\n", 
               result.delay_12_samples, MAX_DELAY_SAMPLES);
        printf("    Delay13: %.1f samples (max: %d)\n", 
               result.delay_13_samples, MAX_DELAY_SAMPLES);
        result.valid = 0;
        return result;
    }
    
    // Convert to time (seconds)
    float delay_12_sec = result.delay_12_samples / SAMPLE_RATE_HZ;
    float delay_13_sec = result.delay_13_samples / SAMPLE_RATE_HZ;
    
    // Convert to distance differences (meters)
    result.delta_d12_m = delay_12_sec * SPEED_OF_SOUND_MPS;
    result.delta_d13_m = delay_13_sec * SPEED_OF_SOUND_MPS;
    
    result.valid = 1;
    return result;
}

// --- Test Function ---
// Simulate captured audio with known time delays
void simulate_clap(MicBuffer *mic1, MicBuffer *mic2, MicBuffer *mic3,
                   int delay_12, int delay_13) {
    printf("Simulating clap with delays: mic2=%d samples, mic3=%d samples\n",
           delay_12, delay_13);
    
    // Generate a simple clap: quick rise, exponential decay
    int clap_start = 200;  // Start at sample 200
    int clap_duration = 100;
    
    // Initialize all buffers to zero
    memset(mic1->samples, 0, sizeof(mic1->samples));
    memset(mic2->samples, 0, sizeof(mic2->samples));
    memset(mic3->samples, 0, sizeof(mic3->samples));
    
    mic1->length = mic2->length = mic3->length = BUFFER_SIZE;
    
    // Generate clap waveform for each mic
    for (int i = 0; i < clap_duration; i++) {
        float amplitude = expf(-i / 20.0f);  // Exponential decay
        
        // Add to mic1
        if (clap_start + i < BUFFER_SIZE) {
            mic1->samples[clap_start + i] = amplitude;
        }
        
        // Add to mic2 (delayed)
        if (clap_start + delay_12 + i < BUFFER_SIZE && clap_start + delay_12 >= 0) {
            mic2->samples[clap_start + delay_12 + i] = amplitude;
        }
        
        // Add to mic3 (delayed)
        if (clap_start + delay_13 + i < BUFFER_SIZE && clap_start + delay_13 >= 0) {
            mic3->samples[clap_start + delay_13 + i] = amplitude;
        }
    }
    
    // Add small amount of noise
    for (int i = 0; i < BUFFER_SIZE; i++) {
        mic1->samples[i] += (rand() % 100 - 50) / 10000.0f;
        mic2->samples[i] += (rand() % 100 - 50) / 10000.0f;
        mic3->samples[i] += (rand() % 100 - 50) / 10000.0f;
    }
}

// --- Main Test Program ---
int main() {
    printf("===== TDoA Time Delay Estimator - Test Suite =====\n");
    printf("Sample Rate: %d Hz\n", SAMPLE_RATE_HZ);
    printf("Max delay: %d samples (%.2f ms)\n", 
           MAX_DELAY_SAMPLES, 1000.0f * MAX_DELAY_SAMPLES / SAMPLE_RATE_HZ);
    printf("Onset threshold: %.1f%% of peak\n\n", ONSET_THRESHOLD * 100);
    
    MicBuffer mic1, mic2, mic3;
    
    // Test 1: Sound from right (mic2 hears it first)
    printf("TEST 1: Sound from right side\n");
    simulate_clap(&mic1, &mic2, &mic3, -10, 5);  // mic2 earlier, mic3 later
    
    process_mic_buffer(&mic1);
    process_mic_buffer(&mic2);
    process_mic_buffer(&mic3);
    
    printf("  Detected onsets: Mic1=%d, Mic2=%d, Mic3=%d samples\n",
           mic1.onset_sample, mic2.onset_sample, mic3.onset_sample);
    
    TimeDelayResult result1 = calculate_time_delays(&mic1, &mic2, &mic3);
    
    if (result1.valid) {
        printf("  ✓ Time delays calculated:\n");
        printf("    Delay 1→2: %.1f samples (%.3f ms) → %.4f m\n",
               result1.delay_12_samples,
               1000.0f * result1.delay_12_samples / SAMPLE_RATE_HZ,
               result1.delta_d12_m);
        printf("    Delay 1→3: %.1f samples (%.3f ms) → %.4f m\n",
               result1.delay_13_samples,
               1000.0f * result1.delay_13_samples / SAMPLE_RATE_HZ,
               result1.delta_d13_m);
    } else {
        printf("  ✗ Time delay calculation failed\n");
    }
    
    // Test 2: Sound from left (mic3 hears it first)
    printf("\nTEST 2: Sound from left side\n");
    simulate_clap(&mic1, &mic2, &mic3, 8, -12);
    
    process_mic_buffer(&mic1);
    process_mic_buffer(&mic2);
    process_mic_buffer(&mic3);
    
    printf("  Detected onsets: Mic1=%d, Mic2=%d, Mic3=%d samples\n",
           mic1.onset_sample, mic2.onset_sample, mic3.onset_sample);
    
    TimeDelayResult result2 = calculate_time_delays(&mic1, &mic2, &mic3);
    
    if (result2.valid) {
        printf("  ✓ Time delays calculated:\n");
        printf("    Delay 1→2: %.1f samples (%.3f ms) → %.4f m\n",
               result2.delay_12_samples,
               1000.0f * result2.delay_12_samples / SAMPLE_RATE_HZ,
               result2.delta_d12_m);
        printf("    Delay 1→3: %.1f samples (%.3f ms) → %.4f m\n",
               result2.delay_13_samples,
               1000.0f * result2.delay_13_samples / SAMPLE_RATE_HZ,
               result2.delta_d13_m);
    } else {
        printf("  ✗ Time delay calculation failed\n");
    }
    
    // Test 3: Sound from front (all mics similar time)
    printf("\nTEST 3: Sound from front (centered)\n");
    simulate_clap(&mic1, &mic2, &mic3, 2, 2);
    
    process_mic_buffer(&mic1);
    process_mic_buffer(&mic2);
    process_mic_buffer(&mic3);
    
    printf("  Detected onsets: Mic1=%d, Mic2=%d, Mic3=%d samples\n",
           mic1.onset_sample, mic2.onset_sample, mic3.onset_sample);
    
    TimeDelayResult result3 = calculate_time_delays(&mic1, &mic2, &mic3);
    
    if (result3.valid) {
        printf("  ✓ Time delays calculated:\n");
        printf("    Delay 1→2: %.1f samples (%.3f ms) → %.4f m\n",
               result3.delay_12_samples,
               1000.0f * result3.delay_12_samples / SAMPLE_RATE_HZ,
               result3.delta_d12_m);
        printf("    Delay 1→3: %.1f samples (%.3f ms) → %.4f m\n",
               result3.delay_13_samples,
               1000.0f * result3.delay_13_samples / SAMPLE_RATE_HZ,
               result3.delta_d13_m);
    } else {
        printf("  ✗ Time delay calculation failed\n");
    }
    
    printf("\n===== Integration Notes =====\n");
    printf("To use with TDoA solver:\n");
    printf("  DELTA_D12_M = result.delta_d12_m\n");
    printf("  DELTA_D13_M = result.delta_d13_m\n");
    printf("\nFor Pico implementation:\n");
    printf("  1. Capture audio from 3 ADC channels to MicBuffer structs\n");
    printf("  2. Call process_mic_buffer() for each channel\n");
    printf("  3. Call calculate_time_delays() to get DELTA_D values\n");
    printf("  4. Feed into TDoA solver for angle & distance\n");
    
    return 0;
}
