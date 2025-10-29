# Technical Explanation: TDoA Sound Source Localization System

## Overview

This document provides a detailed, step-by-step explanation of how our Time Difference of Arrival (TDoA) sound localization system works. The system consists of two main programs that work in sequence to determine the angle and distance to a sound source.

---

## Part 1: Time Delay Estimator (`tdoa_time_delay.c`)

### Purpose
This program analyzes audio captured from three microphones to determine **when** the sound arrived at each microphone. It outputs the time differences (converted to distance differences) between microphone pairs.

### Physical Setup
- Three microphones arranged in an equilateral triangle (20 cm sides)
- Sound travels at 343 m/s through air
- Maximum time difference for our array: ~0.58 milliseconds (sound traveling 20 cm)

### Algorithm Overview

The time delay estimator uses a simple but effective approach called **onset detection** for transient sounds like handclaps.

---

### Step-by-Step Process

#### **Step 1: Audio Capture**

```c
MicBuffer mic1, mic2, mic3;
// Each buffer contains 2400 samples (50ms at 48kHz sample rate)
```

We capture a short window of audio from each microphone:
- **Sample rate**: 48,000 samples per second
- **Buffer size**: 2400 samples = 50 milliseconds of audio
- **Time resolution**: 1/48000 = ~21 microseconds per sample

**Why 50ms?** This is long enough to capture a full handclap (~100ms duration) while keeping memory usage reasonable.

---

#### **Step 2: Envelope Detection**

The raw audio signal oscillates rapidly (positive and negative values). To find when the sound "starts," we need to smooth it out using an **envelope detector**.

```c
void calculate_envelope(float *input, float *envelope, int length, int window)
```

**What it does:**
1. For each sample point, look at surrounding samples within a window (±10 samples)
2. Take the absolute value of each sample (make all positive)
3. Average these values
4. Result: A smooth curve showing the "energy" of the signal over time

**Example:**
```
Raw audio:     /\/\/\/\    /\/\/\/\/\/\
                --------    -------------
Envelope:          /\              /\
                  /  \            /  \
                 /    \          /    \
```

The envelope rises when the clap happens and falls as it fades away.

**Why we need this:** Raw audio oscillates too quickly to find a clear "start time." The envelope gives us a smooth rise that's easy to detect.

---

#### **Step 3: Find Peak Amplitude**

```c
float find_peak_amplitude(float *samples, int length)
```

**What it does:**
- Scans through all samples
- Finds the maximum absolute value
- Returns this peak amplitude

**Why we need this:** We need to know how loud the sound is to set an appropriate detection threshold. A loud clap at 1 meter and a quiet clap at 3 meters should both be detected, even though they have different amplitudes.

---

#### **Step 4: Onset Detection**

```c
int detect_onset(float *envelope, int length, float threshold)
```

**What it does:**
1. Calculate threshold = peak_amplitude × 0.15 (15% of peak)
2. Scan through envelope from start to end
3. Find first sample where envelope exceeds threshold
4. Return that sample index

**Example:**
```
Envelope amplitude over time:

 1.0 |                    peak
     |                     *
 0.8 |                   * * *
     |                  *  *  *
 0.6 |                 *   *   *
     |               *     *     *
 0.4 |             *       *       *
     |           *         *         *
 0.2 |         *           *           *
     |-------*-------------*-------------*----
 0.15|----*  threshold                        
     |   ^
     |   |
     | ONSET (first crossing of threshold)
     |
     +----------------------------------------> time
         200            300            400 samples
```

The **onset sample** is where we consider the sound to have "arrived" at that microphone.

**Why 15%?** This is a tunable parameter:
- Too low (5%): Triggers on background noise
- Too high (50%): Might miss the actual onset if the rise is gradual
- 15% is a good balance for handclaps

---

#### **Step 5: Process All Three Microphones**

```c
process_mic_buffer(&mic1);
process_mic_buffer(&mic2);
process_mic_buffer(&mic3);
```

This runs steps 2-4 for each microphone independently, giving us three onset times:
- `mic1.onset_sample` (e.g., sample 210)
- `mic2.onset_sample` (e.g., sample 205) ← sound arrived earlier!
- `mic3.onset_sample` (e.g., sample 215)

---

#### **Step 6: Calculate Time Delays**

```c
TimeDelayResult calculate_time_delays(MicBuffer *mic1, *mic2, *mic3)
```

**What it does:**

1. **Calculate sample differences:**
   ```
   delay_12_samples = mic2.onset_sample - mic1.onset_sample
                    = 205 - 210 = -5 samples
   
   delay_13_samples = mic3.onset_sample - mic1.onset_sample
                    = 215 - 210 = +5 samples
   ```
   
   **Interpretation:** 
   - Mic2 heard sound 5 samples earlier (negative delay)
   - Mic3 heard sound 5 samples later (positive delay)
   - Therefore: sound came from direction of Mic2

2. **Convert samples to time:**
   ```
   delay_12_seconds = -5 / 48000 = -0.0001042 seconds = -104.2 microseconds
   delay_13_seconds = +5 / 48000 = +0.0001042 seconds = +104.2 microseconds
   ```

3. **Convert time to distance:**
   ```
   delta_d12_m = -0.0001042 × 343 m/s = -0.0357 meters = -3.57 cm
   delta_d13_m = +0.0001042 × 343 m/s = +0.0357 meters = +3.57 cm
   ```

4. **Sanity check:**
   - Verify delays are physically possible
   - Maximum delay should be ≤ baseline/speed_of_sound = 0.2m / 343m/s ≈ 0.58ms
   - At 48kHz, this is ~28 samples maximum
   - Our delays (-5, +5) are well within this range ✓

---

#### **Step 7: Output**

```c
result.delta_d12_m = -0.0357;  // meters
result.delta_d13_m = +0.0357;  // meters
result.valid = 1;              // success flag
```

These distance differences are passed to the TDoA solver.

---

### Key Insights

**Why this works:**
- Sound travels at finite speed (343 m/s)
- Microphones at different positions receive sound at different times
- Time difference directly relates to direction of source

**Limitations:**
- Requires clear onset (works for claps, not continuous sounds)
- Sensitive to noise
- Sequential ADC sampling introduces small timing errors

---

## Part 2: TDoA Solver (`tdoa_solver.c`)

### Purpose
This program takes the distance differences from the time delay estimator and calculates:
1. **Angle**: Direction to sound source (always computed)
2. **Distance**: How far away the source is (computed when confident)

### Mathematical Foundation

The TDoA problem is based on **hyperbolic geometry**:
- A time difference between two microphones defines a hyperbola
- The sound source lies somewhere on this hyperbola
- Two hyperbolas (from 3 mics) intersect at the source location

---

### Step-by-Step Process

#### **Step 1: Angle Calculation (Direct Method)**

```c
float calculate_angle_from_tdoa(float *cos_out, float *sin_out)
```

This uses a **closed-form solution** based on the geometry of our triangular array.

**The math:**

Given:
- `DELTA_D12_M`: Distance difference between mic2 and mic1
- `DELTA_D13_M`: Distance difference between mic3 and mic1
- Known microphone positions

We can directly calculate the direction vector (cos θ, sin θ) using:

```
V21 = position of mic2 - position of mic1
V31 = position of mic3 - position of mic1

det = V21.x × V31.y - V21.y × V31.x

cos θ = (V31.y × DELTA_D12 - V21.y × DELTA_D13) / det
sin θ = (V21.x × DELTA_D13 - V31.x × DELTA_D12) / det
```

**Then:**
1. Normalize (cos θ, sin θ) to unit length
2. Add 180° correction (accounts for systematic offset in our geometry)
3. Convert to degrees: `angle = atan2(sin θ, cos θ) × 180/π`

**Example calculation:**
```
If DELTA_D12 = -0.0357 m and DELTA_D13 = +0.0357 m
→ cos θ ≈ 0.707, sin θ ≈ -0.707
→ angle ≈ 315° (sound from lower-right, toward mic2)
```

**Why this works:**
- The angle calculation is **algebraic** (no iteration needed)
- It's based purely on the geometry of the microphone triangle
- Very fast and accurate (typically within 0-5° of true angle)

---

#### **Step 2: Distance Estimation (Iterative Method)**

Distance is much harder to estimate than angle because small errors in time delays cause large distance errors at long range.

We use **Levenberg-Marquardt (LM) optimization** with a **multi-start strategy**.

---

##### **Step 2A: Multi-Start Strategy**

```c
float test_distances[] = {1.0f, 3.0f, 5.0f, 8.0f, 12.0f, 18.0f};
```

**The problem:** TDoA equations can have multiple solutions (like a quadratic equation).

**The solution:** Try multiple starting guesses along the calculated angle direction:

```
For each test distance:
    initial_x = test_distance × cos(angle)
    initial_y = test_distance × sin(angle)
    
    Run LM solver from this starting point
    
    Keep track of which solution is best
```

**Example:** If angle = 45°, we try starting points at:
- (0.707, 0.707) meters [1m away]
- (2.12, 2.12) meters [3m away]
- (3.54, 3.54) meters [5m away]
- ... etc

We run the solver 6 times and pick the best result.

---

##### **Step 2B: Levenberg-Marquardt Iteration**

For each starting point, we iteratively refine the position estimate.

**Setup:**

We want to find position (x, y) that minimizes the error in our TDoA equations:

```
Error function:
    r1 = distance from (x,y) to mic1
    r2 = distance from (x