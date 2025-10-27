# TDoA Sound Localization - Quick Explanation

## The Big Picture

We locate sound sources using **Time Difference of Arrival (TDoA)**. Sound travels at 343 m/s, so microphones at different positions receive the sound at slightly different times. By measuring these time differences, we can calculate where the sound came from.

**System**: 3 microphones in an equilateral triangle (20 cm sides) → Angle & Distance to sound source

---

## Part 1: Time Delay Estimator

**Goal**: Find *when* sound arrived at each microphone

**Process**:
1. **Capture audio**: 50ms from each mic (2400 samples at 48 kHz)
2. **Smooth it**: Apply envelope detection to get signal energy over time
3. **Find onset**: Detect when signal crosses threshold (15% of peak)
4. **Calculate delays**: Compare onset times between mics
5. **Convert to distances**: time_delay × 343 m/s = distance_difference

**Example**:
- Mic1 onset: sample 210
- Mic2 onset: sample 205 (5 samples earlier)
- Mic3 onset: sample 215 (5 samples later)

→ Sound came from direction of Mic2!

**Output**: `DELTA_D12_M = -0.036 m` and `DELTA_D13_M = +0.036 m`

---

## Part 2: TDoA Solver

**Goal**: Convert distance differences → Angle & Distance

### Angle (Simple & Reliable)

Uses **direct algebraic calculation** based on triangle geometry:
```
Given DELTA_D12 and DELTA_D13 + microphone positions
→ Solve for direction vector (cos θ, sin θ)
→ Convert to angle in degrees
```

**Fast** (no iteration), **accurate** (0-5° error), **always works** at any range.

### Distance (Complex & Range-Limited)

Uses **Levenberg-Marquardt optimization** with **multi-start strategy**:

1. Try 6 starting guesses along the calculated angle: 1m, 3m, 5m, 8m, 12m, 18m
2. For each guess, iteratively refine position:
   - Calculate error: How well does this position match our measurements?
   - Adjust position to reduce error
   - Repeat until converged (typically 10-50 iterations)
3. Keep the best result

**Why multi-start?** TDoA equations can have multiple solutions. Trying different starting points ensures we find the right one.

**Why iterative?** No closed-form solution exists for distance with 3 mics.

**Confidence check**: Only report distance if error < 1m (works well 0-10m, unreliable beyond)

---

## Why It Works

**Angle is reliable** because:
- Depends on *ratio* of time delays (insensitive to small errors)
- Direct calculation (no iteration uncertainty)

**Distance is harder** because:
- Depends on *absolute* time delays (small errors magnified at range)
- Requires solving nonlinear equations (multiple possible solutions)
- Time differences become tiny at long range (<50 microseconds)

---

## Key Numbers

- **Sample rate**: 48 kHz (21 µs resolution)
- **Max time difference**: ~580 µs (20 cm baseline)
- **Angle accuracy**: 0-5° typical
- **Distance accuracy**: Good up to ~10m, unreliable beyond
- **Processing time**: ~20ms total per clap

---

## Summary

**Time Delay Estimator**: Captures audio → Detects onsets → Calculates time/distance differences

**TDoA Solver**: Distance differences → **Angle** (always accurate) + **Distance** (when confident)

The system excels at determining *direction* to a sound source, with distance estimation as a bonus at close range.

---

## Next Steps: Hardware Implementation

### What We Have Now
- Time delay estimator (works on pre-captured buffers)
- TDoA solver (angle + distance calculation)
- Test suites (simulated claps)

### What We Need for Pico Hardware

#### 1. ADC Configuration & Sampling
```c
// Set up 3 ADC channels at 48 kHz
// Configure round-robin sampling (Mic1 → Mic2 → Mic3)
// Account for ~2-20 µs time skew between channels
```

**Key challenge**: ADC multiplexing introduces timing errors. Document this for later correction.

#### 2. Continuous Capture System
```c
// Circular buffer approach:
while (1) {
    // Continuously fill buffer with ADC samples
    sample_all_mics();
    
    // Check for trigger event
    if (signal_detected()) {
        break;
    }
}
```

Need to implement a **ring buffer** that continuously overwrites old data until triggered.

#### 3. Trigger Detection Logic
```c
// Lightweight real-time detection:
bool signal_detected() {
    current_amplitude = abs(mic1_latest_sample);
    
    if (current_amplitude > TRIGGER_THRESHOLD) {
        return true;  // Something happened!
    }
    return false;
}
```

**Strategy**:
- Monitor amplitude on one mic (e.g., Mic1) in real-time
- Threshold: ~5-10% of expected clap amplitude
- When triggered → capture full 50ms window from all 3 mics

#### 4. Window Capture After Trigger
```c
// When trigger detected:
// 1. Stop continuous capture
// 2. Copy last 50ms from ring buffer to MicBuffer structs
// 3. Run onset detection on all 3 channels
// 4. Calculate time delays
// 5. Feed to TDoA solver
// 6. Display results
// 7. Wait 500ms (debounce)
// 8. Resume continuous capture
```

#### 5. Integration & Testing
- Wire up 3 electret mics to ADC pins (GP26, GP27, GP28)
- Add bias resistors (2.2kΩ) and DC-blocking caps (10µF)
- Implement basic UART debug output
- Test with real handclaps at known positions (1m, 2m, 3m)
- Calibrate `ONSET_THRESHOLD` and `TRIGGER_THRESHOLD` for your environment

### Implementation Priority

**Week 1-2: Get it working (any accuracy)**
1. ADC setup and basic sampling
2. Simple trigger detection (threshold on any mic)
3. Capture 50ms window when triggered
4. Run existing time delay estimator code
5. Feed to TDoA solver
6. Print angle to UART

**Week 3-4: Improve accuracy**
1. Tune thresholds for reliable detection
2. Add noise floor estimation
3. Implement debouncing/cooldown period
4. Test accuracy at multiple positions
5. Document ADC timing skew effects

**Future: Production quality**
1. Switch to I2S/PDM mics (eliminate ADC mux skew)
2. Implement GCC-PHAT for continuous sounds
3. Add position tracking/filtering
4. Optimize for real-time performance

### Critical Code Sections to Write

1. **`adc_init_for_tdoa()`** - Configure 3-channel ADC sampling
2. **`ring_buffer_update()`** - Continuous circular buffer management
3. **`check_trigger()`** - Real-time amplitude monitoring
4. **`capture_window()`** - Extract 50ms from ring buffer when triggered
5. **`main_loop()`** - Tie everything together

### Expected Challenges

- **ADC timing**: Sequential sampling creates 2-20µs skew between channels
- **Trigger sensitivity**: Too sensitive = false positives, too insensitive = missed claps
- **Memory**: Need ~14KB RAM for buffers (3 × 2400 samples × 2 bytes)
- **Processing time**: Must process within ~100ms to feel responsive

These are all solvable - the prototype goal is to get something working first, then refine!
