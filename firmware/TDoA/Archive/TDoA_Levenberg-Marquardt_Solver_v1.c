// Several issues with this code; namely, the initial guesses are way off base and are probably worse than useless.

#include <stdio.h>     // For printf
#include <stdint.h>    // Include standard integer types
#include <math.h>      // Standard C math functions (sqrtf, powf, fabsf, atan2f, cosf, sinf, copysignf)
#include <string.h>    // For memcpy

// Define M_PI if it's not available by default
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Configuration ---
#define MAX_ITERATIONS 30      // Increased max iterations slightly
#define CONVERGENCE_THRESHOLD_CM 0.1f // Stop if change is less than 0.1 cm
#define ERROR_THRESHOLD 1e-12f // Stop if sum of squares error is extremely small
#define INITIAL_LAMBDA 0.01f  // Slightly higher initial damping
#define LAMBDA_UP_FACTOR 10.0f
#define LAMBDA_DOWN_FACTOR 10.0f
#define DEFAULT_INITIAL_GUESS_DISTANCE 2.0f // Default distance if quadratic fails

// --- Physical Constants & Geometry ---
const float SPEED_OF_SOUND_MPS = 343.0f;
const float MIC_BASELINE_L = 0.2f;
const float MIC1_X = 0.0f;       const float MIC1_Y = 0.11547f;
const float MIC2_X = -0.1f;      const float MIC2_Y = -0.057735f;
const float MIC3_X = 0.1f;       const float MIC3_Y = -0.057735f;
const float R1_SQ = MIC1_X * MIC1_X + MIC1_Y * MIC1_Y;
const float R2_SQ = MIC2_X * MIC2_X + MIC2_Y * MIC2_Y;
const float R3_SQ = MIC3_X * MIC3_X + MIC3_Y * MIC3_Y;

// --- Global Variables ---
float DELTA_D12_M; float DELTA_D13_M;
float true_x_m, true_y_m;

// --- Data Buffers ---
float current_pos[2]; float candidate_pos[2]; float delta_pos[2];
float residuals[2]; float jacobian[4]; float jacobian_T[4];
float JTJ[4]; float JTJ_damped[4]; float JTJ_damped_inv[4]; float JTF[2];

// --- Helper Functions --- (calculate_distance, calculate_total_error same as before)
float calculate_distance(float x, float y, float mic_x, float mic_y) {
    float dx = x - mic_x;
    float dy = y - mic_y;
    float dist_sq = dx * dx + dy * dy;
    return sqrtf(dist_sq < 1e-12f ? 1e-12f : dist_sq);
}
float calculate_total_error(float x, float y) {
    float r1 = calculate_distance(x, y, MIC1_X, MIC1_Y);
    float r2 = calculate_distance(x, y, MIC2_X, MIC2_Y);
    float r3 = calculate_distance(x, y, MIC3_X, MIC3_Y);
    float f1 = (r2 - r1) - DELTA_D12_M;
    float f2 = (r3 - r1) - DELTA_D13_M;
    return f1 * f1 + f2 * f2;
}
void calculate_ideal_delays(float test_x_m, float test_y_m) {
    true_x_m = test_x_m; true_y_m = test_y_m;
    printf("\nSetting up test case for source at (%.3f, %.3f) meters...\n", test_x_m, test_y_m);
    float r1 = calculate_distance(test_x_m, test_y_m, MIC1_X, MIC1_Y);
    float r2 = calculate_distance(test_x_m, test_y_m, MIC2_X, MIC2_Y);
    float r3 = calculate_distance(test_x_m, test_y_m, MIC3_X, MIC3_Y);
    printf("  True distances: r1=%.3fm, r2=%.3fm, r3=%.3fm\n", r1, r2, r3);
    DELTA_D12_M = r2 - r1; DELTA_D13_M = r3 - r1;
    printf("  True distance differences: d21=%.3fm, d31=%.3fm\n", DELTA_D12_M, DELTA_D13_M);
    if (fabsf(DELTA_D12_M) > MIC_BASELINE_L + 1e-6f || fabsf(DELTA_D13_M) > MIC_BASELINE_L + 1e-6f) {
         printf("  Warning: Distance difference potentially exceeds baseline.\n");
    }
}
float rad_to_deg(float radians) {
    float degrees = fmodf(radians * 180.0f / M_PI, 360.0f);
    if (degrees < 0) degrees += 360.0f;
    return degrees;
}

// --- Manual Matrix Functions --- (mat_transpose_2x2, mat_mult_2x2_2x2, mat_mult_2x2_2x1, mat_add_2x2, mat_inverse_2x2, vec_sub_2x1 same as before)
void mat_transpose_2x2(const float *A, float *AT) { AT[0]=A[0]; AT[1]=A[2]; AT[2]=A[1]; AT[3]=A[3]; }
void mat_mult_2x2_2x2(const float *A, const float *B, float *C) { C[0]=A[0]*B[0]+A[1]*B[2]; C[1]=A[0]*B[1]+A[1]*B[3]; C[2]=A[2]*B[0]+A[3]*B[2]; C[3]=A[2]*B[1]+A[3]*B[3]; }
void mat_mult_2x2_2x1(const float *A, const float *B, float *C) { C[0]=A[0]*B[0]+A[1]*B[1]; C[1]=A[2]*B[0]+A[3]*B[1]; }
void mat_add_2x2(const float *A, const float *B, float *C) { C[0]=A[0]+B[0]; C[1]=A[1]+B[1]; C[2]=A[2]+B[2]; C[3]=A[3]+B[3]; }
int mat_inverse_2x2(const float *A, float *A_inv) {
    float det = A[0]*A[3] - A[1]*A[2]; float det_inv; float epsilon = 1e-12f;
    if (fabsf(det) < epsilon) { printf("  Inside Inverse: Warning - Matrix singular! det = %e\n", det); return 1; }
    det_inv = 1.0f/det; A_inv[0]=A[3]*det_inv; A_inv[1]=-A[1]*det_inv; A_inv[2]=-A[2]*det_inv; A_inv[3]=A[0]*det_inv; return 0;
}
void vec_sub_2x1(const float *A, const float *B, float *C) { C[0]=A[0]-B[0]; C[1]=A[1]-B[1]; }


// --- Standard C Main Function ---
int main() {
    printf("===== TDoA Levenberg-Marquardt Solver Test (Float, Angle+R1 Guess) =====\n");

    // DEFINE TEST SOURCE LOCATION
    calculate_ideal_delays(-3.0f, 2.0f); // input test source location here

    // --- Calculate Initial Guess ---
    printf("\nCalculating Initial Guess...\n");
    float V21_x = MIC2_X - MIC1_X; float V21_y = MIC2_Y - MIC1_Y;
    float V31_x = MIC3_X - MIC1_X; float V31_y = MIC3_Y - MIC1_Y;
    float a = V21_x; float b = V21_y; float d1 = DELTA_D12_M;
    float c = V31_x; float d = V31_y; float d2 = DELTA_D13_M;
    float det_angle = a * d - b * c;
    float cos_theta, sin_theta;
    float initial_guess_angle_rad;

    if (fabsf(det_angle) < 1e-9f) {
        printf("  Error calculating angle guess determinant. Using default angle 0.\n");
        initial_guess_angle_rad = 0.0f;
    } else {
        cos_theta = (d * d1 - b * d2) / det_angle;
        sin_theta = (a * d2 - c * d1) / det_angle;
        float mag = sqrtf(cos_theta * cos_theta + sin_theta * sin_theta);
        if (mag > 1e-9f) { cos_theta /= mag; sin_theta /= mag; }
        initial_guess_angle_rad = atan2f(sin_theta, cos_theta);
        printf("  Initial angle guess: %.2f degrees\n", rad_to_deg(initial_guess_angle_rad));
    }

    // Estimate initial distance r1 using quadratic formula constants
    float initial_r1_estimate = DEFAULT_INITIAL_GUESS_DISTANCE; // Default
    float x21 = MIC2_X - MIC1_X; float y21 = MIC2_Y - MIC1_Y;
    float x31 = MIC3_X - MIC1_X; float y31 = MIC3_Y - MIC1_Y;
    float val1 = d1 * d1 - R2_SQ + R1_SQ;
    float val2 = d2 * d2 - R3_SQ + R1_SQ;
    float det_mic = x21 * y31 - x31 * y21;

    if (fabsf(det_mic) > 1e-9f) {
        float kx = 0.5f * (y31 * val1 - y21 * val2) / det_mic;
        float ky = 0.5f * (x21 * val2 - x31 * val1) / det_mic;
        float mx = (y31 * d1 - y21 * d2) / det_mic;
        float my = (x21 * d2 - x31 * d1) / det_mic;
        float A = mx * mx + my * my - 1.0f;
        float B = 2.0f * ((kx - MIC1_X) * mx + (ky - MIC1_Y) * my);
        float C = (kx - MIC1_X) * (kx - MIC1_X) + (ky - MIC1_Y) * (ky - MIC1_Y);
        float discriminant = B * B - 4.0f * A * C;

        if (discriminant >= 0 && fabsf(A) > 1e-9f) { // Check if quadratic is solvable
            float r1_sol1 = (-B + sqrtf(discriminant)) / (2.0f * A);
            float r1_sol2 = (-B - sqrtf(discriminant)) / (2.0f * A);
            printf("  Quadratic r1 estimates: %.3f, %.3f\n", r1_sol1, r1_sol2);
            // Simple selection: prefer positive solution, maybe closer one?
            if (r1_sol1 > 0 && r1_sol2 > 0) initial_r1_estimate = fminf(r1_sol1, r1_sol2); // Tentative
            else if (r1_sol1 > 0) initial_r1_estimate = r1_sol1;
            else if (r1_sol2 > 0) initial_r1_estimate = r1_sol2;
            else printf("  Quadratic solver yielded no positive r1, using default distance.\n");
        } else {
             printf("  Quadratic solver failed (discriminant=%.2e or A=%.2e), using default distance.\n", discriminant, A);
        }
    } else {
         printf("  Mic determinant zero, using default distance.\n");
    }
    printf("  Using initial distance estimate r1 = %.3f m\n", initial_r1_estimate);


    // Set initial position based on angle and estimated r1
    // Need to convert (r1, angle_relative_to_mic1) to (x, y) coordinates
    // Approximate: Treat r1 as distance from origin for initial guess
    current_pos[0] = initial_r1_estimate * cosf(initial_guess_angle_rad);
    current_pos[1] = initial_r1_estimate * sinf(initial_guess_angle_rad);
    // More accurate: calculate position relative to Mic1 then shift?
    // x = MIC1_X + r1*cos(angle_relative_to_mic1), y = ... complicated
    // Let's stick with origin-based guess for now.

    printf("Initial guess position: (x=%.3f, y=%.3f) meters\n", current_pos[0], current_pos[1]);
    // --- End Initial Guess Calculation ---


    float current_error = calculate_total_error(current_pos[0], current_pos[1]);
    float lambda = INITIAL_LAMBDA; // Damping factor

    // Levenberg-Marquardt Iteration Loop
    int iter;
    int success = 0; // Flag to indicate successful convergence
    for (iter = 0; iter < MAX_ITERATIONS; ++iter) {
        printf("\n--- Iteration %d (lambda = %e) ---\n", iter + 1, lambda);
        float xk = current_pos[0];
        float yk = current_pos[1];

        // 1. Calculate Distances r1, r2, r3
        float r1 = calculate_distance(xk, yk, MIC1_X, MIC1_Y);
        float r2 = calculate_distance(xk, yk, MIC2_X, MIC2_Y);
        float r3 = calculate_distance(xk, yk, MIC3_X, MIC3_Y);

        float epsilon = 1e-9f; // Use smaller epsilon for float distances
        if (r1 < epsilon) r1 = epsilon;
        if (r2 < epsilon) r2 = epsilon;
        if (r3 < epsilon) r3 = epsilon;

        // 2. Calculate Residuals f = [f1, f2]
        residuals[0] = (r2 - r1) - DELTA_D12_M;
        residuals[1] = (r3 - r1) - DELTA_D13_M;
        printf("  Current Error S(x)=%.6e\n", current_error); // Use %e for small errors
        printf("  Residuals: f1=%.3em, f2=%.3em\n", residuals[0], residuals[1]);

        // 3. Calculate Jacobian J
        float dx1 = xk - MIC1_X; float dy1 = yk - MIC1_Y;
        float dx2 = xk - MIC2_X; float dy2 = yk - MIC2_Y;
        float dx3 = xk - MIC3_X; float dy3 = yk - MIC3_Y;
        float r1_inv = 1.0f / r1;
        float r2_inv = 1.0f / r2;
        float r3_inv = 1.0f / r3;

        jacobian[0] = dx2 * r2_inv - dx1 * r1_inv; // df1/dx
        jacobian[1] = dy2 * r2_inv - dy1 * r1_inv; // df1/dy
        jacobian[2] = dx3 * r3_inv - dx1 * r1_inv; // df2/dx
        jacobian[3] = dy3 * r3_inv - dy1 * r1_inv; // df2/dy

        // 4. Calculate LM Update Step
        mat_transpose_2x2(jacobian, jacobian_T);
        mat_mult_2x2_2x2(jacobian_T, jacobian, JTJ);

        // Add damping: JTJ_damped = JTJ + lambda * diag(JTJ)
        JTJ_damped[0] = JTJ[0] * (1.0f + lambda);
        JTJ_damped[1] = JTJ[1];
        JTJ_damped[2] = JTJ[2];
        JTJ_damped[3] = JTJ[3] * (1.0f + lambda);

        if (mat_inverse_2x2(JTJ_damped, JTJ_damped_inv) != 0) {
            printf("Error Inverting Damped JTJ! Increasing lambda.\n");
            lambda *= LAMBDA_UP_FACTOR;
            if (lambda > 1e10) { printf("Lambda too large, aborting.\n"); break; }
            continue; // Retry iteration with higher lambda
        }

        mat_mult_2x2_2x1(jacobian_T, residuals, JTF);
        mat_mult_2x2_2x1(JTJ_damped_inv, JTF, delta_pos);

        // 5. Calculate Candidate Position
        vec_sub_2x1(current_pos, delta_pos, candidate_pos); // candidate = current - delta

        // 6. Evaluate Error at Candidate Position
        float candidate_error = calculate_total_error(candidate_pos[0], candidate_pos[1]);
        printf("  Candidate estimate: (x=%.3fm, y=%.3fm), Error S(cand)=%.6e\n",
               candidate_pos[0], candidate_pos[1], candidate_error);

        // 7. Accept or Reject Step & Adjust Lambda
        if (candidate_error < current_error) {
            printf("  Step Accepted. Decreasing lambda.\n");
            lambda /= LAMBDA_DOWN_FACTOR;
            current_error = candidate_error;
            memcpy(current_pos, candidate_pos, sizeof(current_pos)); // Update position

            float delta_mag_m = sqrtf(powf(delta_pos[0], 2) + powf(delta_pos[1], 2));
            float delta_mag_cm = delta_mag_m * 100.0f;
            printf("  Delta magnitude: %.3f cm\n", delta_mag_cm);

            if (fabsf(delta_mag_cm) < CONVERGENCE_THRESHOLD_CM || current_error < ERROR_THRESHOLD) {
                printf("\nConvergence reached after %d iterations.\n", iter + 1);
                success = 1; break;
            }
        } else {
            printf("  Step Rejected. Increasing lambda.\n");
            lambda *= LAMBDA_UP_FACTOR;
             if (lambda > 1e10) { printf("Lambda too large, aborting.\n"); break; }
        }

    } // End of iteration loop

    // --- Final Result ---
    printf("\n===== Final Result =====\n");
    if (success) {
        printf("Estimated source location (meters): (x = %.3f, y = %.3f)\n", current_pos[0], current_pos[1]);
    } else {
         if (iter == MAX_ITERATIONS) printf("Warning: Maximum iterations reached without convergence.\n");
         else printf("Solver aborted.\n"); // e.g. if lambda got too large
        printf("Last estimated location (meters): (x = %.3f, y = %.3f)\n", current_pos[0], current_pos[1]);
    }
    printf("Expected source location (meters): (x = %.3f, y = %.3f)\n", true_x_m, true_y_m);

    // Calculate angles for comparison
    float true_angle_rad = atan2f(true_y_m, true_x_m);
    if (true_angle_rad < 0) true_angle_rad += 2.0f * M_PI;
    printf("Expected source angle: %.2f degrees\n", rad_to_deg(true_angle_rad));

    float estimated_angle_rad = atan2f(current_pos[1], current_pos[0]);
    if (estimated_angle_rad < 0) estimated_angle_rad += 2.0f * M_PI;
    printf("Estimated source angle: %.2f degrees\n", rad_to_deg(estimated_angle_rad));

    return 0;
}
