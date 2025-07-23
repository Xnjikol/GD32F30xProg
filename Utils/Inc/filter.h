// 滤波器头文件
#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdbool.h>
#include <stdint.h>

#include "gd32f30x.h"

#define MAX_FILTER_SIZE 32

/**
 * @brief Low pass filter structure
 */
typedef struct
{
    float alpha;        // Filter coefficient
    float prev_output;  // Previous output value
    bool initialized;   // Initialization flag
} LowPassFilter_t;

/**
 * @brief Moving average filter structure
 */
typedef struct
{
    float buffer[MAX_FILTER_SIZE];  // Circular buffer
    float sum;                      // Sum of values in buffer
    uint16_t window_size;           // Size of moving window
    uint16_t index;                 // Current index in buffer
    uint16_t count;                 // Number of valid samples
} MovingAverageFilter_t;

/**
 * @brief Median filter structure
 */
typedef struct
{
    float buffer[MAX_FILTER_SIZE];  // Circular buffer
    uint16_t window_size;           // Size of filter window
    uint16_t index;                 // Current index in buffer
    uint16_t count;                 // Number of valid samples
} MedianFilter_t;

/**
 * @brief Simple 1D Kalman filter structure
 */
typedef struct
{
    float x;  // State estimate
    float P;  // Estimate uncertainty
    float Q;  // Process noise variance
    float R;  // Measurement noise variance
    float K;  // Kalman gain
} KalmanFilter_t;

/**
 * @brief Band pass filter structure (using two cascaded filters)
 */
typedef struct
{
    LowPassFilter_t low_pass;   // Low pass stage
    float high_pass_alpha;      // High pass coefficient
    float high_pass_prev_input; // Previous input for high pass
    float high_pass_prev_output;// Previous output for high pass
    bool initialized;           // Initialization flag
} BandPassFilter_t;

/**
 * @brief High pass filter structure
 */
typedef struct
{
    float alpha;        // Filter coefficient
    float prev_input;   // Previous input value
    float prev_output;  // Previous output value
    bool initialized;   // Initialization flag
} HighPassFilter_t;

/**
 * @brief Band stop (notch) filter structure
 */
typedef struct
{
    LowPassFilter_t low_pass;   // Low pass path
    HighPassFilter_t high_pass; // High pass path
    float low_gain;             // Low frequency gain
    float high_gain;            // High frequency gain
} BandStopFilter_t;

// Low pass filter functions
void LowPassFilter_Init(LowPassFilter_t* filter, float cutoff_freq, float sample_freq);
float LowPassFilter_Update(LowPassFilter_t* filter, float input);
void LowPassFilter_Reset(LowPassFilter_t* filter);

// Moving average filter functions
void MovingAverageFilter_Init(MovingAverageFilter_t* filter, uint16_t window_size);
float MovingAverageFilter_Update(MovingAverageFilter_t* filter, float input);
void MovingAverageFilter_Reset(MovingAverageFilter_t* filter);

// Median filter functions
void MedianFilter_Init(MedianFilter_t* filter, uint16_t window_size);
float MedianFilter_Update(MedianFilter_t* filter, float input);
void MedianFilter_Reset(MedianFilter_t* filter);

// Kalman filter functions
void KalmanFilter_Init(KalmanFilter_t* filter, float process_noise, float measurement_noise,
                       float initial_value, float initial_uncertainty);
void KalmanFilter_InitSimple(KalmanFilter_t* filter, float process_noise, float measurement_noise,
                             float initial_value);
float KalmanFilter_Update(KalmanFilter_t* filter, float measurement);
float KalmanFilter_GetUncertainty(KalmanFilter_t* filter);
void KalmanFilter_Reset(KalmanFilter_t* filter, float initial_value);

// High pass filter functions
void HighPassFilter_Init(HighPassFilter_t* filter, float cutoff_freq, float sample_freq);
float HighPassFilter_Update(HighPassFilter_t* filter, float input);
void HighPassFilter_Reset(HighPassFilter_t* filter);

// Band pass filter functions
void BandPassFilter_Init(BandPassFilter_t* filter, float low_cutoff, float high_cutoff, float sample_freq);
float BandPassFilter_Update(BandPassFilter_t* filter, float input);
void BandPassFilter_Reset(BandPassFilter_t* filter);

// Band stop (notch) filter functions
void BandStopFilter_Init(BandStopFilter_t* filter, float low_cutoff, float high_cutoff, float sample_freq);
float BandStopFilter_Update(BandStopFilter_t* filter, float input);
void BandStopFilter_Reset(BandStopFilter_t* filter);

#endif /* __FILTER_H__ */
