#include "filter.h"

#include <string.h>


// Mathematical constants
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Initialize low pass filter with optimized coefficient calculation
 * @param filter: pointer to LowPassFilter_t structure
 * @param cutoff_freq: cutoff frequency in Hz
 * @param sample_freq: sampling frequency in Hz
 */
void LowPassFilter_Init(LowPassFilter_t* filter, float cutoff_freq, float sample_freq)
{
  if (filter == NULL || cutoff_freq <= 0.0f || sample_freq <= 0.0f)
    return;

  // Calculate RC time constant and sampling period
  float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
  float dt = 1.0f / sample_freq;

  // Calculate alpha coefficient using optimized formula
  filter->alpha = dt / (rc + dt);

  // Clamp alpha to valid range [0, 1] for stability
  if (filter->alpha > 1.0f)
    filter->alpha = 1.0f;
  if (filter->alpha < 0.0f)
    filter->alpha = 0.0f;

  filter->prev_output = 0.0f;
  filter->initialized = true;
}

/**
 * @brief Reset low pass filter state
 * @param filter: pointer to LowPassFilter_t structure
 */
void LowPassFilter_Reset(LowPassFilter_t* filter)
{
  if (filter == NULL)
    return;

  filter->prev_output = 0.0f;
  filter->initialized = false;
}

/**
 * @brief Initialize moving average filter with bounds checking
 * @param filter: pointer to MovingAverageFilter_t structure
 * @param window_size: size of the moving window
 */
void MovingAverageFilter_Init(MovingAverageFilter_t* filter, uint16_t window_size)
{
  if (filter == NULL || window_size == 0 || window_size > MAX_FILTER_SIZE)
    return;

  filter->window_size = window_size;
  filter->index = 0;
  filter->sum = 0.0f;
  filter->count = 0;

  // Initialize buffer to zero for consistent behavior
  memset(filter->buffer, 0, sizeof(filter->buffer));
}

/**
 * @brief Update moving average filter with optimized circular buffer
 * @param filter: pointer to MovingAverageFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
float MovingAverageFilter_Update(MovingAverageFilter_t* filter, float input)
{
  if (filter == NULL)
    return input;

  // Subtract the value being replaced (only when buffer is full)
  if (filter->count >= filter->window_size)
  {
    filter->sum -= filter->buffer[filter->index];
  }
  else
  {
    filter->count++;
  }

  // Add new value to buffer and sum
  filter->buffer[filter->index] = input;
  filter->sum += input;

  // Update circular buffer index
  filter->index = (filter->index + 1) % filter->window_size;

  // Return average (optimized division)
  return filter->sum / (float) filter->count;
}

/**
 * @brief Reset moving average filter
 * @param filter: pointer to MovingAverageFilter_t structure
 */
void MovingAverageFilter_Reset(MovingAverageFilter_t* filter)
{
  if (filter == NULL)
    return;

  filter->index = 0;
  filter->sum = 0.0f;
  filter->count = 0;
  memset(filter->buffer, 0, sizeof(filter->buffer));
}

/**
 * @brief Initialize median filter with validation
 * @param filter: pointer to MedianFilter_t structure
 * @param window_size: size of the filter window
 */
void MedianFilter_Init(MedianFilter_t* filter, uint16_t window_size)
{
  if (filter == NULL || window_size == 0 || window_size > MAX_FILTER_SIZE)
    return;

  // Ensure odd window size for proper median calculation
  if (window_size % 2 == 0)
  {
    window_size++;
    if (window_size > MAX_FILTER_SIZE)
    {
      window_size = MAX_FILTER_SIZE;
      if (window_size % 2 == 0)
        window_size--;
    }
  }

  filter->window_size = window_size;
  filter->index = 0;
  filter->count = 0;
  memset(filter->buffer, 0, sizeof(filter->buffer));
}

/**
 * @brief Fast insertion sort optimized for small arrays (used in median filter)
 * @param arr: array to sort
 * @param size: size of array
 */
static void fast_insertion_sort(float* arr, uint16_t size)
{
  uint16_t i, j;
  float key;

  for (i = 1; i < size; i++)
  {
    key = arr[i];
    j = i;

    // Optimized inner loop with early termination
    while (j > 0 && arr[j - 1] > key)
    {
      arr[j] = arr[j - 1];
      j--;
    }
    arr[j] = key;
  }
}

/**
 * @brief Update median filter with optimized sorting
 * @param filter: pointer to MedianFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
float MedianFilter_Update(MedianFilter_t* filter, float input)
{
  if (filter == NULL)
    return input;

  // Add new value to circular buffer
  filter->buffer[filter->index] = input;
  filter->index = (filter->index + 1) % filter->window_size;

  if (filter->count < filter->window_size)
  {
    filter->count++;
  }

  // Create temporary array for sorting (stack allocated for speed)
  float temp_buffer[MAX_FILTER_SIZE];

  // Copy only valid data points
  for (uint16_t i = 0; i < filter->count; i++)
  {
    temp_buffer[i] = filter->buffer[i];
  }

  // Sort using optimized insertion sort (good for small arrays)
  fast_insertion_sort(temp_buffer, filter->count);

  // Return median value
  return temp_buffer[filter->count / 2];
}

/**
 * @brief Reset median filter
 * @param filter: pointer to MedianFilter_t structure
 */
void MedianFilter_Reset(MedianFilter_t* filter)
{
  if (filter == NULL)
    return;

  filter->index = 0;
  filter->count = 0;
  memset(filter->buffer, 0, sizeof(filter->buffer));
}

/**
 * @brief Initialize 1D Kalman filter with parameter validation
 * @param filter: pointer to KalmanFilter_t structure
 * @param process_noise: Q - process noise variance
 * @param measurement_noise: R - measurement noise variance
 * @param initial_value: initial state estimate
 * @param initial_uncertainty: initial estimate uncertainty
 */
void KalmanFilter_Init(KalmanFilter_t* filter, float process_noise, float measurement_noise,
                       float initial_value, float initial_uncertainty)
{
  if (filter == NULL || process_noise <= 0.0f || measurement_noise <= 0.0f)
    return;

  filter->x = initial_value;        // Initial state estimate
  filter->P = initial_uncertainty;  // Initial estimate uncertainty
  filter->Q = process_noise;        // Process noise variance
  filter->R = measurement_noise;    // Measurement noise variance
  filter->K = 0.0f;                 // Kalman gain (will be calculated)
}

/**
 * @brief Initialize Kalman filter with default uncertainty
 * @param filter: pointer to KalmanFilter_t structure
 * @param process_noise: Q - process noise variance
 * @param measurement_noise: R - measurement noise variance
 * @param initial_value: initial state estimate
 */
void KalmanFilter_InitSimple(KalmanFilter_t* filter, float process_noise, float measurement_noise,
                             float initial_value)
{
  KalmanFilter_Init(filter, process_noise, measurement_noise, initial_value, 1.0f);
}

/**
 * @brief Update Kalman filter with improved numerical stability
 * @param filter: pointer to KalmanFilter_t structure
 * @param measurement: measured value
 * @return filtered output value
 */
float KalmanFilter_Update(KalmanFilter_t* filter, float measurement)
{
  if (filter == NULL)
    return measurement;

  // Prediction step
  // State prediction: x_pred = x (constant velocity model)
  // Covariance prediction: P_pred = P + Q
  filter->P += filter->Q;

  // Update step
  // Innovation covariance: S = P_pred + R
  float S = filter->P + filter->R;

  // Avoid division by zero
  if (S <= 0.0f)
  {
    S = 1e-6f;  // Small positive value for numerical stability
  }

  // Kalman gain: K = P_pred / S
  filter->K = filter->P / S;

  // State update: x = x_pred + K * (measurement - x_pred)
  float innovation = measurement - filter->x;
  filter->x += filter->K * innovation;

  // Covariance update: P = (1 - K) * P_pred (Joseph form for numerical stability)
  filter->P = (1.0f - filter->K) * filter->P;

  // Ensure covariance doesn't become negative
  if (filter->P < 0.0f)
  {
    filter->P = 0.0f;
  }

  return filter->x;
}

/**
 * @brief Fast square root approximation using Newton-Raphson method
 * @param x: input value
 * @return approximate square root
 */
static float fast_sqrt(float x)
{
  if (x <= 0.0f)
    return 0.0f;

  // Initial guess using bit manipulation (optional optimization)
  float result = x * 0.5f;

  // Newton-Raphson iterations (2-3 iterations usually sufficient)
  for (int i = 0; i < 3; i++)
  {
    result = 0.5f * (result + x / result);
  }

  return result;
}

/**
 * @brief Get current Kalman filter uncertainty
 * @param filter: pointer to KalmanFilter_t structure
 * @return current uncertainty (standard deviation)
 */
float KalmanFilter_GetUncertainty(KalmanFilter_t* filter)
{
  if (filter == NULL)
    return 0.0f;

  // Return square root of variance for standard deviation
  return (filter->P > 0.0f) ? fast_sqrt(filter->P) : 0.0f;
}

/**
 * @brief Reset Kalman filter to initial state
 * @param filter: pointer to KalmanFilter_t structure
 * @param initial_value: new initial state estimate
 */
void KalmanFilter_Reset(KalmanFilter_t* filter, float initial_value)
{
  if (filter == NULL)
    return;

  filter->x = initial_value;
  filter->P = 1.0f;  // Reset to default uncertainty
  filter->K = 0.0f;
}

// ================== HIGH PASS FILTER IMPLEMENTATION ==================

/**
 * @brief Initialize high pass filter
 * @param filter: pointer to HighPassFilter_t structure
 * @param cutoff_freq: cutoff frequency in Hz
 * @param sample_freq: sampling frequency in Hz
 */
void HighPassFilter_Init(HighPassFilter_t* filter, float cutoff_freq, float sample_freq)
{
  if (filter == NULL || cutoff_freq <= 0.0f || sample_freq <= 0.0f)
    return;

  // Calculate RC time constant and sampling period
  float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
  float dt = 1.0f / sample_freq;

  // Calculate alpha coefficient for high pass filter
  filter->alpha = rc / (rc + dt);

  // Clamp alpha to valid range [0, 1] for stability
  if (filter->alpha > 1.0f)
    filter->alpha = 1.0f;
  if (filter->alpha < 0.0f)
    filter->alpha = 0.0f;

  filter->prev_input = 0.0f;
  filter->prev_output = 0.0f;
  filter->initialized = false;
}

/**
 * @brief Update high pass filter
 * @param filter: pointer to HighPassFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
float HighPassFilter_Update(HighPassFilter_t* filter, float input)
{
  if (filter == NULL)
    return input;

  // Initialize with first input value
  if (!filter->initialized)
  {
    filter->prev_input = input;
    filter->prev_output = 0.0f;
    filter->initialized = true;
    return 0.0f;
  }

  // High pass filter equation: y[n] = α * (y[n-1] + x[n] - x[n-1])
  filter->prev_output = filter->alpha * (filter->prev_output + input - filter->prev_input);
  filter->prev_input = input;

  return filter->prev_output;
}

/**
 * @brief Reset high pass filter state
 * @param filter: pointer to HighPassFilter_t structure
 */
void HighPassFilter_Reset(HighPassFilter_t* filter)
{
  if (filter == NULL)
    return;

  filter->prev_input = 0.0f;
  filter->prev_output = 0.0f;
  filter->initialized = false;
}

// ================== BAND PASS FILTER IMPLEMENTATION ==================

/**
 * @brief Initialize band pass filter (cascaded high-pass and low-pass)
 * @param filter: pointer to BandPassFilter_t structure
 * @param low_cutoff: lower cutoff frequency in Hz
 * @param high_cutoff: upper cutoff frequency in Hz
 * @param sample_freq: sampling frequency in Hz
 */
void BandPassFilter_Init(BandPassFilter_t* filter, float low_cutoff, float high_cutoff,
                         float sample_freq)
{
  if (filter == NULL || low_cutoff <= 0.0f || high_cutoff <= low_cutoff || sample_freq <= 0.0f)
    return;

  // Initialize low pass filter with high cutoff frequency
  LowPassFilter_Init(&filter->low_pass, high_cutoff, sample_freq);

  // Calculate high pass coefficient for low cutoff frequency
  float rc = 1.0f / (2.0f * M_PI * low_cutoff);
  float dt = 1.0f / sample_freq;
  filter->high_pass_alpha = rc / (rc + dt);

  // Clamp alpha to valid range
  if (filter->high_pass_alpha > 1.0f)
    filter->high_pass_alpha = 1.0f;
  if (filter->high_pass_alpha < 0.0f)
    filter->high_pass_alpha = 0.0f;

  filter->high_pass_prev_input = 0.0f;
  filter->high_pass_prev_output = 0.0f;
  filter->initialized = false;
}

/**
 * @brief Update band pass filter
 * @param filter: pointer to BandPassFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
float BandPassFilter_Update(BandPassFilter_t* filter, float input)
{
  if (filter == NULL)
    return input;

  // First apply low pass filter
  float low_pass_output = LowPassFilter_Update(&filter->low_pass, input);

  // Initialize high pass stage with first value
  if (!filter->initialized)
  {
    filter->high_pass_prev_input = low_pass_output;
    filter->high_pass_prev_output = 0.0f;
    filter->initialized = true;
    return 0.0f;
  }

  // Then apply high pass filter
  filter->high_pass_prev_output =
      filter->high_pass_alpha *
      (filter->high_pass_prev_output + low_pass_output - filter->high_pass_prev_input);
  filter->high_pass_prev_input = low_pass_output;

  return filter->high_pass_prev_output;
}

/**
 * @brief Reset band pass filter state
 * @param filter: pointer to BandPassFilter_t structure
 */
void BandPassFilter_Reset(BandPassFilter_t* filter)
{
  if (filter == NULL)
    return;

  LowPassFilter_Reset(&filter->low_pass);
  filter->high_pass_prev_input = 0.0f;
  filter->high_pass_prev_output = 0.0f;
  filter->initialized = false;
}

// ================== BAND STOP FILTER IMPLEMENTATION ==================

/**
 * @brief Initialize band stop (notch) filter
 * @param filter: pointer to BandStopFilter_t structure
 * @param low_cutoff: lower cutoff frequency in Hz (start of stop band)
 * @param high_cutoff: upper cutoff frequency in Hz (end of stop band)
 * @param sample_freq: sampling frequency in Hz
 */
void BandStopFilter_Init(BandStopFilter_t* filter, float low_cutoff, float high_cutoff,
                         float sample_freq)
{
  if (filter == NULL || low_cutoff <= 0.0f || high_cutoff <= low_cutoff || sample_freq <= 0.0f)
    return;

  // Initialize low pass filter with low cutoff (passes low frequencies)
  LowPassFilter_Init(&filter->low_pass, low_cutoff, sample_freq);

  // Initialize high pass filter with high cutoff (passes high frequencies)
  HighPassFilter_Init(&filter->high_pass, high_cutoff, sample_freq);

  // Set equal gains for both paths
  filter->low_gain = 0.5f;
  filter->high_gain = 0.5f;
}

/**
 * @brief Update band stop filter (parallel low-pass and high-pass combination)
 * @param filter: pointer to BandStopFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
float BandStopFilter_Update(BandStopFilter_t* filter, float input)
{
  if (filter == NULL)
    return input;

  // Process input through both parallel paths
  float low_pass_output = LowPassFilter_Update(&filter->low_pass, input);
  float high_pass_output = HighPassFilter_Update(&filter->high_pass, input);

  // Combine outputs with weighted sum
  return filter->low_gain * low_pass_output + filter->high_gain * high_pass_output;
}

/**
 * @brief Reset band stop filter state
 * @param filter: pointer to BandStopFilter_t structure
 */
void BandStopFilter_Reset(BandStopFilter_t* filter)
{
  if (filter == NULL)
    return;

  LowPassFilter_Reset(&filter->low_pass);
  HighPassFilter_Reset(&filter->high_pass);
}