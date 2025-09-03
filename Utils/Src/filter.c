#include "filter.h"

#include <string.h>

/**
 * @brief Initialize low pass filter with optimized coefficient calculation
 * @param filter: pointer to LowPassFilter_t structure
 * @param cutoff_freq: cutoff frequency in Hz
 * @param sample_freq: sampling frequency in Hz
 */
void LowPassFilter_Init(LowPassFilter_t* filter,
                        float            cutoff_freq,
                        float            sample_freq) {
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
void LowPassFilter_Reset(LowPassFilter_t* filter) {
    if (filter == NULL)
        return;

    filter->prev_output = 0.0f;
    filter->initialized = false;
}

/**
 * @brief Initialize a band-pass filter (cascaded high-pass and low-pass).
 * @param filter Pointer to BandPassFilter_t structure to initialize.
 * @param sampleRate Sampling frequency in Hz.
 * @param centerFreq Center frequency of the band-pass filter in Hz.
 * @param bandwidth Bandwidth of the band-pass filter in Hz.
 */
void BandPassFilter_Init(BandPassFilter_t* filter,
                         float             sampleRate,
                         float             centerFreq,
                         float             bandwidth) {
    if (filter->initialized) {
        return;  // 如果已经初始化，直接返回
    }
    // 确保输入参数有效
    if (filter == NULL || sampleRate <= 0 || centerFreq <= 0
        || bandwidth <= 0 || centerFreq - bandwidth / 2 < 0
        || centerFreq + bandwidth / 2 > sampleRate / 2) {
        // 无效参数，初始化滤波器为直通状态
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
        filter->x1 = 0.0f;
        filter->x2 = 0.0f;
        filter->y1 = 0.0f;
        filter->y2 = 0.0f;
        return;
    }

    // 计算二阶巴特沃斯带通滤波器系数
    float Q      = centerFreq / bandwidth;    // 品质因数
    float omega0 = 2.0F * M_PI * centerFreq;  // 中心角频率
    float T      = 1.0F / sampleRate;         // 采样周期
    float alpha  = SIN(omega0 * T) / (2.0F * Q);

    float denominator = 1.0f + alpha;

    // 计算滤波器系数
    filter->b0 = alpha / denominator;
    filter->b1 = 0.0F;
    filter->b2 = -alpha / denominator;
    filter->a1 = (-2.0F * cosf(omega0 * T)) / denominator;
    filter->a2 = (1.0F - alpha) / denominator;

    // 初始化延迟线
    filter->x1 = 0.0F;
    filter->x2 = 0.0F;
    filter->y1 = 0.0F;
    filter->y2 = 0.0F;
}

/**
 * @brief Reset band pass filter state
 * @param filter: pointer to BandPassFilter_t structure
 */
void BandPassFilter_Reset(BandPassFilter_t* filter) {
    if (filter != NULL) {
        // 将所有延迟线值重置为0
        filter->x1          = 0.0f;
        filter->x2          = 0.0f;
        filter->y1          = 0.0f;
        filter->y2          = 0.0f;
        filter->initialized = false;  // 重置初始化标志
    }
}

// ================== BAND STOP FILTER IMPLEMENTATION ==================

/**
 * @brief Initialize band stop (notch) filter
 * @param filter: pointer to BandStopFilter_t structure
 * @param low_cutoff: lower cutoff frequency in Hz (start of stop band)
 * @param high_cutoff: upper cutoff frequency in Hz (end of stop band)
 * @param sample_freq: sampling frequency in Hz
 */
void BandStopFilter_Init(BandStopFilter_t* filter,
                         float             sampleRate,
                         float             centerFreq,
                         float             bandwidth) {
    // 确保输入参数有效
    if (filter == NULL || sampleRate <= 0 || centerFreq <= 0
        || bandwidth <= 0 || centerFreq - bandwidth / 2 < 0
        || centerFreq + bandwidth / 2 > sampleRate / 2) {
        // 无效参数，初始化滤波器为直通状态
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
        BandStopFilter_Reset(filter);  // 调用重置函数初始化状态
        return;
    }

    // 计算二阶巴特沃斯带阻滤波器系数
    float Q      = centerFreq / bandwidth;    // 品质因数
    float omega0 = 2.0f * M_PI * centerFreq;  // 中心角频率
    float T      = 1.0f / sampleRate;         // 采样周期
    float alpha  = sinf(omega0 * T) / (2.0f * Q);

    float denominator = 1.0f + alpha;

    // 计算滤波器系数（带阻滤波器系数与带通不同）
    filter->b0 = 1.0f / denominator;
    filter->b1 = (-2.0f * cosf(omega0 * T)) / denominator;
    filter->b2 = 1.0f / denominator;
    filter->a1 = (-2.0f * cosf(omega0 * T)) / denominator;
    filter->a2 = (1.0f - alpha) / denominator;

    // 初始化延迟线
    BandStopFilter_Reset(filter);  // 调用重置函数初始化状态
}

/**
 * @brief Reset band stop filter state
 * @param filter: pointer to BandStopFilter_t structure
 */
void BandStopFilter_Reset(BandStopFilter_t* filter) {
    if (filter != NULL) {
        // 将所有延迟线值重置为0
        filter->x1 = 0.0f;
        filter->x2 = 0.0f;
        filter->y1 = 0.0f;
        filter->y2 = 0.0f;
    }
}
