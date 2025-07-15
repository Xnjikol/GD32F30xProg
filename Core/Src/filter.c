#include "filter.h"
#include <string.h>
#include <math.h>

/* 添加必要的常量定义 */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

#ifndef SQRT2
#define SQRT2 1.41421356237309504880f
#endif

/*======================*/
/*   Private Functions  */
/*======================*/
static void quicksort(float *arr, int low, int high);
static int partition(float *arr, int low, int high);
static void swap(float *a, float *b);

/*======================*/
/*   Low Pass Filter    */
/*======================*/

/**
 * @brief 低通滤波器初始化
 * @param filter 滤波器结构体指针
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_time 采样时间 (s)
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float cutoff_freq, float sample_time)
{
    if (cutoff_freq < FILTER_MIN_CUTOFF) cutoff_freq = FILTER_MIN_CUTOFF;
    if (cutoff_freq > FILTER_MAX_CUTOFF) cutoff_freq = FILTER_MAX_CUTOFF;
    
    float omega = 2.0f * M_PI * cutoff_freq;
    filter->a = omega * sample_time / (1.0f + omega * sample_time);
    filter->x_last = 0.0f;
    filter->y_last = 0.0f;
}

/**
 * @brief 低通滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float LowPassFilter_Update(LowPassFilter_t *filter, float input)
{
    float output = filter->a * input + (1.0f - filter->a) * filter->y_last;
    filter->x_last = input;
    filter->y_last = output;
    return output;
}

/**
 * @brief 低通滤波器重置
 * @param filter 滤波器结构体指针
 */
void LowPassFilter_Reset(LowPassFilter_t *filter)
{
    filter->x_last = 0.0f;
    filter->y_last = 0.0f;
}

/*======================*/
/*   High Pass Filter   */
/*======================*/

/**
 * @brief 高通滤波器初始化
 * @param filter 滤波器结构体指针
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_time 采样时间 (s)
 */
void HighPassFilter_Init(HighPassFilter_t *filter, float cutoff_freq, float sample_time)
{
    if (cutoff_freq < FILTER_MIN_CUTOFF) cutoff_freq = FILTER_MIN_CUTOFF;
    if (cutoff_freq > FILTER_MAX_CUTOFF) cutoff_freq = FILTER_MAX_CUTOFF;
    
    float omega = 2.0f * M_PI * cutoff_freq;
    filter->a = 1.0f / (1.0f + omega * sample_time);
    filter->x_last = 0.0f;
    filter->y_last = 0.0f;
}

/**
 * @brief 高通滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float HighPassFilter_Update(HighPassFilter_t *filter, float input)
{
    float output = filter->a * (filter->y_last + input - filter->x_last);
    filter->x_last = input;
    filter->y_last = output;
    return output;
}

/**
 * @brief 高通滤波器重置
 * @param filter 滤波器结构体指针
 */
void HighPassFilter_Reset(HighPassFilter_t *filter)
{
    filter->x_last = 0.0f;
    filter->y_last = 0.0f;
}

/*======================*/
/*   Band Pass Filter   */
/*======================*/

/**
 * @brief 带通滤波器初始化
 * @param filter 滤波器结构体指针
 * @param center_freq 中心频率 (Hz)
 * @param bandwidth 带宽 (Hz)
 * @param sample_time 采样时间 (s)
 */
void BandPassFilter_Init(BandPassFilter_t *filter, float center_freq, float bandwidth, float sample_time)
{
    if (center_freq < FILTER_MIN_CUTOFF) center_freq = FILTER_MIN_CUTOFF;
    if (center_freq > FILTER_MAX_CUTOFF) center_freq = FILTER_MAX_CUTOFF;
    
    float omega = 2.0f * M_PI * center_freq;
    float Q = center_freq / bandwidth;
    float norm = 1.0f / (1.0f + omega * sample_time / Q + omega * omega * sample_time * sample_time);
    
    filter->a1 = 2.0f * (omega * omega * sample_time * sample_time - 1.0f) * norm;
    filter->a2 = (1.0f - omega * sample_time / Q + omega * omega * sample_time * sample_time) * norm;
    filter->b1 = omega * sample_time / Q * norm;
    filter->b2 = -omega * sample_time / Q * norm;
    
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

/**
 * @brief 带通滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float BandPassFilter_Update(BandPassFilter_t *filter, float input)
{
    float output = filter->b1 * input + filter->b2 * filter->x1 - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}

/**
 * @brief 带通滤波器重置
 * @param filter 滤波器结构体指针
 */
void BandPassFilter_Reset(BandPassFilter_t *filter)
{
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

/*======================*/
/*   Notch Filter       */
/*======================*/

/**
 * @brief 陷波滤波器初始化
 * @param filter 滤波器结构体指针
 * @param notch_freq 陷波频率 (Hz)
 * @param bandwidth 带宽 (Hz)
 * @param sample_time 采样时间 (s)
 */
void NotchFilter_Init(NotchFilter_t *filter, float notch_freq, float bandwidth, float sample_time)
{
    if (notch_freq < FILTER_MIN_CUTOFF) notch_freq = FILTER_MIN_CUTOFF;
    if (notch_freq > FILTER_MAX_CUTOFF) notch_freq = FILTER_MAX_CUTOFF;
    
    float omega = 2.0f * M_PI * notch_freq;
    float Q = notch_freq / bandwidth;
    float norm = 1.0f / (1.0f + omega * sample_time / Q + omega * omega * sample_time * sample_time);
    
    filter->b0 = (1.0f + omega * omega * sample_time * sample_time) * norm;
    filter->b1 = 2.0f * (omega * omega * sample_time * sample_time - 1.0f) * norm;
    filter->b2 = filter->b0;
    filter->a1 = filter->b1;
    filter->a2 = (1.0f - omega * sample_time / Q + omega * omega * sample_time * sample_time) * norm;
    
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

/**
 * @brief 陷波滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float NotchFilter_Update(NotchFilter_t *filter, float input)
{
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 
                   - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}

/**
 * @brief 陷波滤波器重置
 * @param filter 滤波器结构体指针
 */
void NotchFilter_Reset(NotchFilter_t *filter)
{
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

/*======================*/
/* Moving Average Filter */
/*======================*/

/**
 * @brief 移动平均滤波器初始化
 * @param filter 滤波器结构体指针
 * @param buffer 数据缓冲区
 * @param size 缓冲区大小
 */
void MovingAverageFilter_Init(MovingAverageFilter_t *filter, float *buffer, uint16_t size)
{
    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->initialized = 0;
    
    /* 清空缓冲区 */
    memset(buffer, 0, size * sizeof(float));
}

/**
 * @brief 移动平均滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float MovingAverageFilter_Update(MovingAverageFilter_t *filter, float input)
{
    /* 减去即将被替换的值 */
    filter->sum -= filter->buffer[filter->index];
    
    /* 添加新值 */
    filter->buffer[filter->index] = input;
    filter->sum += input;
    
    /* 更新索引 */
    filter->index = (filter->index + 1) % filter->size;
    
    /* 如果还没有填满缓冲区，计算当前有效数据的平均值 */
    if (!filter->initialized) {
        if (filter->index == 0) {
            filter->initialized = 1;
        }
        return filter->sum / (filter->index == 0 ? filter->size : filter->index);
    }
    
    return filter->sum / filter->size;
}

/**
 * @brief 移动平均滤波器重置
 * @param filter 滤波器结构体指针
 */
void MovingAverageFilter_Reset(MovingAverageFilter_t *filter)
{
    filter->index = 0;
    filter->sum = 0.0f;
    filter->initialized = 0;
    memset(filter->buffer, 0, filter->size * sizeof(float));
}

/*======================*/
/*   Median Filter      */
/*======================*/

/**
 * @brief 中值滤波器初始化
 * @param filter 滤波器结构体指针
 * @param buffer 数据缓冲区
 * @param size 缓冲区大小
 */
void MedianFilter_Init(MedianFilter_t *filter, float *buffer, uint16_t size)
{
    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0;
    filter->initialized = 0;
    
    /* 清空缓冲区 */
    memset(buffer, 0, size * sizeof(float));
}

/**
 * @brief 中值滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float MedianFilter_Update(MedianFilter_t *filter, float input)
{
    /* 添加新值 */
    filter->buffer[filter->index] = input;
    filter->index = (filter->index + 1) % filter->size;
    
    /* 创建临时数组进行排序 */
    float temp[filter->size];
    memcpy(temp, filter->buffer, filter->size * sizeof(float));
    
    /* 确定有效数据长度 */
    uint16_t valid_length = filter->size;
    if (!filter->initialized) {
        if (filter->index == 0) {
            filter->initialized = 1;
        } else {
            valid_length = filter->index;
        }
    }
    
    /* 排序 */
    quicksort(temp, 0, valid_length - 1);
    
    /* 返回中值 */
    if (valid_length % 2 == 0) {
        return (temp[valid_length / 2 - 1] + temp[valid_length / 2]) / 2.0f;
    } else {
        return temp[valid_length / 2];
    }
}

/**
 * @brief 中值滤波器重置
 * @param filter 滤波器结构体指针
 */
void MedianFilter_Reset(MedianFilter_t *filter)
{
    filter->index = 0;
    filter->initialized = 0;
    memset(filter->buffer, 0, filter->size * sizeof(float));
}

/*======================*/
/*   Kalman Filter      */
/*======================*/

/**
 * @brief 卡尔曼滤波器初始化
 * @param filter 滤波器结构体指针
 * @param q 过程噪声方差
 * @param r 测量噪声方差
 * @param initial_value 初始值
 */
void KalmanFilter_Init(KalmanFilter_t *filter, float q, float r, float initial_value)
{
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

/**
 * @brief 卡尔曼滤波器更新
 * @param filter 滤波器结构体指针
 * @param measurement 测量值
 * @return 滤波后的信号
 */
float KalmanFilter_Update(KalmanFilter_t *filter, float measurement)
{
    /* 预测更新 */
    filter->p = filter->p + filter->q;
    
    /* 计算卡尔曼增益 */
    filter->k = filter->p / (filter->p + filter->r);
    
    /* 测量更新 */
    filter->x = filter->x + filter->k * (measurement - filter->x);
    filter->p = (1.0f - filter->k) * filter->p;
    
    return filter->x;
}

/**
 * @brief 卡尔曼滤波器重置
 * @param filter 滤波器结构体指针
 * @param initial_value 初始值
 */
void KalmanFilter_Reset(KalmanFilter_t *filter, float initial_value)
{
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

/*======================*/
/* Butterworth Filter   */
/*======================*/

/**
 * @brief 巴特沃斯滤波器初始化
 * @param filter 滤波器结构体指针
 * @param order 滤波器阶数
 * @param cutoff_freq 截止频率 (Hz)
 * @param sample_time 采样时间 (s)
 */
void ButterworthFilter_Init(ButterworthFilter_t *filter, uint8_t order, float cutoff_freq, float sample_time)
{
    if (order > FILTER_MAX_ORDER) order = FILTER_MAX_ORDER;
    if (cutoff_freq < FILTER_MIN_CUTOFF) cutoff_freq = FILTER_MIN_CUTOFF;
    if (cutoff_freq > FILTER_MAX_CUTOFF) cutoff_freq = FILTER_MAX_CUTOFF;
    
    filter->order = order;
    
    /* 设计巴特沃斯滤波器系数 */
    Filter_DesignButterworth(filter->a, filter->b, order, cutoff_freq, sample_time);
    
    /* 初始化历史数据 */
    memset(filter->x, 0, sizeof(filter->x));
    memset(filter->y, 0, sizeof(filter->y));
}

/**
 * @brief 巴特沃斯滤波器更新
 * @param filter 滤波器结构体指针
 * @param input 输入信号
 * @return 滤波后的信号
 */
float ButterworthFilter_Update(ButterworthFilter_t *filter, float input)
{
    /* 移动历史数据 */
    for (uint8_t i = filter->order; i > 0; i--) {
        filter->x[i] = filter->x[i - 1];
        filter->y[i] = filter->y[i - 1];
    }
    
    filter->x[0] = input;
    
    /* 计算输出 */
    filter->y[0] = 0.0f;
    
    /* 分子项 */
    for (uint8_t i = 0; i <= filter->order; i++) {
        filter->y[0] += filter->b[i] * filter->x[i];
    }
    
    /* 分母项 */
    for (uint8_t i = 1; i <= filter->order; i++) {
        filter->y[0] -= filter->a[i] * filter->y[i];
    }
    
    return filter->y[0];
}

/**
 * @brief 巴特沃斯滤波器重置
 * @param filter 滤波器结构体指针
 */
void ButterworthFilter_Reset(ButterworthFilter_t *filter)
{
    memset(filter->x, 0, sizeof(filter->x));
    memset(filter->y, 0, sizeof(filter->y));
}

/*======================*/
/*   Utility Functions  */
/*======================*/

/**
 * @brief 双线性变换
 * @param analog_freq 模拟频率
 * @param sample_time 采样时间
 * @return 数字频率
 */
float Filter_BilinearTransform(float analog_freq, float sample_time)
{
    return 2.0f * tanf(M_PI * analog_freq * sample_time) / sample_time;
}

/**
 * @brief 设计巴特沃斯滤波器系数
 * @param a 分母系数
 * @param b 分子系数
 * @param order 滤波器阶数
 * @param cutoff_freq 截止频率
 * @param sample_time 采样时间
 */
void Filter_DesignButterworth(float *a, float *b, uint8_t order, float cutoff_freq, float sample_time)
{
    /* 简化的巴特沃斯滤波器设计 */
    /* 这里只实现1阶和2阶的情况，更高阶需要更复杂的算法 */
    
    float omega = 2.0f * M_PI * cutoff_freq;
    float omega_d = 2.0f * tanf(omega * sample_time / 2.0f) / sample_time;
    
    if (order == 1) {
        /* 1阶巴特沃斯 */
        float norm = 1.0f / (1.0f + omega_d * sample_time);
        b[0] = omega_d * sample_time * norm;
        b[1] = b[0];
        a[0] = 1.0f;
        a[1] = (omega_d * sample_time - 1.0f) * norm;
    } else if (order == 2) {
        /* 2阶巴特沃斯 */
        float norm = 1.0f / (1.0f + SQRT2 * omega_d * sample_time + omega_d * omega_d * sample_time * sample_time);
        b[0] = omega_d * omega_d * sample_time * sample_time * norm;
        b[1] = 2.0f * b[0];
        b[2] = b[0];
        a[0] = 1.0f;
        a[1] = 2.0f * (omega_d * omega_d * sample_time * sample_time - 1.0f) * norm;
        a[2] = (1.0f - SQRT2 * omega_d * sample_time + omega_d * omega_d * sample_time * sample_time) * norm;
    }
}

/**
 * @brief 计算滤波器频率响应
 * @param a 分母系数
 * @param b 分子系数
 * @param order 滤波器阶数
 * @param freq 频率
 * @param sample_time 采样时间
 * @return 幅度响应
 */
float Filter_FrequencyResponse(float *a, float *b, uint8_t order, float freq, float sample_time)
{
    float omega = 2.0f * M_PI * freq * sample_time;
    float real_num = 0.0f, imag_num = 0.0f;
    float real_den = 0.0f, imag_den = 0.0f;
    
    /* 计算分子 */
    for (uint8_t i = 0; i <= order; i++) {
        real_num += b[i] * COS(i * omega);
        imag_num -= b[i] * SIN(i * omega);
    }
    
    /* 计算分母 */
    for (uint8_t i = 0; i <= order; i++) {
        real_den += a[i] * COS(i * omega);
        imag_den -= a[i] * SIN(i * omega);
    }
    
    /* 计算幅度 */
    float num_mag = sqrtf(real_num * real_num + imag_num * imag_num);
    float den_mag = sqrtf(real_den * real_den + imag_den * imag_den);
    
    return num_mag / den_mag;
}

/**
 * @brief 批量处理数组
 * @param filter 滤波器结构体指针
 * @param input 输入数组
 * @param output 输出数组
 * @param length 数组长度
 */
void Filter_ProcessArray(LowPassFilter_t *filter, float *input, float *output, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        output[i] = LowPassFilter_Update(filter, input[i]);
    }
}

/**
 * @brief 就地批量处理数组
 * @param filter 滤波器结构体指针
 * @param data 数据数组
 * @param length 数组长度
 */
void Filter_ProcessArrayInPlace(LowPassFilter_t *filter, float *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++) {
        data[i] = LowPassFilter_Update(filter, data[i]);
    }
}

/*======================*/
/*   Private Functions  */
/*======================*/

/**
 * @brief 快速排序
 * @param arr 数组
 * @param low 低索引
 * @param high 高索引
 */
static void quicksort(float *arr, int low, int high)
{
    if (low < high) {
        int pi = partition(arr, low, high);
        quicksort(arr, low, pi - 1);
        quicksort(arr, pi + 1, high);
    }
}

/**
 * @brief 分区函数
 * @param arr 数组
 * @param low 低索引
 * @param high 高索引
 * @return 分区索引
 */
static int partition(float *arr, int low, int high)
{
    float pivot = arr[high];
    int i = low - 1;
    
    for (int j = low; j < high; j++) {
        if (arr[j] <= pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return i + 1;
}

/**
 * @brief 交换两个浮点数
 * @param a 第一个数
 * @param b 第二个数
 */
static void swap(float *a, float *b)
{
    float temp = *a;
    *a = *b;
    *b = temp;
}
