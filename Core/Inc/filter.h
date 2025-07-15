#ifndef _FILTER_H_
#define _FILTER_H_

#include "main.h"

/*======================*/
/*   Filter Constants   */
/*======================*/

#define FILTER_MAX_ORDER    4       /* 最大滤波器阶数 */
#define FILTER_MIN_CUTOFF   0.1f    /* 最小截止频率 (Hz) */
#define FILTER_MAX_CUTOFF   10000.0f /* 最大截止频率 (Hz) */

/*======================*/
/*   Type Definitions   */
/*======================*/

/* 低通滤波器结构 */
typedef struct
{
    float a;        /* 滤波系数 */
    float x_last;   /* 上次输入 */
    float y_last;   /* 上次输出 */
} LowPassFilter_t;

/* 高通滤波器结构 */
typedef struct
{
    float a;        /* 滤波系数 */
    float x_last;   /* 上次输入 */
    float y_last;   /* 上次输出 */
} HighPassFilter_t;

/* 带通滤波器结构 */
typedef struct
{
    float a1, a2, b1, b2;   /* 滤波系数 */
    float x1, x2;           /* 输入历史 */
    float y1, y2;           /* 输出历史 */
} BandPassFilter_t;

/* 陷波滤波器结构 */
typedef struct
{
    float a1, a2, b0, b1, b2;   /* 滤波系数 */
    float x1, x2;               /* 输入历史 */
    float y1, y2;               /* 输出历史 */
} NotchFilter_t;

/* 移动平均滤波器结构 */
typedef struct
{
    float *buffer;      /* 数据缓冲区 */
    uint16_t size;      /* 缓冲区大小 */
    uint16_t index;     /* 当前索引 */
    float sum;          /* 累计和 */
    uint8_t initialized; /* 初始化标志 */
} MovingAverageFilter_t;

/* 中值滤波器结构 */
typedef struct
{
    float *buffer;      /* 数据缓冲区 */
    uint16_t size;      /* 缓冲区大小 */
    uint16_t index;     /* 当前索引 */
    uint8_t initialized; /* 初始化标志 */
} MedianFilter_t;

/* 卡尔曼滤波器结构 */
typedef struct
{
    float q;            /* 过程噪声方差 */
    float r;            /* 测量噪声方差 */
    float x;            /* 状态估计 */
    float p;            /* 估计误差方差 */
    float k;            /* 卡尔曼增益 */
} KalmanFilter_t;

/* 巴特沃斯滤波器结构 */
typedef struct
{
    float a[FILTER_MAX_ORDER + 1];  /* 分母系数 */
    float b[FILTER_MAX_ORDER + 1];  /* 分子系数 */
    float x[FILTER_MAX_ORDER + 1];  /* 输入历史 */
    float y[FILTER_MAX_ORDER + 1];  /* 输出历史 */
    uint8_t order;                  /* 滤波器阶数 */
} ButterworthFilter_t;

/*======================*/
/*   Function Protos    */
/*======================*/

/* 低通滤波器函数 */
void LowPassFilter_Init(LowPassFilter_t *filter, float cutoff_freq, float sample_time);
float LowPassFilter_Update(LowPassFilter_t *filter, float input);
void LowPassFilter_Reset(LowPassFilter_t *filter);

/* 高通滤波器函数 */
void HighPassFilter_Init(HighPassFilter_t *filter, float cutoff_freq, float sample_time);
float HighPassFilter_Update(HighPassFilter_t *filter, float input);
void HighPassFilter_Reset(HighPassFilter_t *filter);

/* 带通滤波器函数 */
void BandPassFilter_Init(BandPassFilter_t *filter, float center_freq, float bandwidth, float sample_time);
float BandPassFilter_Update(BandPassFilter_t *filter, float input);
void BandPassFilter_Reset(BandPassFilter_t *filter);

/* 陷波滤波器函数 */
void NotchFilter_Init(NotchFilter_t *filter, float notch_freq, float bandwidth, float sample_time);
float NotchFilter_Update(NotchFilter_t *filter, float input);
void NotchFilter_Reset(NotchFilter_t *filter);

/* 移动平均滤波器函数 */
void MovingAverageFilter_Init(MovingAverageFilter_t *filter, float *buffer, uint16_t size);
float MovingAverageFilter_Update(MovingAverageFilter_t *filter, float input);
void MovingAverageFilter_Reset(MovingAverageFilter_t *filter);

/* 中值滤波器函数 */
void MedianFilter_Init(MedianFilter_t *filter, float *buffer, uint16_t size);
float MedianFilter_Update(MedianFilter_t *filter, float input);
void MedianFilter_Reset(MedianFilter_t *filter);

/* 卡尔曼滤波器函数 */
void KalmanFilter_Init(KalmanFilter_t *filter, float q, float r, float initial_value);
float KalmanFilter_Update(KalmanFilter_t *filter, float measurement);
void KalmanFilter_Reset(KalmanFilter_t *filter, float initial_value);

/* 巴特沃斯滤波器函数 */
void ButterworthFilter_Init(ButterworthFilter_t *filter, uint8_t order, float cutoff_freq, float sample_time);
float ButterworthFilter_Update(ButterworthFilter_t *filter, float input);
void ButterworthFilter_Reset(ButterworthFilter_t *filter);

/* 辅助函数 */
float Filter_BilinearTransform(float analog_freq, float sample_time);
void Filter_DesignButterworth(float *a, float *b, uint8_t order, float cutoff_freq, float sample_time);
float Filter_FrequencyResponse(float *a, float *b, uint8_t order, float freq, float sample_time);

/* 批量滤波函数 */
void Filter_ProcessArray(LowPassFilter_t *filter, float *input, float *output, uint16_t length);
void Filter_ProcessArrayInPlace(LowPassFilter_t *filter, float *data, uint16_t length);

#endif /* _FILTER_H_ */
