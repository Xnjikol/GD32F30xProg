/**
 * @file filter_usage_example.c
 * @brief 滤波器模块使用示例
 * @author AI Assistant
 * @date 2025-07-15
 * 
 * 该文件展示了如何使用重构后的滤波器模块
 * 包含各种滤波器的使用方法和最佳实践
 */

#include "filter.h"
#include "foc.h"
#include "sensorless.h"

/*======================*/
/*   Usage Examples     */
/*======================*/

/**
 * @brief 低通滤波器使用示例
 */
void Filter_Example_LowPass(void)
{
    /* 创建低通滤波器实例 */
    static LowPassFilter_t current_filter;
    static LowPassFilter_t speed_filter;
    
    /* 初始化滤波器 */
    LowPassFilter_Init(&current_filter, 100.0f, T_2K_HZ);  /* 100Hz截止频率 */
    LowPassFilter_Init(&speed_filter, 50.0f, T_200_HZ);    /* 50Hz截止频率 */
    
    /* 在实时循环中使用 */
    float raw_current = 5.2f;  /* 原始电流值 */
    float filtered_current = LowPassFilter_Update(&current_filter, raw_current);
    
    float raw_speed = 1500.0f;  /* 原始速度值 */
    float filtered_speed = LowPassFilter_Update(&speed_filter, raw_speed);
    
    /* 重置滤波器（如果需要） */
    LowPassFilter_Reset(&current_filter);
}

/**
 * @brief 高通滤波器使用示例
 */
void Filter_Example_HighPass(void)
{
    /* 用于去除直流分量的高通滤波器 */
    static HighPassFilter_t dc_block_filter;
    
    /* 初始化为5Hz高通滤波器 */
    HighPassFilter_Init(&dc_block_filter, 5.0f, T_2K_HZ);
    
    /* 滤波信号 */
    float signal_with_dc = 2.3f + 0.1f * sinf(2.0f * M_PI * 50.0f * 0.001f);
    float ac_signal = HighPassFilter_Update(&dc_block_filter, signal_with_dc);
}

/**
 * @brief 带通滤波器使用示例
 */
void Filter_Example_BandPass(void)
{
    /* 用于高频注入的带通滤波器 */
    static BandPassFilter_t hfi_filter;
    
    /* 初始化为1kHz中心频率，200Hz带宽 */
    BandPassFilter_Init(&hfi_filter, 1000.0f, 200.0f, T_2K_HZ);
    
    /* 提取特定频率分量 */
    float mixed_signal = 1.0f;  /* 混合信号 */
    float extracted_signal = BandPassFilter_Update(&hfi_filter, mixed_signal);
}

/**
 * @brief 陷波滤波器使用示例
 */
void Filter_Example_Notch(void)
{
    /* 用于消除50Hz工频干扰的陷波滤波器 */
    static NotchFilter_t power_line_filter;
    
    /* 初始化为50Hz陷波，10Hz带宽 */
    NotchFilter_Init(&power_line_filter, 50.0f, 10.0f, T_2K_HZ);
    
    /* 滤除工频干扰 */
    float noisy_signal = 3.0f;  /* 含有工频干扰的信号 */
    float clean_signal = NotchFilter_Update(&power_line_filter, noisy_signal);
}

/**
 * @brief 移动平均滤波器使用示例
 */
void Filter_Example_MovingAverage(void)
{
    /* 创建移动平均滤波器 */
    static MovingAverageFilter_t ma_filter;
    static float ma_buffer[32];  /* 32点移动平均 */
    
    /* 初始化 */
    MovingAverageFilter_Init(&ma_filter, ma_buffer, 32);
    
    /* 滤波信号 */
    float noisy_data = 2.5f + 0.3f * (rand() / (float)RAND_MAX - 0.5f);
    float smooth_data = MovingAverageFilter_Update(&ma_filter, noisy_data);
}

/**
 * @brief 中值滤波器使用示例
 */
void Filter_Example_Median(void)
{
    /* 创建中值滤波器 */
    static MedianFilter_t median_filter;
    static float median_buffer[5];  /* 5点中值滤波 */
    
    /* 初始化 */
    MedianFilter_Init(&median_filter, median_buffer, 5);
    
    /* 滤除脉冲干扰 */
    float spike_data = 1.0f;  /* 含有脉冲干扰的数据 */
    float filtered_data = MedianFilter_Update(&median_filter, spike_data);
}

/**
 * @brief 卡尔曼滤波器使用示例
 */
void Filter_Example_Kalman(void)
{
    /* 创建卡尔曼滤波器 */
    static KalmanFilter_t kalman_filter;
    
    /* 初始化：过程噪声0.01，测量噪声0.1，初始值0 */
    KalmanFilter_Init(&kalman_filter, 0.01f, 0.1f, 0.0f);
    
    /* 滤波测量值 */
    float measurement = 1.2f;  /* 测量值 */
    float estimated_value = KalmanFilter_Update(&kalman_filter, measurement);
}

/**
 * @brief 巴特沃斯滤波器使用示例
 */
void Filter_Example_Butterworth(void)
{
    /* 创建2阶巴特沃斯滤波器 */
    static ButterworthFilter_t butterworth_filter;
    
    /* 初始化：2阶，100Hz截止频率 */
    ButterworthFilter_Init(&butterworth_filter, 2, 100.0f, T_2K_HZ);
    
    /* 滤波信号 */
    float input_signal = 3.0f;
    float output_signal = ButterworthFilter_Update(&butterworth_filter, input_signal);
}

/**
 * @brief 在FOC控制中使用滤波器
 */
void Filter_Example_FOC_Integration(void)
{
    /* 电流滤波器 */
    static LowPassFilter_t ia_filter, ib_filter, ic_filter;
    
    /* 初始化电流滤波器 */
    LowPassFilter_Init(&ia_filter, 200.0f, T_2K_HZ);
    LowPassFilter_Init(&ib_filter, 200.0f, T_2K_HZ);
    LowPassFilter_Init(&ic_filter, 200.0f, T_2K_HZ);
    
    /* 在FOC主循环中使用 */
    extern float Ia, Ib, Ic;
    
    float Ia_filtered = LowPassFilter_Update(&ia_filter, Ia);
    float Ib_filtered = LowPassFilter_Update(&ib_filter, Ib);
    float Ic_filtered = LowPassFilter_Update(&ic_filter, Ic);
    
    /* 使用滤波后的电流进行Clarke变换 */
    Clarke_t clarke_filtered;
    ClarkeTransform(Ia_filtered, Ib_filtered, Ic_filtered, &clarke_filtered);
}

/**
 * @brief 在无位置传感器中使用滤波器
 */
void Filter_Example_Sensorless_Integration(void)
{
    /* 为SMO创建反EMF滤波器 */
    static LowPassFilter_t emf_alpha_filter, emf_beta_filter;
    
    /* 初始化反EMF滤波器 */
    LowPassFilter_Init(&emf_alpha_filter, 100.0f, T_2K_HZ);
    LowPassFilter_Init(&emf_beta_filter, 100.0f, T_2K_HZ);
    
    /* 在SMO算法中使用 */
    float emf_alpha_raw = 0.5f;  /* 原始反EMF */
    float emf_beta_raw = 0.3f;
    
    float emf_alpha_filtered = LowPassFilter_Update(&emf_alpha_filter, emf_alpha_raw);
    float emf_beta_filtered = LowPassFilter_Update(&emf_beta_filter, emf_beta_raw);
    
    /* 使用滤波后的反EMF进行位置估计 */
    float theta_est = atan2f(emf_beta_filtered, emf_alpha_filtered);
}

/**
 * @brief 批量处理示例
 */
void Filter_Example_BatchProcessing(void)
{
    static LowPassFilter_t batch_filter;
    LowPassFilter_Init(&batch_filter, 50.0f, T_2K_HZ);
    
    /* 批量处理数据 */
    float input_data[100];
    float output_data[100];
    
    /* 填充输入数据 */
    for (int i = 0; i < 100; i++) {
        input_data[i] = sinf(2.0f * M_PI * i * 0.01f) + 0.1f * (rand() / (float)RAND_MAX - 0.5f);
    }
    
    /* 批量滤波 */
    Filter_ProcessArray(&batch_filter, input_data, output_data, 100);
    
    /* 或就地处理 */
    Filter_ProcessArrayInPlace(&batch_filter, input_data, 100);
}

/**
 * @brief 多级滤波器示例
 */
void Filter_Example_MultiStage(void)
{
    /* 创建多级滤波器 */
    static LowPassFilter_t stage1_filter, stage2_filter;
    static HighPassFilter_t dc_block_filter;
    
    /* 初始化多级滤波器 */
    HighPassFilter_Init(&dc_block_filter, 5.0f, T_2K_HZ);     /* 第一级：去直流 */
    LowPassFilter_Init(&stage1_filter, 200.0f, T_2K_HZ);      /* 第二级：低通 */
    LowPassFilter_Init(&stage2_filter, 100.0f, T_2K_HZ);      /* 第三级：更强的低通 */
    
    /* 多级滤波处理 */
    float raw_signal = 2.0f;
    float stage1_output = HighPassFilter_Update(&dc_block_filter, raw_signal);
    float stage2_output = LowPassFilter_Update(&stage1_filter, stage1_output);
    float final_output = LowPassFilter_Update(&stage2_filter, stage2_output);
}

/**
 * @brief 自适应滤波器示例
 */
void Filter_Example_Adaptive(void)
{
    static LowPassFilter_t adaptive_filter;
    static float last_cutoff = 100.0f;
    
    /* 根据系统状态自适应调整滤波器参数 */
    extern float Speed_Ref;
    float current_speed = fabsf(Speed_Ref);
    
    /* 根据速度调整滤波器截止频率 */
    float cutoff_freq;
    if (current_speed < 100.0f) {
        cutoff_freq = 50.0f;   /* 低速时更强的滤波 */
    } else if (current_speed < 500.0f) {
        cutoff_freq = 100.0f;  /* 中速时中等滤波 */
    } else {
        cutoff_freq = 200.0f;  /* 高速时轻度滤波 */
    }
    
    /* 仅在截止频率变化时重新初始化 */
    if (fabsf(cutoff_freq - last_cutoff) > 5.0f) {
        LowPassFilter_Init(&adaptive_filter, cutoff_freq, T_2K_HZ);
        last_cutoff = cutoff_freq;
    }
    
    /* 使用自适应滤波器 */
    float signal = 1.5f;
    float filtered_signal = LowPassFilter_Update(&adaptive_filter, signal);
}

/**
 * @brief 性能测试示例
 */
void Filter_Example_Performance(void)
{
    static LowPassFilter_t perf_filter;
    LowPassFilter_Init(&perf_filter, 100.0f, T_2K_HZ);
    
    /* 性能测试：处理1000个样本 */
    uint32_t start_time = 0;  /* 获取时间戳 */
    
    for (int i = 0; i < 1000; i++) {
        float test_signal = sinf(2.0f * M_PI * i * 0.001f);
        float filtered = LowPassFilter_Update(&perf_filter, test_signal);
        (void)filtered;  /* 避免编译警告 */
    }
    
    uint32_t end_time = 0;    /* 获取时间戳 */
    uint32_t execution_time = end_time - start_time;
    
    /* 输出性能结果 */
    printf("Filter processing time: %lu cycles\n", execution_time);
}

/**
 * @brief 完整的滤波器使用示例
 */
void Filter_Example_Complete(void)
{
    /* 初始化所有滤波器 */
    Filter_Example_LowPass();
    Filter_Example_HighPass();
    Filter_Example_BandPass();
    Filter_Example_Notch();
    Filter_Example_MovingAverage();
    Filter_Example_Median();
    Filter_Example_Kalman();
    Filter_Example_Butterworth();
    
    /* 在控制系统中集成 */
    Filter_Example_FOC_Integration();
    Filter_Example_Sensorless_Integration();
    
    /* 高级应用 */
    Filter_Example_BatchProcessing();
    Filter_Example_MultiStage();
    Filter_Example_Adaptive();
    
    /* 性能评估 */
    Filter_Example_Performance();
}

/*======================*/
/*   Best Practices     */
/*======================*/

/**
 * @brief 滤波器设计最佳实践
 */
void Filter_BestPractices(void)
{
    /* 1. 选择合适的滤波器类型 */
    // 低通滤波器：去除高频噪声
    // 高通滤波器：去除直流分量
    // 带通滤波器：提取特定频率分量
    // 陷波滤波器：消除特定频率干扰
    
    /* 2. 正确设置截止频率 */
    // 截止频率应该是信号频率的2-5倍
    // 考虑采样频率的奈奎斯特限制
    
    /* 3. 考虑相位延迟 */
    // 滤波器会引入相位延迟
    // 在闭环控制中需要考虑稳定性
    
    /* 4. 初始化时机 */
    // 在系统初始化时进行滤波器初始化
    // 在参数变化时重新初始化
    
    /* 5. 内存管理 */
    // 使用静态分配避免内存碎片
    // 为移动平均和中值滤波器预分配缓冲区
    
    /* 6. 数值稳定性 */
    // 避免除零错误
    // 检查输入参数的有效性
    // 使用合适的数据类型
}

/**
 * @brief 滤波器调试技巧
 */
void Filter_DebuggingTips(void)
{
    static LowPassFilter_t debug_filter;
    LowPassFilter_Init(&debug_filter, 100.0f, T_2K_HZ);
    
    /* 1. 记录滤波器状态 */
    float input = 1.0f;
    float output = LowPassFilter_Update(&debug_filter, input);
    
    /* 输出调试信息 */
    printf("Filter - Input: %.3f, Output: %.3f, State: %.3f\n", 
           input, output, debug_filter.y_last);
    
    /* 2. 测试滤波器响应 */
    // 使用已知的测试信号（如阶跃、正弦波）
    // 验证滤波器的频率响应
    
    /* 3. 检查数值稳定性 */
    // 测试极限值输入
    // 验证滤波器不会发散
    
    /* 4. 性能监控 */
    // 测量滤波器的执行时间
    // 监控内存使用情况
}

/**
 * @brief 滤波器参数优化
 */
void Filter_ParameterOptimization(void)
{
    /* 1. 频率响应分析 */
    static ButterworthFilter_t test_filter;
    ButterworthFilter_Init(&test_filter, 2, 100.0f, T_2K_HZ);
    
    /* 计算不同频率的响应 */
    for (float freq = 1.0f; freq <= 1000.0f; freq *= 1.1f) {
        float response = Filter_FrequencyResponse(test_filter.a, test_filter.b, 
                                                 test_filter.order, freq, T_2K_HZ);
        printf("Freq: %.1f Hz, Response: %.3f\n", freq, response);
    }
    
    /* 2. 时域响应分析 */
    // 测试阶跃响应
    // 测试脉冲响应
    // 测试噪声抑制能力
    
    /* 3. 参数扫描 */
    // 扫描不同的截止频率
    // 比较不同的滤波器阶数
    // 优化滤波器参数
}

/*======================*/
/*   Common Pitfalls    */
/*======================*/

/**
 * @brief 常见错误及解决方案
 */
void Filter_CommonPitfalls(void)
{
    /* 1. 错误：未初始化滤波器 */
    // 解决：始终在使用前初始化滤波器
    
    /* 2. 错误：截止频率过高或过低 */
    // 解决：根据信号特性选择合适的截止频率
    
    /* 3. 错误：忽略采样频率限制 */
    // 解决：确保截止频率 < 采样频率/2
    
    /* 4. 错误：在中断中使用复杂滤波器 */
    // 解决：在主循环中使用复杂滤波器
    
    /* 5. 错误：滤波器参数硬编码 */
    // 解决：使用宏定义或配置参数
    
    /* 6. 错误：未考虑滤波器群延迟 */
    // 解决：在控制系统设计中考虑相位延迟
}

/*======================*/
/*   Migration Guide    */
/*======================*/

/**
 * @brief 从旧代码迁移到新滤波器模块
 */
void Filter_MigrationGuide(void)
{
    /* 旧代码示例：
     * typedef struct {
     *     float a;
     *     float y_last;
     * } OldLowPassFilter_t;
     * 
     * OldLowPassFilter_t old_filter = {.a = 0.1f, .y_last = 0.0f};
     * float output = old_filter.a * input + (1.0f - old_filter.a) * old_filter.y_last;
     * old_filter.y_last = output;
     */
    
    /* 新代码示例： */
    static LowPassFilter_t new_filter;
    LowPassFilter_Init(&new_filter, 100.0f, T_2K_HZ);  /* 根据需要的截止频率初始化 */
    
    float input = 1.0f;
    float output = LowPassFilter_Update(&new_filter, input);
    
    /* 迁移步骤：
     * 1. 包含新的头文件 #include "filter.h"
     * 2. 使用新的结构体类型 LowPassFilter_t
     * 3. 调用初始化函数 LowPassFilter_Init()
     * 4. 使用更新函数 LowPassFilter_Update()
     * 5. 删除旧的滤波器代码
     */
}
