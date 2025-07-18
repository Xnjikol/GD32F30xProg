/**
 * @file twelve_pulse_example.c
 * @brief 十二脉冲初始定位示例代码
 * @author your_name
 * @date 2024-12-19
 * @note 这是一个独立的示例文件，不包含在主程序编译中
 */

#include "../Inc/twelve_pulse_example.h"
#include "../../../Core/Inc/foc.h"
#include "../../../Core/Inc/sensorless.h"
#include "../../../Core/Inc/main.h"
#include "../../../Core/Inc/delay.h"
#include <stdio.h>
#include <float.h>
#include <math.h>

/* 外部变量声明 */
extern FOC_Parameter_t FOC;
extern Sensorless_t Sensorless;

/**
 * @brief 十二脉冲初始定位示例
 * @return 0-成功，非0-失败
 */
int TwelvePulse_Example(void)
{
    uint32_t timeout = 0;
    float estimated_theta = 0.0f;
    float confidence = 0.0f;
    
    /* 1. 确保系统已初始化 */
    if (FOC.Mode == INIT) {
        /* 等待系统初始化完成 */
        while (FOC.Mode == INIT && timeout < 1000) {
            timeout++;
            delay_ms(1);
        }
        
        if (FOC.Mode == INIT) {
            return -1; /* 初始化超时 */
        }
    }
    
    /* 2. 启动十二脉冲测试 */
    printf("Starting twelve pulse initial position detection...\n");
    
    /* 切换到十二脉冲模式 */
    FOC_Pulse_Start();
    
    /* 3. 等待测试完成 */
    timeout = 0;
    while (!TwelvePulse_IsCompleted(&Sensorless.twelve_pulse) && timeout < 5000) {
        timeout++;
        delay_ms(1);
        
        /* 可以在这里添加进度显示 */
        if (timeout % 100 == 0) {
            printf("Twelve pulse progress: pulse %d/12\n", 
                   Sensorless.twelve_pulse.current_pulse);
        }
    }
    
    /* 4. 检查测试结果 */
    if (TwelvePulse_IsCompleted(&Sensorless.twelve_pulse)) {
        estimated_theta = TwelvePulse_GetTheta(&Sensorless.twelve_pulse);
        confidence = TwelvePulse_GetConfidence(&Sensorless.twelve_pulse);
        
        printf("Twelve pulse completed!\n");
        printf("Estimated initial position: %.3f rad (%.1f deg)\n", 
               estimated_theta, estimated_theta * 180.0f / M_PI);
        printf("Confidence: %.3f\n", confidence);
        
        /* 检查置信度 */
        if (confidence > 0.5f) {
            printf("Good confidence, switching to sensorless mode.\n");
            /* 系统会自动切换到无位置传感器模式 */
            return 0;
        } else {
            printf("Low confidence, may need to retry.\n");
            return -2;
        }
    } else {
        printf("Twelve pulse timeout!\n");
        return -3;
    }
}

/**
 * @brief 十二脉冲调试信息输出
 */
void TwelvePulse_PrintDebugInfo(void)
{
    TwelvePulse_t *pulse = &Sensorless.twelve_pulse;
    
    printf("=== Twelve Pulse Debug Info ===\n");
    printf("State: %d\n", pulse->state);
    printf("Current pulse: %d/12\n", pulse->current_pulse);
    printf("Enabled: %d, Completed: %d\n", pulse->enabled, pulse->completed);
    printf("Test voltage: %.2f V\n", pulse->test_voltage);
    printf("Estimated theta: %.3f rad (%.1f deg)\n", 
           pulse->theta_est, pulse->theta_est * 180.0f / M_PI);
    printf("Confidence: %.3f\n", pulse->confidence);
    
    printf("Current responses:\n");
    for (uint8_t i = 0; i < 12; i++) {
        printf("Pulse %2d: Id=%.3f, Iq=%.3f, Mag=%.3f, Angle=%.1f deg\n", 
               i, pulse->id_response[i], pulse->iq_response[i], 
               pulse->current_magnitude[i], i * 30.0f);
    }
    printf("===============================\n");
}

/**
 * @brief 十二脉冲参数配置
 */
void TwelvePulse_ConfigureParameters(void)
{
    TwelvePulse_t *pulse = &Sensorless.twelve_pulse;
    
    /* 可以根据电机参数调整这些值 */
    pulse->test_voltage = 8.0f;    /* 测试电压 */
    pulse->test_time = 0.01f;      /* 脉冲持续时间 */
    pulse->settle_time = 0.002f;   /* 稳定时间 */
    
    printf("Twelve pulse parameters configured:\n");
    printf("Test voltage: %.2f V\n", pulse->test_voltage);
    printf("Test time: %.3f s\n", pulse->test_time);
    printf("Settle time: %.3f s\n", pulse->settle_time);
}

/**
 * @brief 十二脉冲自动测试序列
 */
int TwelvePulse_AutoTest(void)
{
    int result = 0;
    int retry_count = 0;
    const int max_retries = 3;
    
    printf("Starting twelve pulse auto test sequence...\n");
    
    /* 配置参数 */
    TwelvePulse_ConfigureParameters();
    
    /* 最多重试3次 */
    while (retry_count < max_retries) {
        printf("Attempt %d/%d\n", retry_count + 1, max_retries);
        
        result = TwelvePulse_Example();
        
        if (result == 0) {
            printf("Twelve pulse test successful!\n");
            return 0;
        } else {
            printf("Twelve pulse test failed with error %d\n", result);
            
            /* 输出调试信息 */
            TwelvePulse_PrintDebugInfo();
            
            retry_count++;
            
            if (retry_count < max_retries) {
                printf("Retrying in 1 second...\n");
                delay_ms(1000);
            }
        }
    }
    
    printf("Twelve pulse auto test failed after %d attempts\n", max_retries);
    return -1;
}

/**
 * @brief 十二脉冲结果验证
 */
void TwelvePulse_ValidateResults(void)
{
    TwelvePulse_t *pulse = &Sensorless.twelve_pulse;
    
    if (!pulse->completed) {
        printf("Twelve pulse not completed yet.\n");
        return;
    }
    
    printf("=== Twelve Pulse Validation ===\n");
    
    /* 检查所有脉冲的响应 */
    float avg_magnitude = 0.0f;
    float min_magnitude = FLT_MAX;
    float max_magnitude = 0.0f;
    
    for (uint8_t i = 0; i < 12; i++) {
        float mag = pulse->current_magnitude[i];
        avg_magnitude += mag;
        
        if (mag < min_magnitude) min_magnitude = mag;
        if (mag > max_magnitude) max_magnitude = mag;
    }
    
    avg_magnitude /= 12.0f;
    
    printf("Average magnitude: %.3f A\n", avg_magnitude);
    printf("Min magnitude: %.3f A\n", min_magnitude);
    printf("Max magnitude: %.3f A\n", max_magnitude);
    printf("Magnitude ratio: %.3f\n", max_magnitude / (min_magnitude + 1e-6f));
    
    /* 检查估计精度 */
    if (pulse->confidence > 0.8f) {
        printf("High confidence estimate - GOOD\n");
    } else if (pulse->confidence > 0.5f) {
        printf("Medium confidence estimate - ACCEPTABLE\n");
    } else {
        printf("Low confidence estimate - POOR\n");
    }
    
    /* 检查响应一致性 */
    float variance = 0.0f;
    for (uint8_t i = 0; i < 12; i++) {
        float diff = pulse->current_magnitude[i] - avg_magnitude;
        variance += diff * diff;
    }
    variance /= 12.0f;
    
    printf("Response variance: %.6f\n", variance);
    
    if (variance < 0.01f) {
        printf("Response consistency - EXCELLENT\n");
    } else if (variance < 0.05f) {
        printf("Response consistency - GOOD\n");
    } else {
        printf("Response consistency - POOR\n");
    }
    
    printf("===============================\n");
}
