/**
 * @file main_example_twelve_pulse.c
 * @brief 在main函数中使用十二脉冲的示例
 * @note 这是一个示例文件，展示如何集成十二脉冲功能到主程序中
 *       不包含在主程序编译中，仅供参考
 */

#include "../Inc/twelve_pulse_example.h"
#include "../../../Core/Inc/main.h"

/* 外部变量 */
extern FOC_Parameter_t FOC;
extern Sensorless_t Sensorless;

/**
 * @brief 主函数示例 - 集成十二脉冲功能
 */
int main_twelve_pulse_example(void)
{
    /* 系统初始化 */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    
    /* 板级初始化 */
    systick_config();
    gpio_init_all();
    usart_init();
    adc_config();
    Position_Sensor_Init();
    timer_init_all();
    dma_config();
    
    /* FOC初始化 */
    FOC.Mode = INIT;
    
    printf("System initialization completed.\n");
    
    /* 等待系统稳定 */
    delay_ms(1000);
    
    printf("Starting twelve pulse initial position detection...\n");
    
    /* 等待FOC初始化完成 */
    while (FOC.Mode == INIT) {
        delay_ms(10);
    }
    
    /* 执行十二脉冲自动测试 */
    int result = TwelvePulse_AutoTest();
    
    if (result == 0) {
        printf("Twelve pulse detection successful!\n");
        printf("Initial position: %.2f degrees\n", 
               TwelvePulse_GetTheta(&Sensorless.twelve_pulse) * 180.0f / M_PI);
        printf("Confidence: %.3f\n", 
               TwelvePulse_GetConfidence(&Sensorless.twelve_pulse));
        
        /* 验证结果 */
        TwelvePulse_ValidateResults();
        
        /* 开始电机运行 */
        printf("Switching to sensorless control mode...\n");
        
        /* 设置目标转速 */
        extern float Speed_Ref;
        Speed_Ref = 100.0f; // 100 RPM
        
        printf("Motor startup sequence completed.\n");
    } else {
        printf("Twelve pulse detection failed with error: %d\n", result);
        printf("Switching to encoder/resolver mode...\n");
        
        /* 如果十二脉冲失败，使用传统位置传感器 */
        FOC.Mode = Speed_Mode;
    }
    
    /* 主循环 */
    while (1) {
        /* 检查CAN通信 */
        can_receive_check();
        
        /* 其他主循环任务 */
        delay_ms(1);
        
        /* 可选：定期输出状态信息 */
        static uint32_t status_counter = 0;
        if (++status_counter >= 5000) { // 每5秒输出一次
            status_counter = 0;
            
            printf("FOC Mode: %d, Speed: %.1f RPM\n", 
                   FOC.Mode, FOC.Speed);
            
            if (FOC.SensorlessEnabled) {
                printf("Sensorless enabled, Theta: %.2f deg\n", 
                       FOC.Theta * 180.0f / M_PI);
            }
        }
    }
}

/**
 * @brief 简化的十二脉冲测试函数
 * @note 用于快速测试十二脉冲功能
 */
void simple_twelve_pulse_test(void)
{
    printf("=== Simple Twelve Pulse Test ===\n");
    
    /* 配置参数 */
    TwelvePulse_ConfigureParameters();
    
    /* 启动测试 */
    FOC_Pulse_Start();
    
    /* 等待完成 */
    uint32_t timeout = 0;
    while (!TwelvePulse_IsCompleted(&Sensorless.twelve_pulse) && timeout < 5000) {
        delay_ms(1);
        timeout++;
        
        /* 显示进度 */
        if (timeout % 500 == 0) {
            printf("Progress: %d/12 pulses\n", 
                   Sensorless.twelve_pulse.current_pulse);
        }
    }
    
    /* 显示结果 */
    if (TwelvePulse_IsCompleted(&Sensorless.twelve_pulse)) {
        float theta = TwelvePulse_GetTheta(&Sensorless.twelve_pulse);
        float confidence = TwelvePulse_GetConfidence(&Sensorless.twelve_pulse);
        
        printf("Test completed!\n");
        printf("Estimated position: %.2f degrees\n", theta * 180.0f / M_PI);
        printf("Confidence: %.3f\n", confidence);
        
        if (confidence > 0.5f) {
            printf("Result: GOOD\n");
        } else {
            printf("Result: POOR\n");
        }
    } else {
        printf("Test timeout!\n");
    }
    
    printf("================================\n");
}

/**
 * @brief 交互式十二脉冲测试
 * @note 通过串口命令控制测试过程
 */
void interactive_twelve_pulse_test(void)
{
    char command;
    
    printf("=== Interactive Twelve Pulse Test ===\n");
    printf("Commands:\n");
    printf("  s - Start test\n");
    printf("  c - Configure parameters\n");
    printf("  d - Show debug info\n");
    printf("  v - Validate results\n");
    printf("  q - Quit\n");
    printf("======================================\n");
    
    while (1) {
        printf("Enter command: ");
        scanf("%c", &command);
        
        switch (command) {
            case 's':
            case 'S':
                printf("Starting twelve pulse test...\n");
                simple_twelve_pulse_test();
                break;
                
            case 'c':
            case 'C':
                printf("Configuring parameters...\n");
                TwelvePulse_ConfigureParameters();
                break;
                
            case 'd':
            case 'D':
                printf("Debug information:\n");
                TwelvePulse_PrintDebugInfo();
                break;
                
            case 'v':
            case 'V':
                printf("Validating results...\n");
                TwelvePulse_ValidateResults();
                break;
                
            case 'q':
            case 'Q':
                printf("Exiting interactive test.\n");
                return;
                
            default:
                printf("Unknown command. Try again.\n");
                break;
        }
    }
}

/**
 * @brief 电机启动序列 - 集成十二脉冲
 * @return 0-成功，非0-失败
 */
int motor_startup_with_twelve_pulse(void)
{
    printf("=== Motor Startup Sequence ===\n");
    
    /* 步骤1：系统检查 */
    printf("Step 1: System check...\n");
    if (FOC.Mode == INIT) {
        printf("Waiting for FOC initialization...\n");
        uint32_t timeout = 0;
        while (FOC.Mode == INIT && timeout < 2000) {
            delay_ms(1);
            timeout++;
        }
        if (FOC.Mode == INIT) {
            printf("FOC initialization timeout!\n");
            return -1;
        }
    }
    printf("System ready.\n");
    
    /* 步骤2：十二脉冲初始定位 */
    printf("Step 2: Initial position detection...\n");
    int pulse_result = TwelvePulse_Example();
    
    if (pulse_result == 0) {
        printf("Initial position detected successfully.\n");
        
        /* 步骤3：切换到无位置传感器模式 */
        printf("Step 3: Switching to sensorless mode...\n");
        // 系统已自动切换
        
        /* 步骤4：开始旋转 */
        printf("Step 4: Starting motor rotation...\n");
        extern float Speed_Ref;
        Speed_Ref = 50.0f; // 开始时使用较低转速
        
        printf("Motor startup completed successfully.\n");
        return 0;
    } else {
        printf("Initial position detection failed.\n");
        printf("Falling back to sensor-based control...\n");
        
        /* 使用传统位置传感器 */
        FOC.Mode = Speed_Mode;
        FOC.SensorlessEnabled = 0;
        
        return pulse_result;
    }
}

/* 
 * 注意：这个文件是示例代码，不要直接替换原有的main.c
 * 可以将相关函数集成到实际的main.c中
 */
