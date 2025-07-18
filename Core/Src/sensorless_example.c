/**
 * @file sensorless_example.c
 * @brief 无位置传感器算法使用示例
 * @author AI Assistant
 * @date 2025-07-15
 * 
 * 该文件展示了如何使用无位置传感器算法库
 * 包含初始化、配置、运行和故障处理等完整流程
 */

#include "sensorless_interface.h"
#include "foc.h"

/*======================*/
/*   Example Functions  */
/*======================*/

/**
 * @brief 无位置传感器初始化示例
 */
void Sensorless_Example_Init(void)
{
    /* 1. 初始化无位置传感器接口 */
    SensorlessInterface_Init();
    
    /* 2. 配置高频注入参数 */
    SensorlessInterface_ConfigHFI(1000.0f, 5.0f);  /* 1kHz, 5V */
    
    /* 3. 配置滑膜观测器参数 */
    SensorlessInterface_ConfigSMO(200.0f, 100.0f);  /* 增益200, 截止频率100Hz */
    
    /* 4. 配置PLL参数 */
    SensorlessInterface_ConfigPLL(500.0f, 50000.0f);  /* Kp=500, Ki=50000 */
    
    /* 5. 配置算法切换阈值 */
    SensorlessInterface_ConfigSwitchThreshold(200.0f, 150.0f);  /* 200rpm向上, 150rpm向下 */
    
    /* 6. 设置为混合算法模式 */
    SensorlessInterface_SetAlgorithm(SENSORLESS_HYBRID);
    
    /* 7. 启用无位置传感器 */
    SensorlessInterface_Enable(1);
}

/**
 * @brief 无位置传感器运行示例
 */
void Sensorless_Example_Run(void)
{
    /* 1. 更新无位置传感器接口 */
    SensorlessInterface_Update();
    
    /* 2. 获取估计的位置和速度 */
    float theta = SensorlessInterface_GetTheta();
    float speed = SensorlessInterface_GetSpeed();
    
    /* 3. 检查电机状态 */
    MotorState_t motor_state = SensorlessInterface_GetMotorState();
    
    /* 4. 根据状态执行不同的控制策略 */
    switch (motor_state) {
        case MOTOR_STOP:
            /* 电机停止状态 */
            break;
            
        case MOTOR_STARTUP:
            /* 启动状态 - 等待十二脉冲完成 */
            if (SensorlessInterface.twelve_pulse_completed) {
                /* 启动完成，切换到运行模式 */
                FOC.Mode = NoSensorMode;
            }
            break;
            
        case MOTOR_LOW_SPEED:
            /* 低速运行 - 使用HFI算法 */
            FOC.Mode = NoSensorMode;
            break;
            
        case MOTOR_HIGH_SPEED:
            /* 高速运行 - 使用SMO算法 */
            FOC.Mode = NoSensorMode;
            break;
    }
    
    /* 5. 故障处理 */
    uint8_t fault_flags = SensorlessInterface_GetFaultFlags();
    if (fault_flags != SENSORLESS_FAULT_NONE) {
        Sensorless_Example_HandleFaults(fault_flags);
    }
}

/**
 * @brief 启动序列示例
 */
void Sensorless_Example_Startup(void)
{
    /* 1. 确保电机停止 */
    if (SensorlessInterface_GetMotorState() != MOTOR_STOP) {
        return;
    }
    
    /* 2. 清除故障 */
    SensorlessInterface_ClearFaults();
    
    /* 3. 启动十二脉冲序列 */
    SensorlessInterface_StartupSequence();
    
    /* 4. 等待启动完成 */
    uint16_t timeout = 0;
    while (!SensorlessInterface.twelve_pulse_completed && timeout < 2000) {
        /* 更新算法 */
        SensorlessInterface_Update();
        
        /* 检查故障 */
        if (SensorlessInterface_GetFaultFlags() & SENSORLESS_FAULT_STARTUP_FAIL) {
            /* 启动失败，重新尝试 */
            SensorlessInterface_Reset();
            SensorlessInterface_StartupSequence();
            timeout = 0;
        }
        
        timeout++;
        /* 延时1ms */
        // delay_ms(1);
    }
    
    /* 5. 启动完成，获取初始位置 */
    if (SensorlessInterface.twelve_pulse_completed) {
        float initial_theta = SensorlessInterface_GetTheta();
        /* 设置FOC初始位置 */
        FOC.Theta = initial_theta;
    }
}

/**
 * @brief 故障处理示例
 * @param fault_flags 故障标志
 */
void Sensorless_Example_HandleFaults(uint8_t fault_flags)
{
    if (fault_flags & SENSORLESS_FAULT_POSITION_LOST) {
        /* 位置丢失 - 尝试重新启动 */
        SensorlessInterface_Reset();
        SensorlessInterface_StartupSequence();
    }
    
    if (fault_flags & SENSORLESS_FAULT_SPEED_UNSTABLE) {
        /* 速度不稳定 - 降低增益 */
        SensorlessInterface_ConfigSMO(150.0f, 80.0f);  /* 降低SMO增益 */
    }
    
    if (fault_flags & SENSORLESS_FAULT_ALGORITHM_FAIL) {
        /* 算法失效 - 切换到备用算法 */
        SensorlessInterface_SetAlgorithm(SENSORLESS_FLUX_OBSERVER);
    }
    
    if (fault_flags & SENSORLESS_FAULT_STARTUP_FAIL) {
        /* 启动失败 - 停止电机 */
        STOP = 1;
        FOC.Mode = IDLE;
    }
}

/**
 * @brief 调试信息输出示例
 */
void Sensorless_Example_Debug(void)
{
    char debug_buffer[512];
    
    /* 获取调试信息 */
    SensorlessInterface_GetDebugInfo(debug_buffer, sizeof(debug_buffer));
    
    /* 输出调试信息 */
    printf("%s", debug_buffer);
    
    /* 输出详细性能信息 */
    printf("Position Error: %.3f rad\n", SensorlessInterface_GetPositionError());
    printf("Speed Error: %.1f rpm\n", SensorlessInterface_GetSpeedError());
    printf("Confidence: %.2f\n", SensorlessInterface_GetEstimationConfidence());
}

/**
 * @brief 算法性能测试示例
 */
void Sensorless_Example_PerformanceTest(void)
{
    /* 1. 启用测试模式 */
    SensorlessInterface_TestMode(1);
    
    /* 2. 测试HFI算法 */
    SensorlessInterface_ForceAlgorithm(SENSORLESS_HFI);
    
    /* 运行一段时间并记录性能 */
    for (uint16_t i = 0; i < 1000; i++) {
        SensorlessInterface_Update();
        /* 记录性能数据 */
        // performance_data[i] = SensorlessInterface_GetEstimationConfidence();
    }
    
    /* 3. 测试SMO算法 */
    SensorlessInterface_ForceAlgorithm(SENSORLESS_SMO);
    
    /* 运行一段时间并记录性能 */
    for (uint16_t i = 0; i < 1000; i++) {
        SensorlessInterface_Update();
        /* 记录性能数据 */
        // performance_data[i] = SensorlessInterface_GetEstimationConfidence();
    }
    
    /* 4. 恢复自动模式 */
    SensorlessInterface_TestMode(0);
    SensorlessInterface_SetAlgorithm(SENSORLESS_HYBRID);
}

/**
 * @brief 回调函数示例
 * @param interface 无位置传感器接口指针
 */
void Sensorless_Example_PositionCallback(SensorlessInterface_t *interface)
{
    /* 位置更新回调 */
    static float last_theta = 0.0f;
    float theta_change = fabsf(interface->theta_est - last_theta);
    
    if (theta_change > 0.1f) {  /* 位置变化超过0.1rad */
        /* 执行位置相关的处理 */
        last_theta = interface->theta_est;
    }
}

/**
 * @brief 速度回调函数示例
 * @param interface 无位置传感器接口指针
 */
void Sensorless_Example_SpeedCallback(SensorlessInterface_t *interface)
{
    /* 速度更新回调 */
    static float last_speed = 0.0f;
    float speed_change = fabsf(interface->speed_est - last_speed);
    
    if (speed_change > 10.0f) {  /* 速度变化超过10rpm */
        /* 执行速度相关的处理 */
        last_speed = interface->speed_est;
    }
}

/**
 * @brief 故障回调函数示例
 * @param interface 无位置传感器接口指针
 */
void Sensorless_Example_FaultCallback(SensorlessInterface_t *interface)
{
    /* 故障发生回调 */
    printf("Sensorless Fault Detected: 0x%02X\n", interface->fault_flags);
    
    /* 根据故障类型执行相应处理 */
    Sensorless_Example_HandleFaults(interface->fault_flags);
}

/**
 * @brief 设置回调函数示例
 */
void Sensorless_Example_SetupCallbacks(void)
{
    /* 设置位置回调 */
    SensorlessInterface_SetPositionCallback(Sensorless_Example_PositionCallback);
    
    /* 设置速度回调 */
    SensorlessInterface_SetSpeedCallback(Sensorless_Example_SpeedCallback);
    
    /* 设置故障回调 */
    SensorlessInterface_SetFaultCallback(Sensorless_Example_FaultCallback);
}

/**
 * @brief 完整使用示例
 */
void Sensorless_Example_Complete(void)
{
    /* 1. 初始化 */
    Sensorless_Example_Init();
    
    /* 2. 设置回调函数 */
    Sensorless_Example_SetupCallbacks();
    
    /* 3. 启动序列 */
    Sensorless_Example_Startup();
    
    /* 4. 主循环 */
    while (1) {
        /* 更新无位置传感器 */
        Sensorless_Example_Run();
        
        /* 每100ms输出一次调试信息 */
        static uint16_t debug_counter = 0;
        if (++debug_counter >= 200) {  /* 假设2kHz调用频率 */
            debug_counter = 0;
            Sensorless_Example_Debug();
        }
        
        /* 延时 */
        // delay_us(500);  /* 2kHz更新频率 */
    }
}

/*======================*/
/*   Configuration Examples */
/*======================*/

/**
 * @brief 低速电机配置示例
 */
void Sensorless_Example_LowSpeedMotor(void)
{
    /* 适合低速电机的配置 */
    SensorlessInterface_ConfigHFI(1500.0f, 3.0f);      /* 更高频率，更低电压 */
    SensorlessInterface_ConfigSMO(100.0f, 50.0f);      /* 更低增益 */
    SensorlessInterface_ConfigSwitchThreshold(100.0f, 80.0f);  /* 更低切换阈值 */
    
    /* 优先使用HFI */
    SensorlessInterface_SetAlgorithm(SENSORLESS_HFI);
}

/**
 * @brief 高速电机配置示例
 */
void Sensorless_Example_HighSpeedMotor(void)
{
    /* 适合高速电机的配置 */
    SensorlessInterface_ConfigHFI(800.0f, 8.0f);       /* 更低频率，更高电压 */
    SensorlessInterface_ConfigSMO(300.0f, 150.0f);     /* 更高增益 */
    SensorlessInterface_ConfigSwitchThreshold(300.0f, 250.0f);  /* 更高切换阈值 */
    
    /* 优先使用SMO */
    SensorlessInterface_SetAlgorithm(SENSORLESS_SMO);
}

/**
 * @brief 高精度配置示例
 */
void Sensorless_Example_HighPrecision(void)
{
    /* 高精度配置 */
    SensorlessInterface_ConfigHFI(2000.0f, 2.0f);      /* 高频率，低电压 */
    SensorlessInterface_ConfigSMO(500.0f, 200.0f);     /* 高增益，高截止频率 */
    SensorlessInterface_ConfigPLL(1000.0f, 100000.0f); /* 高增益PLL */
    
    /* 使用混合算法 */
    SensorlessInterface_SetAlgorithm(SENSORLESS_HYBRID);
}
