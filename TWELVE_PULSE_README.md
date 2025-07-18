# 十二脉冲初始定位算法说明

## 概述

十二脉冲初始定位算法是一种用于无位置传感器电机控制的初始转子位置检测方法。该算法通过在电机的12个不同角度方向（每30度一个）施加测试电压脉冲，测量对应的电流响应，从而确定转子的初始位置。

## 工作原理

### 基本原理
1. **电压注入**：在静止的电机上，按30度间隔在12个不同的电角度方向注入直流电压
2. **电流响应**：测量每个方向上的电流响应
3. **位置计算**：分析电流响应的模式，计算出转子的初始位置

### 算法步骤
1. 在θ = 0°, 30°, 60°, ..., 330° 方向依次注入测试电压
2. 每个脉冲持续10ms，稳定2ms后采样电流
3. 记录每个方向的Id和Iq电流响应
4. 使用加权平均和谐波分析计算初始位置
5. 计算估计的置信度

## 主要特点

### 优势
- **精度高**：可以达到±15度的位置精度
- **可靠性好**：多种算法融合，提高鲁棒性
- **适应性强**：适用于各种类型的永磁同步电机
- **自诊断**：提供置信度评估，便于判断估计质量

### 技术特色
- **多算法融合**：结合最大响应、加权平均、谐波分析三种方法
- **自适应参数**：根据电机特性自动调整测试参数
- **质量评估**：实时计算估计置信度和响应一致性

## 代码结构

### 核心文件
```
Core/
├── Inc/
│   ├── sensorless.h           # 无位置传感器算法头文件
│   ├── twelve_pulse_example.h # 十二脉冲示例头文件
│   └── foc.h                  # FOC控制头文件
└── Src/
    ├── sensorless.c           # 无位置传感器算法实现
    ├── twelve_pulse_example.c # 十二脉冲示例代码
    └── foc.c                  # FOC主控制循环
```

### 主要数据结构

#### TwelvePulse_t
```c
typedef struct {
    float test_voltage;         // 测试电压 (V)
    float test_time;           // 脉冲持续时间 (s)
    float settle_time;         // 稳定时间 (s)
    uint8_t current_pulse;     // 当前脉冲索引
    float pulse_timer;         // 脉冲计时器
    
    TwelvePulseState_t state;  // 当前状态
    uint8_t completed;         // 完成标志
    uint8_t enabled;           // 使能标志
    float confidence;          // 估计置信度
    
    float id_response[12];     // d轴电流响应
    float iq_response[12];     // q轴电流响应
    float current_magnitude[12]; // 电流幅值
    float response_quality[12];  // 响应质量
    
    float theta_est;           // 估计的初始位置
    float theta_raw;           // 原始位置估计
    float theta_filtered;      // 滤波后的位置估计
} TwelvePulse_t;
```

## 使用方法

### 1. 基本使用步骤

```c
#include "twelve_pulse_example.h"

int main(void) {
    // 系统初始化
    System_Init();
    FOC_Init();
    Sensorless_Init();
    
    // 启动十二脉冲测试
    int result = TwelvePulse_Example();
    
    if (result == 0) {
        printf("十二脉冲定位成功!\n");
        // 继续无位置传感器运行
    } else {
        printf("十二脉冲定位失败: %d\n", result);
    }
    
    return 0;
}
```

### 2. 自动测试序列

```c
// 自动测试，包含重试机制
int result = TwelvePulse_AutoTest();
if (result == 0) {
    printf("自动测试成功!\n");
}
```

### 3. 手动控制

```c
// 手动启动
FOC_TwelvePulse_Start();

// 等待完成
while (!TwelvePulse_IsCompleted(&Sensorless.twelve_pulse)) {
    delay_ms(1);
}

// 获取结果
float theta = TwelvePulse_GetTheta(&Sensorless.twelve_pulse);
float confidence = TwelvePulse_GetConfidence(&Sensorless.twelve_pulse);
```

### 4. 参数配置

```c
// 配置测试参数
TwelvePulse_ConfigureParameters();

// 或手动设置
Sensorless.twelve_pulse.test_voltage = 8.0f;    // 测试电压
Sensorless.twelve_pulse.test_time = 0.01f;      // 脉冲时间
Sensorless.twelve_pulse.settle_time = 0.002f;   // 稳定时间
```

## 调试和验证

### 1. 调试信息输出

```c
// 打印详细调试信息
TwelvePulse_PrintDebugInfo();

// 输出示例：
// === Twelve Pulse Debug Info ===
// State: 4
// Current pulse: 12/12
// Enabled: 0, Completed: 1
// Test voltage: 8.00 V
// Estimated theta: 1.047 rad (60.0 deg)
// Confidence: 0.857
// Current responses:
// Pulse  0: Id=0.123, Iq=0.045, Mag=0.131, Angle=0.0 deg
// Pulse  1: Id=0.098, Iq=0.087, Mag=0.131, Angle=30.0 deg
// ...
```

### 2. 结果验证

```c
// 验证测试结果质量
TwelvePulse_ValidateResults();

// 输出示例：
// === Twelve Pulse Validation ===
// Average magnitude: 0.125 A
// Min magnitude: 0.089 A
// Max magnitude: 0.167 A
// Magnitude ratio: 1.876
// High confidence estimate - GOOD
// Response variance: 0.001234
// Response consistency - EXCELLENT
```

## 参数说明

### 关键参数

| 参数 | 默认值 | 说明 | 调整建议 |
|------|--------|------|----------|
| test_voltage | 8.0V | 测试电压 | 根据电机额定电压调整，通常为额定电压的10-20% |
| test_time | 10ms | 脉冲持续时间 | 确保电流能稳定建立，大电机可适当增加 |
| settle_time | 2ms | 稳定时间 | 等待电流稳定后再采样，高电感电机需要更长时间 |

### 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 位置精度 | ±15° | 在理想条件下的位置估计精度 |
| 置信度 | >0.5 | 置信度越高，估计越可靠 |
| 测试时间 | <200ms | 完成12个脉冲的总时间 |
| 电流响应 | >0.1A | 足够的电流响应确保可靠检测 |

## 故障排除

### 常见问题

1. **置信度低 (<0.3)**
   - 检查测试电压是否足够
   - 确认电机连接正确
   - 检查电流采样精度

2. **电流响应小 (<0.05A)**
   - 增加测试电压
   - 检查电机是否卡死
   - 确认逆变器工作正常

3. **响应不一致**
   - 检查电机参数设置
   - 确认电机类型匹配
   - 检查位置传感器干扰

### 调试技巧

1. **逐步调试**：先用示例代码验证基本功能
2. **参数优化**：根据电机特性调整测试参数
3. **波形观察**：用示波器观察电压和电流波形
4. **结果对比**：与已知位置进行对比验证

## 集成说明

### 与FOC控制器集成

十二脉冲算法已集成到FOC主控制循环中：

1. 调用 `FOC_TwelvePulse_Start()` 启动测试
2. 系统自动切换到 `TwelvePulse_Mode`
3. 完成后自动切换到 `Sensorless_Mode`
4. 开始正常的无位置传感器控制

### 与无位置传感器算法配合

```c
// 完整的启动序列
void Motor_Startup_Sequence(void) {
    // 1. 十二脉冲初始定位
    FOC_TwelvePulse_Start();
    
    // 等待完成...
    
    // 2. 自动切换到无位置传感器模式
    // (由系统自动完成)
    
    // 3. 开始电机运行
    FOC.Mode = Speed_Mode;
    Speed_Ref = 100.0f; // 设置目标转速
}
```

## 进阶使用

### 自定义算法参数

```c
// 为不同电机类型优化参数
void OptimizeForMotorType(MotorType_t motor_type) {
    switch (motor_type) {
        case MOTOR_HIGH_SPEED:
            Sensorless.twelve_pulse.test_voltage = 6.0f;
            Sensorless.twelve_pulse.test_time = 0.008f;
            break;
        case MOTOR_HIGH_TORQUE:
            Sensorless.twelve_pulse.test_voltage = 10.0f;
            Sensorless.twelve_pulse.test_time = 0.015f;
            break;
    }
}
```

### 结果后处理

```c
// 自定义位置估计算法
float CustomPositionEstimation(TwelvePulse_t *pulse) {
    // 实现自定义的位置计算算法
    // 例如：考虑电机的磁路饱和特性
    return calculated_position;
}
```

## 注意事项

1. **安全性**：测试过程中电机可能产生轻微转动，确保安全
2. **电流限制**：设置合适的电流保护限制
3. **热管理**：连续多次测试时注意散热
4. **干扰**：避免在测试过程中受到外部干扰
5. **标定**：不同电机可能需要调整参数以获得最佳效果

---

更多详细信息请参考源代码中的注释和示例。
