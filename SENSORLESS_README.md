# 滤波器模块重构说明

## 重构内容

为了消除代码重复并提高代码复用性，我们将滤波器相关的代码重构为独立的模块：

### 新增文件
- `Core/Inc/filter.h` - 滤波器模块头文件
- `Core/Src/filter.c` - 滤波器模块实现
- `Core/Src/filter_usage_example.c` - 使用示例

### 支持的滤波器类型
1. **低通滤波器 (LowPassFilter_t)** - 去除高频噪声
2. **高通滤波器 (HighPassFilter_t)** - 去除直流分量
3. **带通滤波器 (BandPassFilter_t)** - 提取特定频率分量
4. **陷波滤波器 (NotchFilter_t)** - 消除特定频率干扰
5. **移动平均滤波器 (MovingAverageFilter_t)** - 平滑信号
6. **中值滤波器 (MedianFilter_t)** - 去除脉冲干扰
7. **卡尔曼滤波器 (KalmanFilter_t)** - 最优估计
8. **巴特沃斯滤波器 (ButterworthFilter_t)** - 高阶低通滤波

### 代码重构优势
- **消除重复代码** - 多个文件中的滤波器定义合并为一个模块
- **提高代码复用性** - 统一的滤波器接口，便于在不同模块中使用
- **增强可维护性** - 集中管理滤波器算法，便于调试和优化
- **扩展性好** - 易于添加新的滤波器类型
- **标准化接口** - 统一的初始化、更新和重置函数

### 使用方法
```c
#include "filter.h"

// 创建滤波器实例
static LowPassFilter_t my_filter;

// 初始化滤波器
LowPassFilter_Init(&my_filter, 100.0f, T_2K_HZ);  // 100Hz截止频率

// 在循环中使用
float filtered_value = LowPassFilter_Update(&my_filter, raw_value);
```

### 迁移指南
1. 在需要使用滤波器的文件中包含 `#include "filter.h"`
2. 将旧的滤波器结构体替换为新的标准结构体
3. 使用标准的初始化和更新函数
4. 删除重复的滤波器定义代码

# 无位置传感器算法库

本项目为GD32F30x系列微控制器的FOC电机控制添加了完整的无位置传感器算法支持。

## 功能特性

### 支持的算法

1. **高频注入 (HFI)** - 适用于低速和零速运行
2. **滑膜观测器 (SMO)** - 适用于中高速运行
3. **PLL锁相环** - 用于位置跟踪和滤波
4. **十二脉冲检测** - 用于确定初始转子位置
5. **磁链观测器** - 基于磁链的位置估计
6. **混合算法** - 自动根据速度切换最优算法

### 主要特点

- **模块化设计** - 每个算法都封装在独立的结构中
- **易于配置** - 提供用户友好的接口函数
- **自动切换** - 根据电机状态自动选择最优算法
- **故障检测** - 内置算法可靠性监控
- **性能监控** - 实时评估算法性能
- **调试支持** - 丰富的调试信息和测试模式

## 文件结构

```
Core/
├── Inc/
│   ├── sensorless.h              # 核心算法头文件
│   ├── sensorless_interface.h    # 用户接口头文件
│   └── foc.h                     # 更新的FOC头文件
└── Src/
    ├── sensorless.c              # 核心算法实现
    ├── sensorless_interface.c    # 用户接口实现
    ├── sensorless_example.c      # 使用示例
    └── foc.c                     # 更新的FOC实现
```

## 快速开始

### 1. 基础配置

在 `foc.h` 中启用无位置传感器：

```c
#define SENSORLESS_POSITION    /* 启用无位置传感器 */
```

### 2. 初始化

```c
#include "sensorless_interface.h"

void init_sensorless(void)
{
    // 初始化无位置传感器接口
    SensorlessInterface_Init();
    
    // 配置高频注入参数
    SensorlessInterface_ConfigHFI(1000.0f, 5.0f);  // 1kHz, 5V
    
    // 配置滑膜观测器参数
    SensorlessInterface_ConfigSMO(200.0f, 100.0f);  // 增益200, 截止频率100Hz
    
    // 设置混合算法模式
    SensorlessInterface_SetAlgorithm(SENSORLESS_HYBRID);
    
    // 启用无位置传感器
    SensorlessInterface_Enable(1);
}
```

### 3. 运行时更新

```c
void main_loop(void)
{
    while (1) {
        // 更新无位置传感器
        SensorlessInterface_Update();
        
        // 获取估计的位置和速度
        float theta = SensorlessInterface_GetTheta();
        float speed = SensorlessInterface_GetSpeed();
        
        // 检查故障
        uint8_t faults = SensorlessInterface_GetFaultFlags();
        if (faults != SENSORLESS_FAULT_NONE) {
            // 处理故障
            handle_sensorless_faults(faults);
        }
        
        // 在FOC中使用
        FOC.Mode = Sensorless_Mode;
        FOC_Main();
        
        delay_us(500);  // 2kHz更新频率
    }
}
```

### 4. 启动序列

```c
void motor_startup(void)
{
    // 启动十二脉冲序列确定初始位置
    SensorlessInterface_StartupSequence();
    
    // 等待启动完成
    while (!SensorlessInterface.twelve_pulse_completed) {
        SensorlessInterface_Update();
        
        // 检查启动超时
        if (SensorlessInterface_GetFaultFlags() & SENSORLESS_FAULT_STARTUP_FAIL) {
            // 启动失败处理
            break;
        }
    }
    
    // 启动完成，切换到运行模式
    if (SensorlessInterface.twelve_pulse_completed) {
        FOC.Mode = Sensorless_Mode;
    }
}
```

## 算法详解

### 高频注入 (HFI)

- **原理**: 在d轴注入高频信号，检测转子凸极性
- **适用场景**: 低速 (< 200 rpm) 和零速
- **优点**: 零速可用，精度高
- **缺点**: 噪声大，功耗高

**配置参数**:
- 注入频率: 800-2000 Hz
- 注入电压: 2-10 V
- 滤波器截止频率: 注入频率的 20%

### 滑膜观测器 (SMO)

- **原理**: 观测反电动势估计位置
- **适用场景**: 中高速 (> 150 rpm)
- **优点**: 计算简单，鲁棒性好
- **缺点**: 低速性能差

**配置参数**:
- 滑膜增益: 100-500
- 滞环宽度: 0.05-0.2
- 反EMF滤波器: 50-200 Hz

### PLL锁相环

- **原理**: 跟踪反EMF相位
- **适用场景**: 与SMO配合使用
- **优点**: 滤波效果好，响应快
- **缺点**: 需要准确的反EMF信号

### 十二脉冲检测

- **原理**: 在12个方向施加脉冲电压，检测电流响应
- **适用场景**: 启动时确定初始位置
- **优点**: 可以确定绝对位置
- **缺点**: 需要电机停止

### 混合算法

- **原理**: 根据速度自动切换HFI和SMO
- **切换逻辑**:
  - 速度 < 150 rpm: 使用HFI
  - 速度 > 200 rpm: 使用SMO
  - 中间区域: 根据迟滞切换

## 参数调优

### 电机参数

确保以下电机参数正确设置：

```c
Motor.Rs = 0.65f;       // 定子电阻 (Ω)
Motor.Ld = 0.1175f;     // d轴电感 (H)
Motor.Lq = 0.044f;      // q轴电感 (H)
Motor.Pn = 2;           // 极对数
```

### HFI参数调优

1. **注入频率**: 从1000Hz开始，根据电机噪声调整
2. **注入电压**: 从5V开始，根据电流响应调整
3. **滤波器**: 截止频率设为注入频率的20%

### SMO参数调优

1. **滑膜增益**: 从200开始，增加直到抖振明显
2. **滞环宽度**: 从0.1开始，调整到合适的抖振水平
3. **反EMF滤波器**: 从100Hz开始，根据噪声调整

## 故障处理

### 故障类型

- `SENSORLESS_FAULT_POSITION_LOST`: 位置估计丢失
- `SENSORLESS_FAULT_SPEED_UNSTABLE`: 速度估计不稳定
- `SENSORLESS_FAULT_ALGORITHM_FAIL`: 算法失效
- `SENSORLESS_FAULT_STARTUP_FAIL`: 启动失败

### 故障处理策略

```c
void handle_sensorless_faults(uint8_t fault_flags)
{
    if (fault_flags & SENSORLESS_FAULT_POSITION_LOST) {
        // 重新启动位置估计
        SensorlessInterface_Reset();
        SensorlessInterface_StartupSequence();
    }
    
    if (fault_flags & SENSORLESS_FAULT_SPEED_UNSTABLE) {
        // 降低观测器增益
        SensorlessInterface_ConfigSMO(150.0f, 80.0f);
    }
    
    if (fault_flags & SENSORLESS_FAULT_STARTUP_FAIL) {
        // 停止电机
        STOP = 1;
        FOC.Mode = IDLE;
    }
}
```

## 性能优化

### 算法频率

- 主循环: 2kHz (与PWM同步)
- 位置更新: 2kHz
- 速度计算: 200Hz (10倍降频)

### 内存优化

- 使用单精度浮点数
- 复用滤波器结构
- 避免动态内存分配

### 计算优化

- 使用ARM DSP库函数
- 预计算三角函数
- 向量化运算

## 调试功能

### 调试信息

```c
char debug_buffer[512];
SensorlessInterface_GetDebugInfo(debug_buffer, sizeof(debug_buffer));
printf("%s", debug_buffer);
```

### 性能监控

```c
float confidence = SensorlessInterface_GetEstimationConfidence();
float pos_error = SensorlessInterface_GetPositionError();
float speed_error = SensorlessInterface_GetSpeedError();
```

### 测试模式

```c
// 启用测试模式
SensorlessInterface_TestMode(1);

// 强制使用特定算法
SensorlessInterface_ForceAlgorithm(SENSORLESS_HFI);
```

## 注意事项

1. **电机参数**: 必须准确设置电机参数
2. **采样频率**: 建议使用2kHz或更高
3. **电流采样**: 需要三相电流采样
4. **电压反馈**: 如果可用，建议使用电压反馈
5. **干扰抑制**: 注意电磁干扰对算法的影响

## 常见问题

### Q: 为什么HFI算法噪声很大？

A: 这是正常现象。可以通过以下方式减少噪声：
- 降低注入电压
- 优化滤波器参数
- 使用更高的注入频率

### Q: SMO算法在低速时不稳定？

A: SMO依赖反电动势，在低速时信号微弱。建议：
- 在低速时切换到HFI
- 优化滤波器参数
- 增加观测器增益

### Q: 启动时位置估计不准确？

A: 检查以下几点：
- 十二脉冲电压是否足够
- 电机是否真正静止
- 电机参数是否正确

## 扩展功能

### 自定义算法

可以通过修改 `sensorless.c` 添加自定义算法：

```c
// 添加新的算法类型
typedef enum {
    // ... 现有算法 ...
    SENSORLESS_CUSTOM,
} SensorlessAlgorithm_t;

// 实现自定义算法
void Custom_Algorithm_Update(/* 参数 */) {
    // 自定义算法实现
}
```

### 实时调参

可以通过CAN或串口实时调整参数：

```c
void update_hfi_params(float freq, float voltage) {
    SensorlessInterface_ConfigHFI(freq, voltage);
}
```

## 参考资料

1. 《无刷直流电机控制技术》
2. 《基于滑膜观测器的无位置传感器控制》
3. 《高频注入无位置传感器控制》
4. STM32电机控制应用笔记

## 版本历史

- v1.0: 初始版本，支持HFI、SMO、PLL、十二脉冲算法
- v1.1: 添加磁链观测器和混合算法
- v1.2: 添加用户接口和故障检测
- v1.3: 添加性能监控和调试功能

## 技术支持

如需技术支持，请提供以下信息：
1. 电机参数
2. 调试信息输出
3. 故障现象描述
4. 配置参数设置
