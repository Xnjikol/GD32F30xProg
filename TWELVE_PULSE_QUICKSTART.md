# 十二脉冲初始定位 - 快速使用指南

## 功能概述

十二脉冲初始定位算法已完成实现，提供以下功能：

### ✅ 完成的功能
- **完整的十二脉冲算法实现**
- **与FOC控制器的集成**
- **多种位置估算算法**（最大响应、加权平均、谐波分析）
- **置信度评估和质量检查**
- **自动参数配置**
- **详细的调试和验证工具**
- **完整的示例代码**

## 快速开始

### 1. 最简单的使用方式

```c
#include "twelve_pulse_example.h"

// 在main函数中调用
int result = TwelvePulse_AutoTest();
if (result == 0) {
    printf("十二脉冲定位成功!\n");
}
```

### 2. 集成到电机启动序列

```c
#include "foc.h"
#include "twelve_pulse_example.h"

void motor_startup(void) {
    // 启动十二脉冲
    FOC_TwelvePulse_Start();
    
    // 等待完成（系统会自动切换到无位置传感器模式）
    while (!TwelvePulse_IsCompleted(&Sensorless.twelve_pulse)) {
        delay_ms(1);
    }
    
    // 开始电机运行
    Speed_Ref = 100.0f; // 设置目标转速
}
```

### 3. 手动控制和调试

```c
// 配置参数
TwelvePulse_ConfigureParameters();

// 查看调试信息
TwelvePulse_PrintDebugInfo();

// 验证结果质量
TwelvePulse_ValidateResults();
```

## 文件结构

```
Core/
├── Inc/
│   ├── sensorless.h                    # 核心算法头文件
│   ├── twelve_pulse_example.h          # 示例代码头文件
│   └── main_example_twelve_pulse.h     # 主程序示例头文件
└── Src/
    ├── sensorless.c                    # 核心算法实现
    ├── twelve_pulse_example.c          # 示例和工具函数
    ├── main_example_twelve_pulse.c     # 主程序示例
    └── foc.c                          # 已集成十二脉冲模式

Documentation/
├── TWELVE_PULSE_README.md             # 详细技术文档
└── 本文件                             # 快速使用指南
```

## 主要API

### 核心函数
```c
// 启动十二脉冲测试
void FOC_TwelvePulse_Start(void);

// 检查是否完成
uint8_t TwelvePulse_IsCompleted(TwelvePulse_t *pulse);

// 获取估计位置
float TwelvePulse_GetTheta(TwelvePulse_t *pulse);

// 获取置信度
float TwelvePulse_GetConfidence(TwelvePulse_t *pulse);
```

### 便捷函数
```c
// 自动测试（推荐）
int TwelvePulse_AutoTest(void);

// 简单测试
int TwelvePulse_Example(void);

// 配置参数
void TwelvePulse_ConfigureParameters(void);

// 调试信息
void TwelvePulse_PrintDebugInfo(void);

// 结果验证
void TwelvePulse_ValidateResults(void);
```

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| test_voltage | 8.0V | 测试电压 |
| test_time | 10ms | 单个脉冲时间 |
| settle_time | 2ms | 稳定时间 |

## 性能指标

- **位置精度**: ±15°
- **测试时间**: <200ms
- **置信度范围**: 0.0 - 1.0
- **建议置信度**: >0.5

## 集成状态

### ✅ 已完成
- [x] 核心算法实现
- [x] FOC集成（新增TwelvePulse_Mode）
- [x] 示例代码
- [x] 调试工具
- [x] 文档

### 🔧 需要根据实际电机调整
- [ ] 测试电压参数
- [ ] 测试时间参数
- [ ] 电流保护限制

## 故障排除

1. **编译错误**: 确保包含了所有必要的头文件
2. **置信度低**: 调整test_voltage参数
3. **电流响应小**: 检查电机连接和逆变器状态
4. **测试超时**: 增加test_time或检查系统时基

## 下一步

1. **测试验证**: 使用`TwelvePulse_AutoTest()`进行初步测试
2. **参数优化**: 根据实际电机调整参数
3. **集成验证**: 测试与整个系统的集成效果
4. **性能评估**: 评估定位精度和可靠性

## 技术支持

详细的技术文档请参考 `TWELVE_PULSE_README.md`。

如有问题，可以：
1. 查看调试输出 `TwelvePulse_PrintDebugInfo()`
2. 验证结果质量 `TwelvePulse_ValidateResults()`
3. 参考示例代码 `twelve_pulse_example.c`

---

**注意**: 确保在测试时电机处于安全状态，测试过程中可能产生轻微转动。
