# 十二脉冲初始定位示例代码

## 概述

本文件夹包含十二脉冲初始定位功能的示例代码和参考实现，**这些文件不包含在主程序的编译中**，仅供开发者参考和学习使用。

## 文件结构

```
Examples/TwelvePulse/
├── Inc/
│   ├── twelve_pulse_example.h          # 十二脉冲示例函数头文件
│   └── main_example_twelve_pulse.h     # 主程序集成示例头文件
├── Src/
│   ├── twelve_pulse_example.c          # 十二脉冲示例函数实现
│   └── main_example_twelve_pulse.c     # 主程序集成示例实现
└── README.md                           # 本说明文件
```

## 文件说明

### twelve_pulse_example.c/h
包含以下示例函数：
- `TwelvePulse_Example()` - 基础的十二脉冲测试示例
- `TwelvePulse_AutoTest()` - 自动测试序列，包含重试机制
- `TwelvePulse_PrintDebugInfo()` - 调试信息输出
- `TwelvePulse_ConfigureParameters()` - 参数配置示例
- `TwelvePulse_ValidateResults()` - 结果验证函数

### main_example_twelve_pulse.c/h
包含以下示例函数：
- `main_twelve_pulse_example()` - 完整的主程序示例
- `simple_twelve_pulse_test()` - 简化的测试函数
- `interactive_twelve_pulse_test()` - 交互式测试
- `motor_startup_with_twelve_pulse()` - 电机启动序列示例

## 使用方法

### 1. 复制需要的函数到你的项目中

```c
// 例如，将自动测试函数复制到你的main.c中
#include "foc.h"
#include "sensorless.h"

// 复制这个函数到你的代码中
int TwelvePulse_AutoTest(void) {
    // ... 复制函数实现
}

// 在主程序中调用
int main(void) {
    // 系统初始化...
    
    // 运行十二脉冲测试
    int result = TwelvePulse_AutoTest();
    if (result == 0) {
        printf("十二脉冲定位成功!\n");
    }
    
    // 继续其他逻辑...
}
```

### 2. 包含整个示例文件（可选）

如果你想要在项目中临时包含这些示例文件进行测试，可以：

1. 在CMakeLists.txt中临时添加：
```cmake
# 临时添加示例文件用于测试
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Examples/TwelvePulse/Src/twelve_pulse_example.c
    # Examples/TwelvePulse/Src/main_example_twelve_pulse.c  # 如果需要
)

# 添加示例头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Examples/TwelvePulse/Inc
)
```

2. 在你的main.c中包含头文件：
```c
#include "twelve_pulse_example.h"

int main(void) {
    // 系统初始化...
    
    // 运行示例
    TwelvePulse_AutoTest();
    
    // 或者运行完整的电机启动序列
    motor_startup_with_twelve_pulse();
    
    return 0;
}
```

**注意：测试完成后记得从CMakeLists.txt中移除示例文件。**

## 核心API使用

十二脉冲的核心功能已经集成到主程序中，你只需要调用以下API：

```c
#include "foc.h"
#include "sensorless.h"

// 启动十二脉冲测试
FOC_TwelvePulse_Start();

// 等待完成
while (!TwelvePulse_IsCompleted(&Sensorless.twelve_pulse)) {
    delay_ms(1);
}

// 获取结果
float theta = TwelvePulse_GetTheta(&Sensorless.twelve_pulse);
float confidence = TwelvePulse_GetConfidence(&Sensorless.twelve_pulse);

printf("位置: %.2f°, 置信度: %.3f\n", 
       theta * 180.0f / M_PI, confidence);
```

## 快速集成代码片段

### 最简单的使用方式

```c
// 在适当的地方调用（例如电机启动时）
FOC_TwelvePulse_Start();

// 等待完成（系统会自动切换到无位置传感器模式）
uint32_t timeout = 0;
while (!TwelvePulse_IsCompleted(&Sensorless.twelve_pulse) && timeout < 5000) {
    delay_ms(1);
    timeout++;
}

if (TwelvePulse_IsCompleted(&Sensorless.twelve_pulse)) {
    float confidence = TwelvePulse_GetConfidence(&Sensorless.twelve_pulse);
    if (confidence > 0.5f) {
        printf("十二脉冲定位成功, 置信度: %.3f\n", confidence);
        // 继续正常运行
    } else {
        printf("十二脉冲定位置信度低, 切换到传感器模式\n");
        FOC.Mode = Speed_Mode;
    }
} else {
    printf("十二脉冲超时, 切换到传感器模式\n");
    FOC.Mode = Speed_Mode;
}
```

## 调试建议

1. **查看调试信息**：复制`TwelvePulse_PrintDebugInfo()`函数来查看详细状态
2. **验证结果**：复制`TwelvePulse_ValidateResults()`函数来评估结果质量
3. **参数调整**：复制`TwelvePulse_ConfigureParameters()`函数来调整测试参数

## 注意事项

- 这些示例文件**不会自动编译**到主程序中
- 使用时需要根据你的项目结构调整头文件路径
- 示例代码中的printf需要确保你的项目支持标准输出
- 测试时确保电机处于安全状态

## 技术支持

更多详细信息请参考：
- `../../TWELVE_PULSE_QUICKSTART.md` - 快速使用指南
- `../../Core/Inc/sensorless.h` - 核心算法接口
- `../../Core/Inc/foc.h` - FOC控制器接口

如有问题，建议先运行示例代码进行验证，然后根据实际情况调整参数。
