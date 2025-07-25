# 无传感器控制使用示例

## 重构后的结构说明

重构后，无传感器控制模块分为两层：

### 1. 算法层 (`Sensorless/`)
- **位置**: `e:\Doc\Work\FRECON\Prj\GD32F30xProg\Sensorless\`
- **功能**: 包含核心算法实现和私有函数
- **文件**:
  - `Inc/flux_observer.h` - 磁链观测器算法接口
  - `Src/flux_observer.c` - 磁链观测器算法实现

### 2. 接口层 (`Middleware/Sensorless_Interface/`)
- **位置**: `e:\Doc\Work\FRECON\Prj\GD32F30xProg\Middleware\Sensorless_Interface\`
- **功能**: 提供高级API接口，封装底层算法
- **文件**:
  - `Inc/sensorless_interface.h` - 高级接口定义
  - `Src/sensorless_interface.c` - 高级接口实现

## 使用方法

### 基本使用步骤

```c
#include "sensorless_interface.h"

void main(void) {
    // 1. 获取默认配置
    sensorless_config_t config;
    sensorless_get_default_config(&config);
    
    // 2. 根据实际电机修改参数
    config.motor_rs = 0.185f;              // 定子电阻
    config.motor_ls = 0.00032f;            // 定子电感
    config.motor_flux_rated = 0.00656f;    // 额定磁链
    config.motor_pole_pairs = 4.0f;        // 极对数
    config.control_ts = 0.0001f;           // 控制周期 10kHz
    
    // 3. 初始化无传感器控制系统
    if (sensorless_init(&config) != 0) {
        // 初始化失败处理
        return;
    }
    
    // 4. 使能系统
    sensorless_set_enable(1);
    
    // 主循环
    while (1) {
        // 获取电压和电流测量值
        float ua = get_phase_a_voltage();
        float ub = get_phase_b_voltage();
        float uc = get_phase_c_voltage();
        float ia = get_phase_a_current();
        float ib = get_phase_b_current();
        float ic = get_phase_c_current();
        
        // 5. 执行无传感器控制
        sensorless_execute(ua, ub, uc, ia, ib, ic);
        
        // 6. 获取结果
        if (sensorless_is_valid()) {
            float rotor_angle = sensorless_get_rotor_angle();
            float rotor_speed = sensorless_get_rotor_speed();
            float rotor_speed_mech = sensorless_get_rotor_speed_mech();
            
            // 使用估计的位置和速度进行FOC控制
            foc_control(rotor_angle, rotor_speed);
        }
        
        // 等待下一个控制周期
        delay_us(100);  // 10kHz控制频率
    }
}
```

### 高级使用

```c
// 获取完整输出结构
sensorless_output_t output;
sensorless_get_output(&output);

if (output.valid) {
    printf("转子角度: %.3f rad\n", output.rotor_angle);
    printf("电角速度: %.3f rad/s\n", output.rotor_speed);
    printf("机械角速度: %.3f rad/s\n", output.rotor_speed_mech);
    printf("磁链幅值: %.6f Wb\n", output.flux_magnitude);
    printf("状态: %d\n", output.state);
}

// 状态检查
switch (sensorless_get_state()) {
    case SENSORLESS_STATE_STOPPED:
        // 系统停止状态
        break;
    case SENSORLESS_STATE_STARTING:
        // 系统启动状态，磁链建立中
        break;
    case SENSORLESS_STATE_RUNNING:
        // 系统正常运行状态
        break;
    case SENSORLESS_STATE_ERROR:
        // 系统错误状态
        break;
}
```

## 优势

### 分层架构的好处

1. **算法层独立**: 算法实现与应用解耦，便于算法优化和测试
2. **接口统一**: 应用层只需关心高级接口，不用了解算法细节
3. **易于扩展**: 可以轻松添加新的观测器算法（SMO、EKF等）
4. **参数管理**: 统一的参数配置和状态管理
5. **错误处理**: 完善的状态检查和数据有效性验证

### 应用层简化

- 不需要了解Clarke变换等细节
- 自动处理速度计算和滤波
- 统一的状态管理和错误处理
- 简洁的API接口

## 编译配置

重构后的CMake配置已集成到Middleware总的CMakeLists.txt中，会自动处理依赖关系：

```cmake
# 应用代码只需要链接到Middleware即可
target_link_libraries(your_application PUBLIC
    Middleware  # 自动包含所有Middleware接口，包括Sensorless_Interface
)
```

无需单独的CMakeLists.txt文件，所有配置都在Middleware/CMakeLists.txt中统一管理。

## 注意事项

1. **参数配置**: 确保电机参数（Rs、Ls、磁链等）配置正确
2. **控制频率**: 建议控制频率不低于5kHz，推荐10kHz
3. **数据有效性**: 在使用估计值之前，务必检查`sensorless_is_valid()`
4. **状态监控**: 监控系统状态，在启动状态时可能需要特殊处理
5. **低速性能**: 电压模型在极低速时性能有限，可能需要其他启动策略
