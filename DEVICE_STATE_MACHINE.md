# 设备状态机重构说明

## 问题描述

原有程序存在逻辑混乱的问题：
1. 程序运行时需要通过 `Peripheral_GetSystemFrequency` 函数获取系统参数（Ts、freq）
2. 初始化后需要正常运行逻辑（虽然ADC采样不正常但要运行）
3. 一段时间后需要通过CCP协议手动触发完整初始化（修改 Device.Mode）
4. 这导致了两遍初始化，使得主中断函数逻辑混乱

## 解决方案

### 新的设备状态定义

```c
typedef enum
{
  BASIC_INIT,      // 基础初始化：仅获取系统参数
  BASIC_READY,     // 基础就绪：等待完整初始化触发
  FULL_INIT,       // 完整初始化：用户触发的完整参数初始化
  READY,           // 就绪：完整初始化完成，准备运行
  RUNNING          // 运行：正常工作状态
} DeviceStateEnum_t;

typedef struct
{
  DeviceStateEnum_t Mode;
  bool basic_init_done;     // 基础初始化完成标志
  bool full_init_done;      // 完整初始化完成标志
  bool system_params_valid; // 系统参数有效标志
} DeviceState_t;
```

### 状态转换流程

1. **BASIC_INIT**: 系统启动时的状态
   - 执行 `Main_Int_Basic_Init()` 
   - 调用 `Peripheral_InitProtectParameter()` 和 `Peripheral_GetSystemFrequency()`
   - 获取到有效的系统参数后转到 BASIC_READY

2. **BASIC_READY**: 基础初始化完成，等待用户触发
   - 继续执行基本的外设更新逻辑
   - 系统参数已可用，但FOC控制尚未完全初始化
   - 通过CCP协议调用 `Main_Int_TriggerFullInit()` 可以触发完整初始化

3. **FULL_INIT**: 完整初始化状态
   - 执行 `Main_Int_Parameter_Init()`
   - 初始化所有FOC相关参数
   - 完成后转到 READY 状态

4. **READY**: 完整初始化完成
   - 准备进入运行状态
   - 立即转到 RUNNING 状态

5. **RUNNING**: 正常运行状态
   - 执行完整的FOC控制逻辑
   - 调用 `FOC_Main()` 进行电机控制

### 新增的API函数

#### 触发完整初始化
```c
void Main_Int_TriggerFullInit(void);
```
在CCP协议中调用此函数来触发完整初始化。

#### 状态查询函数
```c
bool Main_Int_IsBasicReady(void);    // 检查基础初始化是否完成
bool Main_Int_IsFullyReady(void);    // 检查完整初始化是否完成
```

### 使用示例

#### CCP协议中触发完整初始化
```c
// 在CCP协议处理函数中
if (received_full_init_command)
{
  if (Main_Int_IsBasicReady())
  {
    Main_Int_TriggerFullInit();
    // 发送确认响应
  }
  else
  {
    // 发送错误响应：基础初始化未完成
  }
}
```

#### 监控初始化状态
```c
// 在其他地方监控初始化状态
if (Main_Int_IsBasicReady())
{
  // 可以开始发送CCP命令触发完整初始化
}

if (Main_Int_IsFullyReady())
{
  // 系统已完全就绪，可以开始电机控制
}
```

## 优势

1. **逻辑清晰**: 明确分离了基础初始化和完整初始化
2. **可控性强**: 外部可以精确控制何时进行完整初始化
3. **状态明确**: 可以清楚地知道系统当前处于哪个初始化阶段
4. **扩展性好**: 便于添加更多的初始化状态或检查点
5. **避免重复**: 基础系统参数只获取一次，避免重复初始化

## 注意事项

1. 系统启动后会自动进入 BASIC_INIT 状态
2. 只有在 BASIC_READY 状态下才能触发完整初始化
3. 系统参数（FOC.Ts, FOC.freq）在基础初始化阶段获取，之后不再重复获取
4. ADC校准在完整初始化阶段进行，确保传感器数据的准确性
