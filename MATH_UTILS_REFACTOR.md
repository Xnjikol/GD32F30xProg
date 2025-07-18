# 数学工具函数重构总结

## 概述
将原本分散在 `foc.c` 和 `sensorless.c` 中的工具函数提取到新的公共模块中，提高代码的复用性和维护性。

## 创建的新文件

### 1. `Core/Inc/math_utils.h`
- 定义了通用的数学工具函数接口
- 包含角度处理、限幅、符号函数等实用工具
- 统一了数学宏定义（COS、SIN、MOD、ABS等）
- 添加了数学常数定义（M_PI、M_PI_2、SQRT3等）

### 2. `Core/Src/math_utils.c`
- 实现了角度归一化函数：`MathUtils_WrapAngle2Pi()` 和 `MathUtils_WrapAnglePi()`
- 实现了角度差值计算：`MathUtils_AngleDifference()`
- 实现了其他实用工具函数：限幅、符号、死区、线性插值等

## 修改的文件

### 3. `Core/Src/sensorless.c`
- 添加了 `#include "math_utils.h"`
- 将所有 `Sensorless_WrapTheta2Pi()` 调用替换为 `MathUtils_WrapAngle2Pi()`
- 删除了原有的 `Sensorless_WrapTheta2Pi()` 和 `Sensorless_AngleDifference()` 函数定义
- 删除了静态函数声明

### 4. `Core/Inc/sensorless.h`
- 添加了 `#include "math_utils.h"`
- 删除了 `Sensorless_AngleDifference()` 函数声明

### 5. `Core/Src/foc.c`
- 添加了 `#include "math_utils.h"`
- 将 `wrap_theta_2pi()` 调用替换为 `MathUtils_WrapAngle2Pi()`
- 将角度差值计算替换为 `MathUtils_AngleDifference()`
- 删除了原有的 `wrap_theta_2pi()` 函数定义和声明

### 6. `Core/Inc/foc.h`
- 添加了 `#include "math_utils.h"`
- 移除了重复的数学宏定义（现在由 math_utils.h 提供）
- 移除了重复的数学常数定义（SQRT3）

### 7. `CMakeLists.txt`
- 在源文件列表中添加了 `Core/Src/math_utils.c`

## 提供的工具函数

### 角度处理函数
- `MathUtils_WrapAngle2Pi()` - 角度归一化到 [0, 2π)
- `MathUtils_WrapAnglePi()` - 角度归一化到 [-π, π)
- `MathUtils_AngleDifference()` - 计算两个角度的最小差值

### 数学工具函数
- `MathUtils_Clamp()` - 限幅函数
- `MathUtils_Sign()` - 符号函数
- `MathUtils_Deadband()` - 死区函数
- `MathUtils_LinearInterpolate()` - 线性插值

### 内联函数
- `MathUtils_DegreesToRadians()` - 角度转弧度
- `MathUtils_RadiansToDegrees()` - 弧度转角度
- `MathUtils_Square()` - 平方函数
- `MathUtils_VectorMagnitude()` - 向量模长

## 优势

1. **代码复用**：消除了重复的函数定义
2. **统一接口**：所有数学工具函数都有统一的命名和接口
3. **易于维护**：数学函数集中管理，便于调试和优化
4. **模块化**：将通用功能从具体业务逻辑中分离
5. **扩展性**：方便添加新的数学工具函数

## 编译验证
- 代码重构后编译成功，无错误和警告
- 内存使用情况正常：
  - FLASH: 30808 B / 256 KB (11.75%)
  - RAM: 5240 B / 48 KB (10.66%)

## 使用方法
在需要使用数学工具函数的文件中，只需包含 `#include "math_utils.h"` 即可使用所有提供的数学工具函数。

原来的函数调用：
```c
Sensorless_WrapTheta2Pi(angle)
wrap_theta_2pi(angle)
Sensorless_AngleDifference(angle1, angle2)
```

现在统一使用：
```c
MathUtils_WrapAngle2Pi(angle)
MathUtils_AngleDifference(angle1, angle2)
```
