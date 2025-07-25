# Sensorless library
# 无传感器控制核心算法库，只包含算法实现和私有函数

# 添加源文件
set(Sensorless_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/flux_observer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/hf_injection.c
)

add_library(Sensorless STATIC
    ${Sensorless_SOURCES}
)

# 添加头文件目录  
target_include_directories(Sensorless PUBLIC
    Inc
)

# 链接依赖库
target_link_libraries(Sensorless PUBLIC
    MCU # 链接到MCU库以获取GD32F30x头文件和基本类型定义
    Utils # 链接到Utils库以获取滤波器、PID等工具算法
)

# 编译选项
target_compile_options(Sensorless PRIVATE
    -Wall
    -Wextra
    -O2
)
