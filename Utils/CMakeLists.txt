# Utils library
# 实用工具库，包含滤波器等常用算法实现

# 添加源文件
set(Utils_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/filter.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pll.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/signal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/transformation.c
)

add_library(Utils STATIC
    ${Utils_SOURCES}
)

# 添加头文件目录  
target_include_directories(Utils PUBLIC
    Inc
)

# 链接依赖库
target_link_libraries(Utils PUBLIC
    MCU # 链接到MCU库以获取GD32F30x头文件和基本类型定义
)

# 编译选项
target_compile_options(Utils PRIVATE
    -Wall
    -Wextra
    -O2
)
