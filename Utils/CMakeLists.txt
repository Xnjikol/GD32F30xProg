# Utils library
# 实用工具库，包含滤波器等常用算法实现

add_library(Utils STATIC)

# 添加源文件
target_sources(Utils PRIVATE
    Src/filter.c
)

# 添加头文件目录  
target_include_directories(Utils PUBLIC
    Inc
)

# 链接依赖库
target_link_libraries(Utils PUBLIC
    MCU  # 链接到MCU库以获取GD32F30x头文件和基本类型定义
)

# 编译选项
target_compile_options(Utils PRIVATE
    -Wall
    -Wextra
    -O2
)