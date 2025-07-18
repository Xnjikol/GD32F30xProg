/**
 * @file main_example_twelve_pulse.h
 * @brief 十二脉冲主程序示例头文件
 * @note 这是一个示例文件，不包含在主程序编译中
 */

#ifndef _MAIN_EXAMPLE_TWELVE_PULSE_H_
#define _MAIN_EXAMPLE_TWELVE_PULSE_H_

#include "../../../Core/Inc/main.h"

/*======================*/
/*    Function Protos   */
/*======================*/

/**
 * @brief 主函数示例 - 集成十二脉冲功能
 * @return 程序退出码
 */
int main_twelve_pulse_example(void);

/**
 * @brief 简化的十二脉冲测试函数
 */
void simple_twelve_pulse_test(void);

/**
 * @brief 交互式十二脉冲测试
 */
void interactive_twelve_pulse_test(void);

/**
 * @brief 电机启动序列 - 集成十二脉冲
 * @return 0-成功，非0-失败
 */
int motor_startup_with_twelve_pulse(void);

#endif /* _MAIN_EXAMPLE_TWELVE_PULSE_H_ */
