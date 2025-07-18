/**
 * @file twelve_pulse_example.h
 * @brief 十二脉冲初始定位示例头文件
 * @author your_name
 * @date 2024-12-19
 * @note 这是一个独立的示例文件，不包含在主程序编译中
 */

#ifndef _TWELVE_PULSE_EXAMPLE_H_
#define _TWELVE_PULSE_EXAMPLE_H_

#include "../../../Core/Inc/main.h"

/*======================*/
/*    Function Protos   */
/*======================*/

/**
 * @brief 十二脉冲初始定位示例
 * @return 0-成功，非0-失败
 */
int TwelvePulse_Example(void);

/**
 * @brief 十二脉冲调试信息输出
 */
void TwelvePulse_PrintDebugInfo(void);

/**
 * @brief 十二脉冲参数配置
 */
void TwelvePulse_ConfigureParameters(void);

/**
 * @brief 十二脉冲自动测试序列
 * @return 0-成功，非0-失败
 */
int TwelvePulse_AutoTest(void);

/**
 * @brief 十二脉冲结果验证
 */
void TwelvePulse_ValidateResults(void);

#endif /* _TWELVE_PULSE_EXAMPLE_H_ */
