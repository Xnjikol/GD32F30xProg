/*********************************************************************/
/*                        初始化参数头文件                              */
/*                                                                   */
/* 文件名: Initialization.h                                           */
/* 描述: 包含电机控制所需的所有初始化函数                                  */
/*********************************************************************/

#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <stdbool.h>

/*********************************************************************/
/*                        函数声明                                    */
/*********************************************************************/
/**
 * @brief  初始化所有控制变量
 * @return 初始化是否成功
 * @retval true  - 初始化成功
 * @retval false - 初始化失败
 */
bool Initialization_Variables(void);

/**
 * @brief  执行所有初始化过程
 * @return 初始化是否成功
 * @retval true  - 初始化成功
 * @retval false - 初始化失败
 */
bool Initialization_Execute(void);

#endif /* INITIALIZATION_H */
