#ifndef _HW_TYPES_BRIDGE_H_
#define _HW_TYPES_BRIDGE_H_

#include "transformation.h"  // 包含Park_t, Clark_t, Phase_t等定义

/* 数据类型桥接 - 使用typedef创建别名，保持语义清晰 */
typedef Phase_t HW_CurrentData_t;    // 三相电流数据：{a, b, c} 对应 {ia, ib, ic}
typedef Clark_t HW_AlphaBetaData_t;  // αβ轴数据：{alpha, beta}
typedef Park_t  HW_DQData_t;         // dq轴数据：{d, q}

/* 语义化的访问宏 - 让代码更容易理解 */
#define HW_CURRENT_A(curr_data)  ((curr_data)->a)  // A相电流
#define HW_CURRENT_B(curr_data)  ((curr_data)->b)  // B相电流  
#define HW_CURRENT_C(curr_data)  ((curr_data)->c)  // C相电流

#define HW_ALPHA(ab_data)        ((ab_data)->alpha)  // α轴分量
#define HW_BETA(ab_data)         ((ab_data)->beta)   // β轴分量

#define HW_D_AXIS(dq_data)       ((dq_data)->d)      // d轴分量
#define HW_Q_AXIS(dq_data)       ((dq_data)->q)      // q轴分量

/* 方便的初始化宏 */
#define HW_CURRENT_INIT(ia, ib, ic)  {.a = (ia), .b = (ib), .c = (ic)}
#define HW_ALPHA_BETA_INIT(a, b)     {.alpha = (a), .beta = (b)}
#define HW_DQ_INIT(d, q)             {.d = (d), .q = (q)}

#endif /* _HW_TYPES_BRIDGE_H_ */
