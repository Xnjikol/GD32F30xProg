/*
 * flying.h
 *
 * 初始位置与转速辨识头文件
 */
#ifndef __FLYING_H__
#define __FLYING_H__

#include <stdbool.h>
#include <stdint.h>

bool Flying_Set_Enabled(bool enabled);

void Flying_Update(bool reset);

bool Flying_Is_Completed(void);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* __FLYING_H__ */
