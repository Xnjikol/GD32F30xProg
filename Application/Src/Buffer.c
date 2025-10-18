#include "Buffer.h"
#include <string.h>  // for memset
#include "justfloat.h"

/* 固定大小的内部缓冲区（10 个 float） */
static float Buffer_Pool[BUFFER_MAX_CAPACITY];
/* 当前有效容量（<= 10） */
static size_t Buffer_Capacity  = BUFFER_DEFAULT_CAPACITY;
static size_t Buffer_Prescaler = 1U;

void Buffer_Init(size_t length, size_t prescalor)
{
    /* 限制长度不超过默认容量 */
    Buffer_Capacity = (length == 0U) ? BUFFER_DEFAULT_CAPACITY : length;
    Buffer_Prescaler = prescalor;
    if (Buffer_Capacity > BUFFER_MAX_CAPACITY)
    {
        Buffer_Capacity = BUFFER_MAX_CAPACITY;
    }

    /* 清空前 Buffer_Capacity 个元素，保持其余不变无影响 */
    memset(Buffer_Pool, 0, Buffer_Capacity * sizeof(float));
}

void Buffer_Put(float value, size_t index)
{
    if (index < Buffer_Capacity)
    {
        Buffer_Pool[index] = value;
    }
    /* 越界则忽略写入 */
}

void Buffer_Send(void)
{
    static uint16_t buffer_cnt = 0x0000U;
    buffer_cnt++;
    if (buffer_cnt < Buffer_Prescaler)
    {
        return;
    }
    justfloat(Buffer_Pool, Buffer_Capacity);
    buffer_cnt = 0x0000U;
}
