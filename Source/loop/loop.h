#ifndef __LOOP_H
#define __LOOP_H

#include "main.h"

// 定义任务结构体
typedef struct {
    void (*func)(void);     // 任务函数指针
    uint32_t interval;      // 任务执行间隔时间
    uint32_t last_executed; // 上次执行任务的时间
} task_t;

extern uint32_t timer;


void loop(void);
unsigned long currentTimeGet(void);
void taskAdd(void (*func)(void), uint32_t interval);
void taskInit(void);

#endif  /* __LOOP_H */
