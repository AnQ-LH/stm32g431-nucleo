#ifndef __LOOP_H
#define __LOOP_H

#include "main.h"

// ��������ṹ��
typedef struct {
    void (*func)(void);     // ������ָ��
    uint32_t interval;      // ����ִ�м��ʱ��
    uint32_t last_executed; // �ϴ�ִ�������ʱ��
} task_t;

extern uint32_t timer;


void loop(void);
unsigned long currentTimeGet(void);
void taskAdd(void (*func)(void), uint32_t interval);
void taskInit(void);

#endif  /* __LOOP_H */
