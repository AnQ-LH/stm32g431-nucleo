#include "./loop/loop.h"

/* ����������кͼ�ʱ������ */
task_t task_queue[10];
uint8_t task_count = 0;
uint32_t timer = 0;


/**
 * @name   taskAdd
 * @brief  ��������������
 * @param  ��������ִ��ʱ��
 * @retval ��
 */
void taskAdd(void (*func)(void), uint32_t interval)
{
    if(task_count < 10)
    {
        task_queue[task_count].func = func;
        task_queue[task_count].interval = interval;
        task_queue[task_count].last_executed = 0;
        task_count++;
    }
}

/**
 * @name   loop
 * @brief  ���������������
 * @param  ��
 * @retval ��
 */
void loop(void)
{
    while(1)
    {
		/* ��ȡ��ǰʱ�� */
        uint32_t current_time = currentTimeGet();

		/* ����������У�����Ƿ���Ҫִ������ */
        for(int i = 0; i < task_count; i++)
        {
			/* ���������ʱ���ѹ�����ִ������ */
            if(current_time - task_queue[i].last_executed >= task_queue[i].interval)
            {
                task_queue[i].func();
                task_queue[i].last_executed = current_time;
            }
        } 
		/* ���¼�ʱ��,���ڶ�ʱ���� */
        //timer++;
    }
}

/**
 * @name   add_task
 * @brief  ��������������
 * @param  ��������ִ��ʱ��
 * @retval ��
 */
unsigned long currentTimeGet(void)
{
    return timer;
}

/**
 * @name   task1
 * @brief  ����1
 * @param  ��
 * @retval ��
 */
#define interval1 1
void task1(void)
{

}

/**
 * @name   task2
 * @brief  ����2
 * @param  ��
 * @retval ��
 */
#define interval2 1
void task2(void)
{

}

/**
 * @name   taskInit
 * @brief  �����ʼ��
 * @param  ��
 * @retval ��
 */
void taskInit(void)
{
	taskAdd(task1, interval1);
	taskAdd(task2, interval2);
}


