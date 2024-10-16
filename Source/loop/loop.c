#include "./loop/loop.h"

/* 定义任务队列和计时器变量 */
task_t task_queue[10];
uint8_t task_count = 0;
uint32_t timer = 0;


/**
 * @name   taskAdd
 * @brief  添加任务到任务队列
 * @param  任务函数；执行时间
 * @retval 无
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
 * @brief  任务调度器主函数
 * @param  无
 * @retval 无
 */
void loop(void)
{
    while(1)
    {
		/* 获取当前时间 */
        uint32_t current_time = currentTimeGet();

		/* 遍历任务队列，检查是否需要执行任务 */
        for(int i = 0; i < task_count; i++)
        {
			/* 如果任务间隔时间已过，则执行任务 */
            if(current_time - task_queue[i].last_executed >= task_queue[i].interval)
            {
                task_queue[i].func();
                task_queue[i].last_executed = current_time;
            }
        } 
		/* 更新计时器,放在定时器里 */
        //timer++;
    }
}

/**
 * @name   add_task
 * @brief  添加任务到任务队列
 * @param  任务函数；执行时间
 * @retval 无
 */
unsigned long currentTimeGet(void)
{
    return timer;
}

/**
 * @name   task1
 * @brief  任务1
 * @param  无
 * @retval 无
 */
#define interval1 1
void task1(void)
{

}

/**
 * @name   task2
 * @brief  任务2
 * @param  无
 * @retval 无
 */
#define interval2 1
void task2(void)
{

}

/**
 * @name   taskInit
 * @brief  任务初始化
 * @param  无
 * @retval 无
 */
void taskInit(void)
{
	taskAdd(task1, interval1);
	taskAdd(task2, interval2);
}


