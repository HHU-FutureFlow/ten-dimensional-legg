#ifndef DEBUG_MODE_H
#define DEBUG_MODE_H

/* 与普通应用互斥。Init 在调度器启动前调用，Task 由底盘周期任务调用。 */
void DebugModeInit(void);
void DebugModeTask(void);

#endif
