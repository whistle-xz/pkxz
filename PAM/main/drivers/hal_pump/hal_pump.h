#ifndef HAL_PUMP_H
#define HAL_PUMP_H

// 初始化气泵和压力开关 GPIO
void pump_init(void);

// 气泵保压逻辑任务 (放在 FreeRTOS 任务中运行)
// 它会自动检测 QPM11 状态并启停气泵
void pump_control_loop(void);

#endif
