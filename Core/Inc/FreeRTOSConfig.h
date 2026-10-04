#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif

/* --- CẤU HÌNH HỆ THỐNG CƠ BẢN --- */
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000) /* 1ms per tick */
#define configMAX_PRIORITIES (7)
#define configMINIMAL_STACK_SIZE ((unsigned short)128)
#define configTOTAL_HEAP_SIZE ((size_t)(13 * 1024)) /* 13KB cho RAM 20KB của F103 */
#define configMAX_TASK_NAME_LEN (16)
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_MUTEXES 1
#define configQUEUE_REGISTRY_SIZE 8
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_MALLOC_FAILED_HOOK 0
#define configUSE_APPLICATION_TASK_TAG 0
#define configUSE_COUNTING_SEMAPHORES 1

/* --- QUẢN LÝ BỘ NHỚ --- */
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/* --- CẤU HÌNH TÍNH NĂNG ĐỒNG BỘ/HOOKS --- */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (2)
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)

/* --- CHỌN CÁC HÀM API ĐƯỢC PHÉP SỬ DỤNG (INCLUDE) --- */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskCleanUpResources 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1

/* Bật tính năng theo dõi và thu thập dữ liệu Task */
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Bật hàm kiểm tra Stack dư (Stack High Water Mark) */
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/* --- CẤU HÌNH LỒNG NGẮT (INTERRUPT NESTING) CHO CORTEX-M3 --- */
#ifdef __NVIC_PRIORITY_BITS
#define configPRIORITY_BITS __NVIC_PRIORITY_BITS
#else
#define configPRIORITY_BITS 4 /* STM32F103 có 4 bit ưu tiên */
#endif

/* Mức ưu tiên ngắt thấp nhất (Thường gán cho KERNEL và các tác vụ nền) */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

/* Mức ưu tiên cao nhất mà FreeRTOS quản lý (Từ 5 -> 15 gọi được hàm API) */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Định dạng cấu hình ngắt cho phần cứng phần lõi của Cortex-M3 */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIORITY_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIORITY_BITS))

/* --- LIÊN KẾT CÁC VECTOR NGẮT HỆ THỐNG VỚI FREE RTOS --- */
/* Áp dụng định nghĩa lại để tránh xung đột với các trình phục vụ ngắt của thư viện HAL */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define configASSERT(x)           \
    if ((x) == 0)                 \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for (;;)                  \
            ;                     \
    }

#endif /* FREERTOS_CONFIG_H */
