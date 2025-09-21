#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

#include "stdio.h"

// 定义日志等级

#define LOG_LEVEL_NONE 0    // 关闭所有日志
#define LOG_LEVEL_FATAL 1   // Fatal 级别
#define LOG_LEVEL_ERROR 2   // Error 级别
#define LOG_LEVEL_WARNING 3 // Warning 级别
#define LOG_LEVEL_INFO 4    // Info 级别
#define LOG_LEVEL_TRACE 5   // Trace 级别 (最高)

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

// 日志输出函数（可以根据需要修改为串口输出）
#define DEBUG_PRINT(tag, level_str, format, ...) \
    printf("[%s][%s][%s] " format "\n", level_str, tag, __FILE__, ##__VA_ARGS__)

// Fatal 级别日志
#if LOG_LEVEL >= LOG_LEVEL_FATAL
#define DEBUG_LOGF(tag, format, ...) DEBUG_PRINT(tag, "F", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGF(tag, format, ...)
#endif

// Error 级别日志
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define DEBUG_LOGE(tag, format, ...) DEBUG_PRINT(tag, "E", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGE(tag, format, ...)
#endif

// Warning 级别日志
#if LOG_LEVEL >= LOG_LEVEL_WARNING
#define DEBUG_LOGW(tag, format, ...) DEBUG_PRINT(tag, "W", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGW(tag, format, ...)
#endif

// Info 级别日志
#if LOG_LEVEL >= LOG_LEVEL_INFO
#define DEBUG_LOGI(tag, format, ...) DEBUG_PRINT(tag, "I", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGI(...)
#endif

// Trace 级别日志
#if LOG_LEVEL >= LOG_LEVEL_TRACE
#define DEBUG_LOGT(...) printf(__VA_ARGS__)
#else
#define DEBUG_LOGT(...)
#endif

#endif /* __DEBUG_LOG_H__ */
