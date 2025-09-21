#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

#include "stdio.h"

// ������־�ȼ�

#define LOG_LEVEL_NONE 0    // �ر�������־
#define LOG_LEVEL_FATAL 1   // Fatal ����
#define LOG_LEVEL_ERROR 2   // Error ����
#define LOG_LEVEL_WARNING 3 // Warning ����
#define LOG_LEVEL_INFO 4    // Info ����
#define LOG_LEVEL_TRACE 5   // Trace ���� (���)

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

// ��־������������Ը�����Ҫ�޸�Ϊ���������
#define DEBUG_PRINT(tag, level_str, format, ...) \
    printf("[%s][%s][%s] " format "\n", level_str, tag, __FILE__, ##__VA_ARGS__)

// Fatal ������־
#if LOG_LEVEL >= LOG_LEVEL_FATAL
#define DEBUG_LOGF(tag, format, ...) DEBUG_PRINT(tag, "F", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGF(tag, format, ...)
#endif

// Error ������־
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define DEBUG_LOGE(tag, format, ...) DEBUG_PRINT(tag, "E", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGE(tag, format, ...)
#endif

// Warning ������־
#if LOG_LEVEL >= LOG_LEVEL_WARNING
#define DEBUG_LOGW(tag, format, ...) DEBUG_PRINT(tag, "W", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGW(tag, format, ...)
#endif

// Info ������־
#if LOG_LEVEL >= LOG_LEVEL_INFO
#define DEBUG_LOGI(tag, format, ...) DEBUG_PRINT(tag, "I", format, ##__VA_ARGS__)
#else
#define DEBUG_LOGI(...)
#endif

// Trace ������־
#if LOG_LEVEL >= LOG_LEVEL_TRACE
#define DEBUG_LOGT(...) printf(__VA_ARGS__)
#else
#define DEBUG_LOGT(...)
#endif

#endif /* __DEBUG_LOG_H__ */
