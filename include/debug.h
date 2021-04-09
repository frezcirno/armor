#pragma once
#include <cstdio>

#define DEBUG_MODE  // 使能 DEBUG 宏

#ifdef DEBUG_MODE

/* 文件名获取 */
#    define __FILENAME__ (strrchr(__FILE__, '/') + 1)

#    define DEBUG(X) \
        {            \
        }
#else
#    define DEBUG(content)
#endif

#define STATE(level, name, content) cout << "--- [" << level << "][" << name << "] " << content << endl;
#define ERROR "error"
#define INFOO "infoo"
#define WARN "warn"

/* 绿色 */
#define PRINT_INFO(content, ...) printf("\033[32m" content "\033[0m", ##__VA_ARGS__)
/* 黄色 */
#define PRINT_WARN(content, ...) printf("\033[33m" content "\033[0m", ##__VA_ARGS__)
/* 红色 */
#define PRINT_ERROR(content, ...) printf("\033[31m" content "\033[0m", ##__VA_ARGS__)