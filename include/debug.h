#pragma once
#include <cstdio>

#define STATE(level, name, content) std::cout << "--- [" << level << "][" << name << "] " << content << std::endl;

/* 绿色 */
#define PRINT_INFO(content, ...) printf("\033[32m" content "\033[0m", ##__VA_ARGS__)
/* 黄色 */
#define PRINT_WARN(content, ...) printf("\033[33m" content "\033[0m", ##__VA_ARGS__)
/* 红色 */
#define PRINT_ERROR(content, ...) printf("\033[31m" content "\033[0m", ##__VA_ARGS__)
