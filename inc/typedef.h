//
// Created by EiveLL on 25-5-11.
//

#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <cstdint>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <fcntl.h>

#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/mman.h>

#include "stdio.h"
#include <cstdint>
#include "stdbool.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"

typedef float float32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

// 尽量使用 stdint.h 定义的类型名称 避免冲突 这里可以裁剪
typedef unsigned char       uint8;                                              // 无符号  8 bits
typedef unsigned short int  uint16;                                             // 无符号 16 bits
typedef unsigned int        uint32;                                             // 无符号 32 bits
// typedef unsigned long long  uint64;                                             // 无符号 64 bits

typedef signed char         int8;                                               // 有符号  8 bits
typedef signed short int    int16;                                              // 有符号 16 bits
typedef signed int          int32;                                              // 有符号 32 bits
// typedef signed long long    int64;                                              // 有符号 64 bits

typedef volatile uint8      vuint8;                                             // 易变性修饰 无符号  8 bits
typedef volatile uint16     vuint16;                                            // 易变性修饰 无符号 16 bits
typedef volatile uint32     vuint32;                                            // 易变性修饰 无符号 32 bits
// typedef volatile uint64     vuint64;                                            // 易变性修饰 无符号 64 bits

typedef volatile int8       vint8;                                              // 易变性修饰 有符号  8 bits
typedef volatile int16      vint16;                                             // 易变性修饰 有符号 16 bits
typedef volatile int32      vint32;                                             // 易变性修饰 有符号 32 bits
// typedef volatile int64      vint64;                                             // 易变性修饰 有符号 64 bits

#endif //TYPEDEF_H
