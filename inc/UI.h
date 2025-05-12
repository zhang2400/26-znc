//
// Created by EiveLL on 25-5-9.
//

#ifndef UI_H
#define UI_H

#include "config.h"
#include "switch.h"
#include "counter.h"
#include "PID.h"
#include "flag.h"
#include "image_process.h"
#include "GPIO.h"
#include "tft180.h"

typedef signed char         int8;                                               // 有符号  8 bits
typedef signed short int    int16;                                              // 有符号 16 bits
typedef signed int          int32;                                              // 有符号 32 bits

typedef volatile int8       vint8;                                              // 易变性修饰 有符号  8 bits
typedef volatile int16      vint16;                                             // 易变性修饰 有符号 16 bits
typedef volatile int32      vint32;                                             // 易变性修饰 有符号 32 bits

typedef double (*UI_func_t)(void);  // 接收 void 返回 double

typedef enum {
    INT32 = 0,
    INT16 = 1,
    INT8 = 2,
    UINT32 = 3,
    UINT16 = 4,
    UINT8 = 5,
    DOUBLE = 6,
    FLOAT = 7,
    EMPTY = 8,
    FUNC = 9,
    CHAR = 10
} item_type;

typedef union {
    int32 *int32_p;
    int16 *int16_p;
    int8 *int8_p;
    uint32 *uint32_p;
    uint16 *uint16_p;
    uint8 *uint8_p;
    double *double_p;
    float *float_p;

    void *func_p;  // 支持显示空参数int函数的返回值
    char *char_p;
} UI_Var_p;

typedef struct {
    char name[7];
    uint8 type;
    UI_Var_p var_p;
} UI_item;

void UI_item_init(UI_item *item, const char *name, int type, void *var_ptr);

void UI_item_set_value(UI_item *item, double value);

double UI_item_get_value(UI_item *item);

void UI_item_show_name(UI_item *item, uint16 x, uint16 y);

void UI_item_show_value(UI_item *item, uint16 x, uint16 y);

void UI_init();

void UI_show();

void UI_show_custom_part();

void UI_key_process();


#endif //UI_H
