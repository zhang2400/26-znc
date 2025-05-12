//
// Created by EiveLL on 25-5-11.
//

#ifndef TFT180_H
#define TFT180_H

#include "typedef.h"
#include "font.h"
#include "zf_common_function.h"

#define TFT180_DEFAULT_PENCOLOR         (RGB565_CYAN    )                        // 默认的画笔颜色
#define TFT180_DEFAULT_BGCOLOR          (RGB565_BLACK  )                        // 默认的背景颜色


void    tft180_clear            (void);
void    tft180_full             (const uint16 color);
void    tft180_draw_point       (uint16 x, uint16 y, const uint16 color);
void    tft180_set_color        (uint16 pen, const uint16 bgcolor);
void    tft180_draw_line        (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);

void    tft180_show_char        (uint16 x, uint16 y, const char dat);
void    tft180_show_string      (uint16 x, uint16 y, const char dat[]);
void    tft180_show_int         (uint16 x, uint16 y, const int32 dat, uint8 num);
void    tft180_show_uint        (uint16 x, uint16 y, const uint32 dat, uint8 num);
void    tft180_show_float       (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);

void    tft180_show_gray_image  (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height);

void    tft180_init             (const char *path);

#endif //TFT180_H
