//
// Created by EiveLL on 25-5-11.
//

#include "tft180.h"

static uint16 tft180_pencolor = TFT180_DEFAULT_PENCOLOR;
static uint16 tft180_bgcolor = TFT180_DEFAULT_BGCOLOR;

static int tft180_width;
static int tft180_height;
unsigned short *screen_base = nullptr;         //映射后的显存基地址

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     tft180 清屏函数
// 参数说明     void
// 返回参数     void
// 使用示例     tft180_clear();
// 备注信息     将屏幕清空成背景颜色
//-------------------------------------------------------------------------------------------------------------------
void tft180_clear()
{
    tft180_full(TFT180_DEFAULT_BGCOLOR);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     tft180 屏幕填充函数
// 参数说明     color           颜色格式 RGB565 或者可以使用 zf_common_font.h 内 rgb565_color_enum 枚举值或者自行写入
// 返回参数     void
// 使用示例     tft180_full(RGB565_BLACK);
// 备注信息     将屏幕填充成指定颜色
//-------------------------------------------------------------------------------------------------------------------
void tft180_full(const uint16 color)
{
    for(uint16 i = 0;i < tft180_width; i++)
    {
        for(uint16 j = 0; j<tft180_height; j++)
        {
            tft180_draw_point(i, j, color);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TFT180 画点
// 参数说明     x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明     dat             需要显示的颜色
// 返回参数     void
// 使用示例     tft180_draw_point(0, 0, RGB565_RED);            // 坐标 0,0 画一个红色的点
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void tft180_draw_point(uint16_t x, uint16_t y, const uint16_t color)
{
    screen_base[y * tft180_width + x] = color;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示颜色
// 参数说明     pen             颜色格式 RGB565 或者可以使用 zf_common_font.h 内常用颜色宏定义
// 参数说明     bgcolor         颜色格式 RGB565 或者可以使用 zf_common_font.h 内常用颜色宏定义
// 返回参数     void
// 使用示例     tft180_set_color(RGB565_WHITE, RGB565_BLACK);
// 备注信息     字体颜色和背景颜色也可以随时自由设置 设置后生效
//-------------------------------------------------------------------------------------------------------------------
void tft180_set_color (uint16 pen, const uint16 bgcolor)
{
    tft180_pencolor = pen;
    tft180_bgcolor = bgcolor;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     tft180 画线
// 参数说明     x_start         坐标x方向的起点 [0, tft180_x_max-1]
// 参数说明     y_start         坐标y方向的起点 [0, tft180_y_max-1]
// 参数说明     x_end           坐标x方向的终点 [0, tft180_x_max-1]
// 参数说明     y_end           坐标y方向的终点 [0, tft180_y_max-1]
// 参数说明     color           颜色格式 RGB565 或者可以使用 zf_common_font.h 内 rgb565_color_enum 枚举值或者自行写入
// 返回参数     void
// 使用示例     tft180_draw_line(0, 0, 10, 10, RGB565_RED);     // 坐标 0,0 到 10,10 画一条红色的线
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void tft180_draw_line (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
    int16 x_dir = x_start < x_end ? 1 : -1;
    int16 y_dir = y_start < y_end ? 1 : -1;
    float temp_rate = 0;
    float temp_b = 0;

    do
    {
        if(x_start != x_end)
        {
            temp_rate = (float)(y_start - y_end) / (float)(x_start - x_end);
            temp_b = (float)y_start - (float)x_start * temp_rate;
        }
        else
        {
            while(y_start != y_end)
            {
                tft180_draw_point(x_start, y_start, color);
                y_start += y_dir;
            }
            tft180_draw_point(x_start, y_start, color);
            break;
        }
        if(func_abs(y_start - y_end) > func_abs(x_start - x_end))
        {
            while(y_start != y_end)
            {
                tft180_draw_point(x_start, y_start, color);
                y_start += y_dir;
                x_start = (int16)(((float)y_start - temp_b) / temp_rate);
            }
            tft180_draw_point(x_start, y_start, color);
        }
        else
        {
            while(x_start != x_end)
            {
                tft180_draw_point(x_start, y_start, color);
                x_start += x_dir;
                y_start = (int16)((float)x_start * temp_rate + temp_b);
            }
            tft180_draw_point(x_start, y_start, color);
        }
    }while(false);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TFT180 显示字符
// 参数说明     x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明     dat             需要显示的字符
// 返回参数     void
// 使用示例     tft180_show_char(0, 0, 'x');                    // 坐标 0,0 写一个字符 x
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_char(uint16 x, uint16 y, const char dat)
{
    uint8 i = 0, j = 0;
    for(i = 0; 8 > i; i ++)
    {
        uint8 temp_top = ascii_font_8x16[dat - 32][i];
        uint8 temp_bottom = ascii_font_8x16[dat - 32][i + 8];
        for(j = 0; 8 > j; j ++)
        {
            if(temp_top & 0x01)
            {
                tft180_draw_point(x + i, y + j, tft180_pencolor);
            }
            else
            {
                tft180_draw_point(x + i, y + j, tft180_bgcolor);
            }
            temp_top >>= 1;
        }

        for(j = 0; 8 > j; j ++)
        {
            if(temp_bottom & 0x01)
            {
                tft180_draw_point(x + i, y + j + 8, tft180_pencolor);
            }
            else
            {
                tft180_draw_point(x + i, y + j + 8, tft180_bgcolor);
            }
            temp_bottom >>= 1;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TFT180 显示字符串
// 参数说明     x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明     dat             需要显示的字符串
// 返回参数     void
// 使用示例     tft180_show_string(0, 0, "seekfree");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_string(uint16 x, uint16 y, const char dat[])
{
    uint16 j = 0;

    while('\0' != dat[j])
    {
        tft180_show_char(x + 8 * j,  y, dat[j]);
        j++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     TFT180 显示32位有符号 (去除整数部分无效的0)
// 参数说明     x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明     dat             需要显示的变量 数据类型 int32
// 参数说明     num             需要显示的位数 最高10位  不包含正负号
// 返回参数     void
// 使用示例     tft180_show_int(0, 0, x, 3);                    // x 可以为 int32 int16 int8 类型
// 备注信息     负数会显示一个 ‘-’号   正数显示一个空格
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_int(uint16 x, uint16 y, const int32 dat, uint8 num)
{
    int32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num+1);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_int_to_str(data_buffer, dat_temp);
    tft180_show_string(x, y, (const char *)&data_buffer);
}

void tft180_show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num)
{

    uint32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12];
    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_uint_to_str(data_buffer, dat_temp);
    tft180_show_string(x, y, (const char *)&data_buffer);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     tft180 显示浮点数(去除整数部分无效的0)
// 参数说明     x               坐标x方向的起点 参数范围 [0, tft180_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, tft180_y_max-1]
// 参数说明     dat             需要显示的变量 数据类型 double
// 参数说明     num             整数位显示长度   最高8位
// 参数说明     pointnum        小数位显示长度   最高6位
// 返回参数     void
// 使用示例     tft180_show_float(0, 0, x, 2, 3);               // 显示浮点数   整数显示2位   小数显示三位
// 备注信息     特别注意当发现小数部分显示的值与你写入的值不一样的时候，
//              可能是由于浮点数精度丢失问题导致的，这并不是显示函数的问题，
//              有关问题的详情，请自行百度学习   浮点数精度丢失问题。
//              负数会显示一个 ‘-’号
//-------------------------------------------------------------------------------------------------------------------
void tft180_show_float (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum)
{
    // // 如果程序在输出了断言信息 并且提示出错位置在这里
    // // 那么一般是屏幕显示的时候超过屏幕分辨率范围了
    // zf_assert(x < tft180_x_max);
    // zf_assert(y < tft180_y_max);
    // zf_assert(0 < num);
    // zf_assert(8 >= num);
    // zf_assert(0 < pointnum);
    // zf_assert(6 >= pointnum);

    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17];
    memset(data_buffer, 0, 17);
    memset(data_buffer, ' ', num+pointnum+2);

    // 用来计算余数显示 123 显示 2 位则应该显示 23
    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    tft180_show_string(x, y, data_buffer);
}


void tft180_show_gray_image(uint16 x, uint16 y, const uint8 *image,
uint16 width, uint16 height)
{
    uint32 x_start = 0, y_start = 0;
    uint16 color = 0;

    for(y_start = y; y_start < (y + height); y_start++)
    {
        for(x_start = x; x_start < (x + width); x_start++)
        {
            uint8_t grayValue = image[(x_start - x) + (y_start- y) * width];

            uint16_t r = (grayValue >> 3) & 0b11111;
            uint16_t g = (grayValue >> 2) & 0b111111;
            uint16_t b = (grayValue >> 3) & 0b11111;
            color = (r << 11) | (g << 5) | (b << 0);

            tft180_draw_point(x_start, y_start,  color);
        }
    }
}

void tft180_init(const char *path)
{
    struct fb_fix_screeninfo fb_fix{};
    struct fb_var_screeninfo fb_var{};
    unsigned int screen_size;
    int fd;

    if (0 > (fd = open(path, O_RDWR))) {
        perror("open error");
        exit(EXIT_FAILURE);
    }

    if (0 > (fd = open(path, O_RDWR))) {
        perror("open error");
        exit(EXIT_FAILURE);
    }

    /* 获取参数信息 */
    ioctl(fd, FBIOGET_VSCREENINFO, &fb_var);
    ioctl(fd, FBIOGET_FSCREENINFO, &fb_fix);

    screen_size = fb_fix.line_length * fb_var.yres;
    tft180_width = fb_var.xres;
    tft180_height = fb_var.yres;

    printf("tft180_width = %d, tft180_height = %d\n", tft180_width, tft180_height);

    /* 将显示缓冲区映射到进程地址空间 */
    screen_base = (unsigned short *)mmap(nullptr, screen_size, PROT_WRITE, MAP_SHARED, fd, 0);
    if (MAP_FAILED == (void *)screen_base) {
        perror("mmap error");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // 刷屏全黑
    for(uint16 i=0;i<tft180_width;i++)
    {
        for(uint16 j=0;j<tft180_height;j++)
        {
            tft180_draw_point(i, j, TFT180_DEFAULT_BGCOLOR);
        }
    }
    printf("tft180_init success!\n");
}