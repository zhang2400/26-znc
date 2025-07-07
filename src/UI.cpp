//
// Created by EiveLL on 25-5-9.
//

#include "UI.h"

#define FONT_WEIGHT 8
#define FONT_HEIGHT 16

char buf[32];
UI_item items[8][8];
int8 empty = 0;
int8 cursor_pos = 0;
int8 exponent = 0;
int8 ui_state = 0;  // 0:移动光标 1:修改数值 2:修改数值的指数
int8 key_pressed = 0;
int perspective = 0;  // 0:原图 1:透视图

void UI_item_init(UI_item *item, const char *name, int type, void *var_ptr) {
    strcpy(item->name, name);
    item->type = type;
    switch (item->type) {
        case INT32:
            item->var_p.int32_p = (int32 *) var_ptr;
            break;
        case INT16:
            item->var_p.int16_p = (int16 *) var_ptr;
            break;
        case INT8:
            item->var_p.int8_p = (int8 *) var_ptr;
            break;
        case UINT32:
            item->var_p.uint32_p = (uint32 *) var_ptr;
            break;
        case UINT16:
            item->var_p.uint16_p = (uint16 *) var_ptr;
            break;
        case UINT8:
            item->var_p.uint8_p = (uint8 *) var_ptr;
            break;
        case DOUBLE:
            item->var_p.double_p = (double *) var_ptr;
            break;
        case FLOAT:
            item->var_p.float_p = (float *) var_ptr;
            break;
        case EMPTY:
            item->var_p.int8_p = (int8 *) var_ptr;
            break;
        case FUNC:
            item->var_p.func_p = var_ptr;
            break;
        case CHAR:
            item->var_p.char_p = (char *) var_ptr;
            break;
        default: ;
    }
}

double UI_item_get_value(UI_item *item) {
    switch (item->type) {
        case INT32:
            return (double) *item->var_p.int32_p;
        case INT16:
            return (double) *item->var_p.int16_p;
        case INT8:
            return (double) *item->var_p.int8_p;
        case UINT32:
            return (double) *item->var_p.uint32_p;
        case UINT16:
            return (double) *item->var_p.uint16_p;
        case UINT8:
            return (double) *item->var_p.uint8_p;
        case DOUBLE:
            return *item->var_p.double_p;
        case FLOAT:
            return (double) *item->var_p.float_p;
        case FUNC: {
            auto func = (UI_func_t)(item->var_p.func_p);
            return func();  // 转换后调用
        }
        case CHAR:
            return (double) *item->var_p.char_p;
        default: ;
    }
    return 0;
}

void UI_item_set_value(UI_item *item, double value) {
    if(item->type == EMPTY) return;
    switch (item->type) {
        case INT32:
            *item->var_p.int32_p = (int32) value;
            break;
        case INT16:
            *item->var_p.int16_p = (int16) value;
            break;
        case INT8:
            *item->var_p.int8_p = (int8) value;
            break;
        case UINT32:
            *item->var_p.uint32_p = (uint32) value;
            break;
        case UINT16:
            *item->var_p.uint16_p = (uint16) value;
            break;
        case UINT8:
            *item->var_p.uint8_p = (uint8) value;
            break;
        case DOUBLE:
            *item->var_p.double_p = value;
            break;
        case FLOAT:
            *item->var_p.float_p = (float) value;
            break;
        case CHAR:
            *item->var_p.char_p = (char) value;
            break;
        default: ;
    }
}

void UI_item_show_name(UI_item *item, uint16 x, uint16 y) {
    if(item->type == EMPTY) {
        tft180_show_string(x, y, "               ");
    }
    tft180_set_color (RGB565_YELLOW, RGB565_BLACK);
    tft180_show_string(x, y, item->name);
}

void UI_item_show_value(UI_item *item, uint16 x, uint16 y) {
    if(item->type == EMPTY) return;
    double value = UI_item_get_value(item);
    tft180_set_color (RGB565_CYAN, RGB565_BLACK);
    switch (item->type) {
        case INT32:
        case INT16:
        case INT8:
        case UINT32:
        case UINT16:
        case UINT8:
        case FUNC:
            if(value > 9999999999 || value < -999999999){
                sprintf(buf, " Out range");
            } else {
                sprintf(buf, "%10.0f", value);
            }
            break;
        case DOUBLE:
        case FLOAT:
            if(value > 9999999.99 || value < -999999.99){
                sprintf(buf, " Out range");
            } else {
                sprintf(buf, "%10.3f", UI_item_get_value(item));
            }
            break;
        case CHAR:
            sprintf(buf, "%10c", (char)value);
            break;
        default: ;
    }
    tft180_show_string(x, y, buf);
}

void UI_init(){
    // 初始化所有的item为EMPTY
    for(auto & item : items){
        for(auto & j : item){
            j.type = EMPTY;
        }
    }
    UI_item_init(&items[0][0], "TKp  ", FLOAT, &Kp_max);
    UI_item_init(&items[0][1], "TKd  ", FLOAT, &Kd_max);
    UI_item_init(&items[0][2], "Aconv", FLOAT, &CAR_ANGLE_CONVERT);
    UI_item_init(&items[0][3], "spd_L", INT32, &Moto_L.speed);
    UI_item_init(&items[0][4], "spd_R", INT32, &Moto_R.speed);
    UI_item_init(&items[0][5], "start", INT8, &flag.start);
    UI_item_init(&items[0][6], "stop ", INT8, &flag.stop);
    // UI_item_init(&items[0][4], "Smid ", FLOAT, &servo_motor_mid);
    // UI_item_init(&items[0][5], "SLeft", FLOAT, &servo_motor_l_max);
    // UI_item_init(&items[0][6], "SRigh", FLOAT, &servo_motor_r_max);

    UI_item_init(&items[1][0], "TKp  ", FLOAT, &Kp_max);
    UI_item_init(&items[1][1], "TKd  ", FLOAT, &Kd_max);
    UI_item_init(&items[1][2], "AngZ ", FLOAT, &icm20948_data.anglez);
    UI_item_init(&items[1][3], "start", INT8, &flag.start);
    UI_item_init(&items[1][4], "stop ", INT8, &flag.stop);
    UI_item_init(&items[1][5], "incis", INT32, &incision);
    UI_item_init(&items[1][6], "speed_L", INT32, &Moto_L.speed);
    UI_item_init(&items[1][7], "speed_R", INT32, &Moto_R.speed);

    UI_item_init(&items[4][0], "diff", INT32, &image_diff);
    UI_item_init(&items[4][1], "SPD_R", INT32, &Moto_R.speed);
    UI_item_init(&items[4][2], "SPD_BA", FLOAT, &speed_base);
    UI_item_init(&items[4][3], "SPD_SET", FLOAT, &speed_setpoint);
    UI_item_init(&items[4][4], "LSP_SP", FLOAT, &left_speed_setpoint);
    UI_item_init(&items[4][5], "RSP_SP", FLOAT, &right_speed_setpoint);
    UI_item_init(&items[4][6], "LSP_O", FLOAT, &left_wheel_pidout);
    UI_item_init(&items[4][7], "RSP_O", FLOAT, &right_wheel_pidout);

    UI_item_init(&items[6][0], "SPD_L", INT32, &Moto_L.speed);
    UI_item_init(&items[6][1], "SPD_R", INT32, &Moto_R.speed);
    UI_item_init(&items[6][2], "SPD_B", FLOAT, &speed_base);
    UI_item_init(&items[6][3], "SPD_S", FLOAT, &speed_setpoint);
    UI_item_init(&items[6][4], "START", INT8, &flag.start);
    UI_item_init(&items[6][5], "STOP", INT8, &flag.stop);
    UI_item_init(&items[6][6], "LSP_O", FLOAT, &left_wheel_pidout);
    UI_item_init(&items[6][7], "RSP_O", FLOAT, &right_wheel_pidout);


    UI_item_init(&items[5][0], "TKp  ", FLOAT, &Kp_max);
    UI_item_init(&items[5][1], "TKd  ", FLOAT, &Kd_max);
}

void UI_show(){
    static uint8 last_dip_switch = 8;
    static uint8 last_ui_state = 3;
    uint8 dip_switch = DIP_SWITCH;
    uint8 show_static_part = (dip_switch != last_dip_switch) || (ui_state != last_ui_state) || key_pressed;

    // 显示静态部分
    if(show_static_part) {
        last_dip_switch = dip_switch;
        last_ui_state = ui_state;
        key_pressed = 0;

        // 顶部
        tft180_set_color(RGB565_GREEN, RGB565_BLACK);
        sprintf(buf, "Page%d       10%+d", dip_switch, exponent);
        tft180_show_string(0, 0, buf);
        for (uint8 i = 0; i < 8; i++) {
            tft180_show_char(0, FONT_HEIGHT * i + FONT_HEIGHT, '|');
        }

        // 显示光标
        switch (ui_state) {
            case 0:
                tft180_show_char(0, FONT_HEIGHT * cursor_pos + FONT_HEIGHT, '>');
            break;
            case 1:
                tft180_show_char(0, FONT_HEIGHT * cursor_pos + FONT_HEIGHT, '*');
            break;
            case 2:
                tft180_show_char(0, FONT_HEIGHT * cursor_pos + FONT_HEIGHT, '^');
            tft180_set_color(RGB565_PINK, RGB565_BLACK);
            sprintf(buf, "%+d", exponent);
            tft180_show_string(112, 0, buf);
            break;
            default: ;
        }

        // 显示名字
        for(uint8 i = 0; i < 8; i++){
            UI_item_show_name(&items[dip_switch][i], FONT_WEIGHT, FONT_HEIGHT * i + FONT_HEIGHT);
        }
    }

    // 显示值
     for(uint8 i = 0; i < 8; i++){
         UI_item_show_value(&items[dip_switch][i], FONT_WEIGHT * 6, FONT_HEIGHT * i + FONT_HEIGHT);
     }

    // 显示自定义部分
    // UI_show_custom_part();

    // 陀螺仪异常刷新屏幕
    if (flag.icm20948_error) {
        tft180_clear();
        flag.icm20948_error  = false;
    }

}

void UI_show_custom_part(){
    // if(counter.save_flash_led > 0){
    //     tft180_set_color(RGB565_GREEN, RGB565_BLACK);
    //     tft180_show_string(48, 0, "Saved");
    // }
    // if(counter.read_flash_led > 0){
    //     tft180_set_color(RGB565_GREEN, RGB565_BLACK);
    //     tft180_show_string(44, 0, "Readed");
    // }
    // switch (DIP_SWITCH){
    //     case 4:
    //         if(perspective){
    //             tft180_show_binary_image(16, 16, (const uint8 *) binary_pers_image, 47, 45);
    //             tft180_draw_border_line(16, 16, left_border_pers, RGB565_RED);
    //             tft180_draw_border_line(16, 16, right_border_pers, RGB565_RED);
    //             tft180_draw_border_line(16, 16, middle_line_single_pers, RGB565_BLUE);
    //         } else {
    //             tft180_show_binary_image(16, 16, (const uint8 *) binary_image, 94, 45);
    //             tft180_draw_border_line(16, 16, left_border, RGB565_RED);
    //             tft180_draw_border_line(16, 16, right_border, RGB565_RED);
    //             tft180_draw_border_line(16, 16, middle_line_single, RGB565_BLUE);
    //         }
    //         break;
    //     case 5:
    //         if(perspective) {
    //             tft180_show_gray_image(16, 16, (const uint8 *) gray_pers_image, 47, 45, 47, 45);
    //             tft180_draw_border_line(16, 16, left_distance_line_pers, RGB565_YELLOW);
    //             tft180_draw_border_line(16, 16, right_distance_line_pers, RGB565_YELLOW);
    //             tft180_draw_border_line(16, 16, distance_middle_line_pers, RGB565_GREEN);
    //         } else {
    //             tft180_show_gray_image(16, 16, (const uint8 *) gray_image, 94, 45, 94, 45);
    //             tft180_draw_border_line(16, 16, left_border, RGB565_RED);
    //             tft180_draw_border_line(16, 16, right_border, RGB565_RED);
    //             tft180_draw_border_line(16, 16, left_distance_line, RGB565_YELLOW);
    //             tft180_draw_border_line(16, 16, right_distance_line, RGB565_YELLOW);
    //             tft180_draw_border_line(16, 16, distance_middle_line, RGB565_GREEN);
    //         }
    //         break;
    //     case 6:
    //         if(perspective){
    //             tft180_show_binary_image(16, 16, (const uint8 *) gray_binary_pers_image, 47, 45);
    //         } else {
    //             tft180_show_binary_image(16, 16, (const uint8 *) gray_binary_image, 94, 45);
    //         }
    //         break;
    //     case 7:
    //         if(perspective){
    //             tft180_show_gray_image(16, 16, (const uint8 *) contrast_pers_image, 47, 45, 47, 45);
    //             tft180_draw_border_line(16, 16, narrow_line, RGB565_RED);
    //         } else {
    //             tft180_show_gray_image(16, 16, (const uint8 *) contrast_image, 94, 45, 94, 45);
    //         }
    //         break;
    //     default: ;
    // }
}

void UI_key_process(){
    static int8 key_forward_pressed = 0;
    static int8 key_up_pressed = 0;
    static int8 key_down_pressed = 0;
    static int8 key_back_pressed = 0;
    static uint32 press_start_time = 0;

    uint8 dip_switch = DIP_SWITCH;
    UI_item *item = &items[dip_switch][cursor_pos];
    // 切换模式
    // 按住1秒以上保存flash
    if(KEY_FORWARD && !key_forward_pressed){
        // press_start_time = system_getval_ms();
        key_forward_pressed = 1;
        key_pressed = 1;
        switch (ui_state) {
            case 0:
                ui_state = 1;
                break;
            case 1:
                ui_state = 2;
                break;
            case 2:
                ui_state = 0;
                break;
            default: ;
        }
    } else if(!KEY_FORWARD && key_forward_pressed){
        key_forward_pressed = 0;
        // if(system_getval_ms() - press_start_time > 1000){
        //     counter.save_flash_led = 1000;
        //     // flash_write_all();
        //     switch (ui_state) {
        //         case 0:
        //             ui_state = 2;
        //             break;
        //         case 1:
        //             ui_state = 0;
        //             break;
        //         case 2:
        //             ui_state = 1;
        //             break;
        //         default: ;
        //     }
        // }
    }

    // 0:上移光标 1:增加数值 2:增加数值的指数
    // 按住1秒以上读取flash
    if(KEY_UP && !key_up_pressed){
        // press_start_time = system_getval_ms();
        key_up_pressed = 1;
        key_pressed = 1;
        switch (ui_state) {
            case 0:
                cursor_pos = (cursor_pos + 7) % 8;
                break;
            case 1:
                UI_item_set_value(item, UI_item_get_value(item) + pow(10, exponent));
                break;
            case 2:
                if(exponent < 7) exponent++;
                break;
            default: ;
        }
    } else if(!KEY_UP && key_up_pressed){
        key_up_pressed = 0;
        // if(system_getval_ms() - press_start_time > 1000){
        //     counter.read_flash_led = 1000;
        //     // flash_read_all();
        // }
    }

    // 0:下移光标 1:减少数值 2:减少数值的指数
    // 按住1秒以上切换逆透视图
    if(KEY_DOWN && !key_down_pressed){
        // press_start_time = system_getval_ms();
        key_down_pressed = 1;
        key_pressed = 1;
        switch (ui_state) {
            case 0:
                cursor_pos = (cursor_pos + 1) % 8;
                break;
            case 1:
                UI_item_set_value(item, UI_item_get_value(item) - pow(10, exponent));
                break;
            case 2:
                if(exponent > -7) exponent--;
                break;
            default: ;
        }
    } else if(!KEY_DOWN && key_down_pressed){
        key_down_pressed = 0;
        // if(system_getval_ms() - press_start_time > 1000){
        //     perspective = !perspective;
        // }
    }

    // 切换模式（反）
    if(KEY_BACK && !key_back_pressed){
        key_back_pressed = 1;
        key_pressed = 1;
        flag.start = true;
        // running_start_time = system_getval_ms();
        counter.beep_ms = 300;
    } else if(!KEY_BACK && key_back_pressed){
        key_back_pressed = 0;
    }
}