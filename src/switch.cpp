//
// Created by EiveLL on 25-3-1.
//

#include "switch.h"

GPIO switch_1(GPIO44);
GPIO switch_2(GPIO45);

// 记录上一次的按键状态
bool last_switch1_state = false;
bool last_switch2_state = false;

void switch_init()
{
    switch_1.setDirection("in");
    switch_2.setDirection("in");
}

// 只在按键 **刚刚** 被按下时返回 true（相当于中断触发一次）
bool switch1() {
    bool current_state = SWITCH_1;  // 读取当前按键状态
    if (current_state && !last_switch1_state) {  // 检测从未按下到按下
        last_switch1_state = true;  // 记录状态
        return true;  // 触发一次
    }
    if (!current_state) {
        last_switch1_state = false;  // 释放按键时重置状态
    }
    return false;  // 其他情况不触发
}

bool switch2() {
    bool current_state = SWITCH_2;
    if (current_state && !last_switch2_state) {
        last_switch2_state = true;
        return true;
    }
    if (!current_state) {
        last_switch2_state = false;
    }
    return false;
}
