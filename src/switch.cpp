//
// Created by EiveLL on 25-3-1.
//

#include "switch.h"

GPIO switch_1(GPIO42);
GPIO switch_2(GPIO44);
GPIO switch_3(GPIO43);
GPIO switch_4(GPIO61);

GPIO dip_switch_1(GPIO74);

void switch_init()
{
    switch_1.setDirection("in");
    switch_2.setDirection("in");
    switch_3.setDirection("in");
    switch_4.setDirection("in");

    dip_switch_1.setDirection("in");
}

int dip_switch(){
    return DIP_SWITCH;
}

int switch1() {
    return SWITCH_1;
}

int switch2() {
    return SWITCH_2;
}

int switch3() {
    return SWITCH_3;
}

int switch4() {
    return SWITCH_4;
}