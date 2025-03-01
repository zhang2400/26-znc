//
// Created by EiveLL on 25-3-1.
//
#include "switch.h"
GPIO switch_1 (GPIO44);
GPIO switch_2 (GPIO45);

void switch_init()
{
    switch_1.setDirection("in");
    switch_2.setDirection("in");

    switch_1.setValue(true);
    switch_2.setValue(true);
}

bool switch1() {
    return SWITCH_1;
}

bool switch2() {
    return SWITCH_2;
}

