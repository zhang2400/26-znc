//
// Created by EiveLL on 25-3-1.
//

#ifndef SWITCH_H
#define SWITCH_H

#include "GPIO.h"

#define DIP_SWITCH_1 (!dip_switch_1.readValue())

#define DIP_SWITCH   (DIP_SWITCH_1)

#define SWITCH_1 (!switch_1.readValue())
#define SWITCH_2 (!switch_2.readValue())
#define SWITCH_3 (!switch_3.readValue())
#define SWITCH_4 (!switch_4.readValue())

#define KEY_FORWARD SWITCH_1
#define KEY_UP SWITCH_2
#define KEY_DOWN SWITCH_3
#define KEY_BACK SWITCH_4

void switch_init();

int dip_switch();

int switch1();

int switch2();

int switch3();

int switch4();

extern GPIO switch_1;
extern GPIO switch_2;
extern GPIO switch_3;
extern GPIO switch_4;

extern GPIO dip_switch_1;
extern GPIO dip_switch_2;
extern GPIO dip_switch_3;

#endif //SWITCH_H
