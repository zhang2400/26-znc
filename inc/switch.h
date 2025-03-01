//
// Created by EiveLL on 25-3-1.
//

#ifndef SWITCH_H
#define SWITCH_H

#include "GPIO.h"

#define SWITCH_1 !switch_1.readValue();
#define SWITCH_2 !switch_2.readValue();

void switch_init();
bool switch1();
bool switch2();

#endif //SWITCH_H
