//
// Created by EiveLL on 25-2-27.
//

#include "beep.h"
#include <counter.h>

BEEP::BEEP(int beepnum)
    :beep(beepnum)
{
    // beep.setDirection("out");
    // beep_off();
}

void BEEP::beep_on() {
    // beep.setValue(true);
}

void BEEP::beep_off() {
    // beep.setValue(false);
}

void BEEP::beep_ms(int ms) {
    // counter.beep_ms = ms;
}
