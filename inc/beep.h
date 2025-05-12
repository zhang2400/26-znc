//
// Created by EiveLL on 25-2-27.
//

#ifndef BEEP_H
#define BEEP_H
#include "GPIO.h"

class BEEP{
public:
    explicit BEEP(int beepnum);

    void beep_on();
    void beep_off();
    void beep_ms(int ms);
private:
    GPIO beep;
};

#endif //BEEP_H
