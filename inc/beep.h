//
// Created by EiveLL on 25-2-27.
//

#ifndef BEEP_H
#define BEEP_H
#include "GPIO.h"

class BEEP{
public:
    BEEP(int beepnum);

    void beep_on(void);
    void beep_off(void);
private:
    GPIO beep;
};

#endif //BEEP_H
