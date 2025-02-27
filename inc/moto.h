//
// Created by EiveLL on 25-1-23.
//

#ifndef MOTO_H
#define MOTO_H

#include <pwm.h>
#include <encoder.h>
#include "GPIO.h"
#include "config.h"

class Moto {
public:
    Moto(int pwmNum, int dirgpio, int encoderNum, int encgpio, bool inverse);
    ~Moto();
    void set_speed(int duty);
    void update_speed();
    int speed;
private:
    bool inverse;
    PWM motor;
    GPIO direction;
    ENCODER encoder;
};

#endif //MOTO_H
