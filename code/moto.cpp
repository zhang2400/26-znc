//
// Created by EiveLL on 25-1-23.
//

#include "moto.h"

Moto::Moto(int pwmNum, int dirgpio, int encoderNum, int encgpio)
    : motor(pwmNum), direction(dirgpio), encoder(encoderNum, encgpio), speed(0)
{
    motor.set_frequency(15000);
    motor.set_duty(0);
    motor.enable();
    direction.setDirection("out");
    direction.setValue(false);
}

Moto::~Moto()
{
    motor.set_duty(0);
    motor.disable();
    direction.setValue(false);
}

void Moto::set_speed(int duty)
{
    if (duty > WHEEL_MAX_DUTY) {
        duty = WHEEL_MAX_DUTY;
    } else if (duty < -WHEEL_MAX_DUTY) {
        duty = -WHEEL_MAX_DUTY;
    }
    if (duty > 0)
    {
        direction.setValue(false);
    }
    else
    {
        direction.setValue(true);
        duty = -duty;
    }
    motor.set_duty(duty);
}

void Moto::update_speed()
{
    speed = encoder.pulse_counter_update();
}
