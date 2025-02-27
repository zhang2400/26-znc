//
// Created by EiveLL on 25-1-23.
//

#include "moto.h"
#include "config.h"
Moto::Moto(int pwmNum, int dirgpio, int encoderNum, int encgpio)
    : motor(pwmNum), direction(dirgpio), encoder(encoderNum, encgpio), speed(0)
{
    motor.set_frequency(15000);
    motor.set_duty(0);
    motor.enable();
    direction.setDirection("out");
    direction.setValue(false);

}

void Moto::set_speed(int _speed)
{
    if (_speed > PWM_DUTY_MAX) {
        _speed = PWM_DUTY_MAX;
    } else if (_speed < -PWM_DUTY_MAX) {
        _speed = -PWM_DUTY_MAX;
    }
    if (_speed > 0)
    {
        direction.setValue(false);
    }
    else
    {
        direction.setValue(true);
        _speed = -_speed;
    }
    motor.set_duty(_speed);
}

void Moto::update_speed()
{
    speed = encoder.pulse_counter_update();
}
