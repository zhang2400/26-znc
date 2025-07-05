//
//  Created by EiveLL on 25-6-8.
//

#ifndef BLDC_H
#define BLDC_H

#include "PwmController.h"
#include "config.h"

class BLDC : public PwmController {
public:
    // 构造函数：指定BLDC通道、频率、范围
    explicit BLDC(
        int pwmchip = 4,
        int pwmnum = 1,
        uint32_t freq_hz = BLDC_FREQ,      // 默认频率333Hz

        uint32_t max_duty = BLDC_DUTY_MAX,
        uint32_t min_duty = BLDC_DUTY_MIN
    );

    // 设置转速
    void set_bldc_duty(uint32_t duty);

private:
    uint32_t min_duty_;   // 最小占空比
    uint32_t max_duty_;   // 最大占空比
};

#endif //BLDC_H
