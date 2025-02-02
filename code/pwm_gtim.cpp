#include "pwm_gtim.h"

PWM_GTIM::PWM_GTIM(int gpio, int mux, int chNum_, double frequency, int duty_value)
    : chNum(chNum_ - 1)  // chNum 从 1 开始，需要减 1
{
    if (chNum < 0 || chNum > 3) {
        return;
    }

    // 计算周期（10 纳秒单位）
    period_10ns = (unsigned int)(1e8 / frequency);

    // 计算占空比（0～10000 对应 0%～100%）
    duty_cycle_10ns = duty_value;

    // 配置 GPIO 复用功能
    void *gpio_mux_buffer = map_register(GPIO_MUX_BASE_ADDR + (gpio / 16) * 0x04, PAGE_SIZE);
    unsigned int old_mux = REG_READ(gpio_mux_buffer);
    REG_WRITE(gpio_mux_buffer, (old_mux & ~(0b11 << (gpio % 16 * 2))) | (mux << (gpio % 16 * 2)));
    unsigned int new_mux = REG_READ(gpio_mux_buffer);

    // 初始化 GTIM 寄存器
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_EGR_OFFSET, PAGE_SIZE), 0x01);

    // 绑定寄存器
    period_buffer = map_register(GTIM_BASE_ADDR + GTIM_ARR_OFFSET, PAGE_SIZE);
    duty_cycle_buffer = map_register(GTIM_BASE_ADDR + GTIM_CCR1_OFFSET + chNum * 0x04, PAGE_SIZE);
    ccmr_buffer[0] = map_register(GTIM_BASE_ADDR + GTIM_CCMR1_OFFSET, PAGE_SIZE);
    ccmr_buffer[1] = map_register(GTIM_BASE_ADDR + GTIM_CCMR2_OFFSET, PAGE_SIZE);
    ccer_buffer = map_register(GTIM_BASE_ADDR + GTIM_CCER_OFFSET, PAGE_SIZE);
    cnt_buffer = map_register(GTIM_BASE_ADDR + GTIM_CNT_OFFSET, PAGE_SIZE);

    // 配置 PWM 模式（确保 CHx 处于 PWM1 模式）
    unsigned int ccmr_val = REG_READ(ccmr_buffer[chNum / 2]);
    ccmr_val &= ~(0x7 << (chNum % 2 * 8 + 4));  // 清除
    ccmr_val |= (0x6 << (chNum % 2 * 8 + 4));   // 设置 PWM1 模式
    REG_WRITE(ccmr_buffer[chNum / 2], ccmr_val);

    // 设置输出极性（默认不反相）
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 1)));

    // 设置周期和占空比
    REG_WRITE(period_buffer, period_10ns);
    unsigned int duty_ns = (unsigned int)(((uint64_t)period_10ns * duty_value) / 10000);
    REG_WRITE(duty_cycle_buffer, duty_ns);

    // 复位计数器
    REG_WRITE(cnt_buffer, 0);

    printf("[PWM INIT] GPIO %d (CH%d): Freq=%.2f Hz, ARR=%u, CCR=%u (%u%%)\n",
           gpio, chNum + 1, frequency, period_10ns, duty_ns, (duty_value * 100) / 10000);
}

PWM_GTIM::~PWM_GTIM(void)
{
    munmap(ccmr_buffer[0], PAGE_SIZE);
    munmap(ccmr_buffer[1], PAGE_SIZE);
    munmap(period_buffer, PAGE_SIZE);
    munmap(duty_cycle_buffer, PAGE_SIZE);
    munmap(ccer_buffer, PAGE_SIZE);
    munmap(cnt_buffer, PAGE_SIZE);
}

void PWM_GTIM::enable(void)
{
    // 先关闭 CH2/CH3，确保状态干净
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4)));

    // 使能 PWM 通道
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4)));

    // 立即更新 ARR 和 CCR，确保 PWM 正确运行
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_EGR_OFFSET, PAGE_SIZE), 0x01);

    printf("[PWM ENABLE] CH%d Enabled, CCER: 0x%x\n", chNum + 1, REG_READ(ccer_buffer));
}


void PWM_GTIM::disable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4)));
}

// 设置周期（10 纳秒单位）
void PWM_GTIM::setPeriod(unsigned int period_10ns_)
{
    period_10ns = period_10ns_;
    REG_WRITE(period_buffer, period_10ns);

    // 强制更新 ARR
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_EGR_OFFSET, PAGE_SIZE), 0x01);

    // 更新占空比
    unsigned int duty_ns = (unsigned int)(((uint64_t)period_10ns * duty_cycle_10ns) / 10000);
    REG_WRITE(duty_cycle_buffer, duty_ns);
}



void PWM_GTIM::setDutyCycle(unsigned int duty_value)
{
    duty_cycle_10ns = duty_value;
    unsigned int duty_ns = (unsigned int)(((uint64_t)period_10ns * duty_value) / 10000);
    REG_WRITE(duty_cycle_buffer, duty_ns);
    REG_WRITE(cnt_buffer, 0);
}

void PWM_GTIM::setFrequency(double frequency) {
    if (frequency <= 0) {
        printf("Invalid frequency: %f Hz\n", frequency);
        return;
    }

    // 1. 计算新的 ARR（周期）
    period_10ns = (unsigned int)(1e8 / frequency);

    // 2. 禁止 ARR 立即生效（使用影子寄存器）
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_CR1_OFFSET, PAGE_SIZE),
              REG_READ(map_register(GTIM_BASE_ADDR + GTIM_CR1_OFFSET, PAGE_SIZE)) & ~(0x1 << 7)); // 清除 ARPE

    // 3. 更新 ARR（周期寄存器）
    REG_WRITE(period_buffer, period_10ns);

    // 4. 重新计算并更新 CCR（占空比寄存器）
    unsigned int duty_ns = (unsigned int)(((uint64_t)period_10ns * duty_cycle_10ns) / 10000);
    REG_WRITE(duty_cycle_buffer, duty_ns);

    // 5. 使能影子寄存器（确保新 ARR 在下个 PWM 周期自动生效）
    REG_WRITE(map_register(GTIM_BASE_ADDR + GTIM_CR1_OFFSET, PAGE_SIZE),
              REG_READ(map_register(GTIM_BASE_ADDR + GTIM_CR1_OFFSET, PAGE_SIZE)) | (0x1 << 7)); // 置位 ARPE

    printf("Frequency set successfully: %f Hz (New ARR: %u, New CCR: %u)\n", frequency, period_10ns, duty_ns);
}

