/**
 * @file pwm_gen.c
 * TIM8 CH1 on PC6 — advanced frequency selection and safe duty extremes.
 */
#include "pwm_gen.h"
#include "stm32f1xx_hal.h"

#define PWM_TIM_APB2_HZ       72000000UL
#define PWM_TIM_PSC_MAX       65535U
#define PWM_TIM_ARR_MAX       65535U

static uint8_t s_duty = 50U;
static uint8_t s_output_on = 0U;   /* User-selected output ON */
static uint8_t s_pwm_active = 0U;  /* Timer channel driving pin */
static uint32_t s_freq_hz = 1000U;
static ModuleState s_state = MODULE_STATE_IDLE;
static uint16_t s_psc = 71U;
static uint16_t s_arr = 999U;
static uint32_t s_actual_hz = 1000U;

static uint32_t pwm_compute_actual_hz(uint16_t psc, uint16_t arr)
{
    uint64_t denom = (uint64_t)(psc + 1U) * (uint64_t)(arr + 1U);
    if (denom == 0ULL)
    {
        return 0U;
    }
    return (uint32_t)(PWM_TIM_APB2_HZ / denom);
}

static void pwm_select_timer_regs(uint32_t target_hz, uint16_t *psc_out, uint16_t *arr_out)
{
    uint32_t best_err = 0xFFFFFFFFUL;
    uint16_t best_psc = 71U;
    uint16_t best_arr = 999U;
    uint16_t psc;

    if (target_hz < PWM_GEN_MIN_FREQUENCY_HZ)
    {
        target_hz = PWM_GEN_MIN_FREQUENCY_HZ;
    }
    if (target_hz > PWM_GEN_MAX_FREQUENCY_HZ)
    {
        target_hz = PWM_GEN_MAX_FREQUENCY_HZ;
    }
    if (target_hz == 0U)
    {
        target_hz = 1000U;
    }

    for (psc = 0U; psc <= 2000U; psc++)
    {
        uint64_t product = (uint64_t)target_hz * (uint64_t)(psc + 1U);
        uint32_t arr_plus1;

        if (product > (uint64_t)PWM_TIM_APB2_HZ)
        {
            continue;
        }

        arr_plus1 = (uint32_t)(PWM_TIM_APB2_HZ / product);
        if (arr_plus1 < 2U)
        {
            continue;
        }
        if (arr_plus1 > ((uint32_t)PWM_TIM_ARR_MAX + 1U))
        {
            continue;
        }

        {
            uint32_t actual = pwm_compute_actual_hz(psc, (uint16_t)(arr_plus1 - 1U));
            uint32_t err = (actual > target_hz) ? (actual - target_hz) : (target_hz - actual);

            if (err < best_err)
            {
                best_err = err;
                best_psc = psc;
                best_arr = (uint16_t)(arr_plus1 - 1U);
            }
            if (err == 0U)
            {
                break;
            }
        }
    }

    *psc_out = best_psc;
    *arr_out = best_arr;
}

static void pwm_force_pin_low(void)
{
    TIM8->CCER &= ~(1U << 0);
    GPIOC->CRL &= ~(0xFU << 24);
    GPIOC->CRL |= (0x3U << 24);
    GPIOC->BRR = (1U << 6);
    s_pwm_active = 0U;
}

static void pwm_force_pin_high(void)
{
    TIM8->CCER &= ~(1U << 0);
    GPIOC->CRL &= ~(0xFU << 24);
    GPIOC->CRL |= (0x3U << 24);
    GPIOC->BSRR = (1U << 6);
    s_pwm_active = 0U;
}

static void pwm_enable_timer_output(void)
{
    GPIOC->CRL &= ~(0xFU << 24);
    GPIOC->CRL |= (0xBU << 24);
    TIM8->BDTR |= (1U << 15);
    TIM8->CCER |= (1U << 0);
    TIM8->CR1 |= 1U;
    s_pwm_active = 1U;
}

static void pwm_apply_output_state(void)
{
    if (s_output_on == 0U)
    {
        pwm_force_pin_low();
        return;
    }
    if (s_duty == 0U)
    {
        pwm_force_pin_low();
        return;
    }
    if (s_duty >= 100U)
    {
        pwm_force_pin_high();
        return;
    }
    pwm_enable_timer_output();
}

static void pwm_apply_compare(uint16_t arr, uint8_t duty)
{
    uint32_t period = (uint32_t)arr + 1U;
    uint32_t ccr;

    if (duty == 0U)
    {
        ccr = 0U;
    }
    else if (duty >= 100U)
    {
        ccr = period;
    }
    else
    {
        ccr = (period * (uint32_t)duty + 50U) / 100U;
        if (ccr >= period)
        {
            ccr = period;
        }
    }

    TIM8->CCR1 = ccr;
}

static void pwm_apply_frequency(void)
{
    pwm_select_timer_regs(s_freq_hz, &s_psc, &s_arr);
    s_actual_hz = pwm_compute_actual_hz(s_psc, s_arr);

    TIM8->PSC = s_psc;
    TIM8->ARR = s_arr;
    pwm_apply_compare(s_arr, s_duty);
    TIM8->EGR = 1U;
}

void PwmGen_Init(void)
{
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIOC->CRL &= ~(0xFU << 24);
    GPIOC->CRL |= (0xBU << 24);

    TIM8->CR1 = 0U;
    TIM8->PSC = s_psc;
    TIM8->ARR = s_arr;
    TIM8->CCR1 = 0U;
    TIM8->CCMR1 &= ~(0xFFU);
    TIM8->CCMR1 |= (6U << 4) | (1U << 3);
    TIM8->CCER &= ~((1U << 1) | (1U << 0));
    TIM8->CR1 |= (1U << 7);
    TIM8->EGR = 1U;
    TIM8->BDTR |= (1U << 15);
    TIM8->CR1 |= 1U;
    TIM8->CCER &= ~(1U << 0);

    GPIOC->CRL &= ~(0xFU << 24);
    GPIOC->CRL |= (0x3U << 24);
    GPIOC->BRR = (1U << 6);

    pwm_apply_frequency();
    s_output_on = 0U;
    s_state = MODULE_STATE_RUNNING;
}

void PwmGen_Start(void)
{
    pwm_apply_frequency();
    s_output_on = 1U;
    pwm_apply_output_state();
}

void PwmGen_Stop(void)
{
    s_output_on = 0U;
    pwm_force_pin_low();
}

void PwmGen_SetFrequencyHz(uint32_t frequency_hz)
{
    if (frequency_hz < PWM_GEN_MIN_FREQUENCY_HZ)
    {
        frequency_hz = PWM_GEN_MIN_FREQUENCY_HZ;
    }
    if (frequency_hz > PWM_GEN_MAX_FREQUENCY_HZ)
    {
        frequency_hz = PWM_GEN_MAX_FREQUENCY_HZ;
    }
    s_freq_hz = frequency_hz;
    pwm_apply_frequency();
    pwm_apply_output_state();
}

void PwmGen_SetDutyPercent(uint8_t duty_percent)
{
    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }
    s_duty = duty_percent;
    pwm_apply_compare(s_arr, s_duty);
    pwm_apply_output_state();
}

void PwmGen_GetConfig(PwmGen_Config *out)
{
    if (out == NULL)
    {
        return;
    }
    out->requested_frequency_hz = s_freq_hz;
    out->actual_frequency_hz = s_actual_hz;
    out->duty_percent = s_duty;
    out->output_enabled = s_output_on;
    out->timer_psc = s_psc;
    out->timer_arr = s_arr;
    out->timer_ccr = (uint16_t)TIM8->CCR1;
    out->module_state = s_state;
}

uint32_t PwmGen_GetFrequencyHz(void)
{
    return s_freq_hz;
}

uint8_t PwmGen_GetDutyPercent(void)
{
    return s_duty;
}

uint8_t PwmGen_IsEnabled(void)
{
    return s_output_on;
}
