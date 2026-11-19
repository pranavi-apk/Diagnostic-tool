/**
 * @file signal_analyzer.c
 */
#include "signal_analyzer.h"
#include "stm32f1xx_hal.h"

#define ANALYZER_MIN_PERIOD_US  2U
#define ANALYZER_MAX_PERIOD_US  10000000UL
#define ANALYZER_MIN_FREQ_HZ    100UL
#define ANALYZER_MAX_FREQ_HZ    50000UL

volatile uint32_t Signal_Frequency_Hz = 0U;
volatile uint32_t Signal_Period_us = 0U;
volatile uint32_t Signal_Duty_Percent = 0U;
volatile uint8_t Signal_Detected = 0U;

typedef struct
{
    uint32_t period_us;
    uint32_t frequency_hz;
    uint32_t high_us;
    uint32_t duty_percent;
} AnalyzerCycleSample;

static AnalyzerCycleSample s_samples[SIGNAL_ANALYZER_SAMPLE_COUNT];
static uint8_t s_sample_count = 0U;
static uint8_t s_sample_write = 0U;

static uint16_t s_last_rising = 0U;
static uint8_t s_have_rising = 0U;
static volatile uint32_t s_last_edge_ms = 0U;
static uint32_t s_pending_high_us = 0U;
static uint8_t s_pending_high_valid = 0U;

static ModuleState s_module_state = MODULE_STATE_IDLE;
static SignalAnalyzer_Limits s_limits = {
    1000U, 20000U, 50U, 5U
};
static MeasureVerdict s_verdict = MEASURE_VERDICT_NONE;
static DiagErrorCode s_last_error = DIAG_OK;

static uint8_t analyzer_period_valid(uint32_t period_us)
{
    if (period_us < ANALYZER_MIN_PERIOD_US)
    {
        return 0U;
    }
    if (period_us > ANALYZER_MAX_PERIOD_US)
    {
        return 0U;
    }
    return 1U;
}

static void analyzer_commit_sample(uint32_t period_us, uint32_t high_us)
{
    uint32_t freq_hz;
    uint32_t duty;

    if (!analyzer_period_valid(period_us))
    {
        s_last_error = DIAG_ERR_INVALID_DATA;
        return;
    }

    freq_hz = 1000000UL / period_us;
    if ((freq_hz < ANALYZER_MIN_FREQ_HZ) || (freq_hz > ANALYZER_MAX_FREQ_HZ))
    {
        s_last_error = DIAG_ERR_INVALID_DATA;
        return;
    }

    if (high_us > period_us)
    {
        s_last_error = DIAG_ERR_CAPTURE;
        return;
    }

    duty = (high_us * 100U + (period_us / 2U)) / period_us;
    if (duty > 100U)
    {
        duty = 100U;
    }

    s_samples[s_sample_write].period_us = period_us;
    s_samples[s_sample_write].frequency_hz = freq_hz;
    s_samples[s_sample_write].high_us = high_us;
    s_samples[s_sample_write].duty_percent = duty;
    s_sample_write = (uint8_t)((s_sample_write + 1U) % SIGNAL_ANALYZER_SAMPLE_COUNT);
    if (s_sample_count < SIGNAL_ANALYZER_SAMPLE_COUNT)
    {
        s_sample_count++;
    }
    s_last_error = DIAG_OK;
}

static void analyzer_publish_legacy(uint32_t freq, uint32_t period, uint32_t duty)
{
    Signal_Frequency_Hz = freq;
    Signal_Period_us = period;
    Signal_Duty_Percent = duty;
    Signal_Detected = 1U;
}

static void analyzer_clear_measurements(void)
{
    s_sample_count = 0U;
    s_sample_write = 0U;
    s_pending_high_valid = 0U;
    Signal_Frequency_Hz = 0U;
    Signal_Period_us = 0U;
    Signal_Duty_Percent = 0U;
    Signal_Detected = 0U;
    s_verdict = MEASURE_VERDICT_NO_SIGNAL;
}

static void analyzer_recompute_stats(SignalAnalyzer_Snapshot *snap)
{
    uint8_t i;
    uint32_t sum_period = 0U;
    uint32_t sum_duty = 0U;
    uint32_t sum_freq = 0U;
    uint32_t fmin = 0xFFFFFFFFUL;
    uint32_t fmax = 0U;
    uint16_t n;

    if (snap == NULL)
    {
        return;
    }

    n = s_sample_count;
    snap->valid_sample_count = n;
    if (n == 0U)
    {
        snap->signal_valid = 0U;
        snap->frequency_hz_avg = 0U;
        snap->frequency_hz_min = 0U;
        snap->frequency_hz_max = 0U;
        snap->frequency_variation_hz = 0U;
        snap->period_us_avg = 0U;
        snap->duty_percent_avg = 0U;
        snap->verdict = MEASURE_VERDICT_NO_SIGNAL;
        snap->module_state = s_module_state;
        snap->last_error = s_last_error;
        return;
    }

    for (i = 0U; i < n; i++)
    {
        uint8_t idx = (uint8_t)((s_sample_write + SIGNAL_ANALYZER_SAMPLE_COUNT - 1U - i) %
                                SIGNAL_ANALYZER_SAMPLE_COUNT);
        uint32_t f = s_samples[idx].frequency_hz;

        sum_period += s_samples[idx].period_us;
        sum_duty += s_samples[idx].duty_percent;
        sum_freq += f;
        if (f < fmin)
        {
            fmin = f;
        }
        if (f > fmax)
        {
            fmax = f;
        }
    }

    snap->period_us_avg = sum_period / n;
    snap->duty_percent_avg = sum_duty / n;
    snap->frequency_hz_avg = sum_freq / n;
    snap->frequency_hz_min = fmin;
    snap->frequency_hz_max = fmax;
    snap->frequency_variation_hz = (fmax >= fmin) ? (fmax - fmin) : 0U;
    snap->signal_valid = 1U;
    snap->module_state = s_module_state;
    snap->last_error = s_last_error;

    analyzer_publish_legacy(snap->frequency_hz_avg, snap->period_us_avg, snap->duty_percent_avg);
}

void SignalAnalyzer_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIOA->CRL &= ~(0xFU << 24);
    GPIOA->CRL |= (0x8U << 24);
    GPIOA->ODR &= ~(1U << 6);

    TIM3->CR1 = 0U;
    TIM3->DIER = 0U;
    TIM3->PSC = 71U;
    TIM3->ARR = 0xFFFFU;
    TIM3->CNT = 0U;
    TIM3->CCMR1 = (1U << 0) | (2U << 8);
    TIM3->CCER = (1U << 0) | (1U << 4) | (1U << 5);
    TIM3->SMCR = 0U;
    TIM3->SR = 0U;
    TIM3->DIER = (1U << 1) | (1U << 2);

    s_last_rising = 0U;
    s_have_rising = 0U;
    s_last_edge_ms = 0U;
    analyzer_clear_measurements();
    s_module_state = MODULE_STATE_RUNNING;

    HAL_NVIC_SetPriority(TIM3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->EGR = 1U;
    TIM3->CR1 |= 1U;
}

void SignalAnalyzer_Tick(void)
{
    uint32_t now = HAL_GetTick();

    if ((s_last_edge_ms != 0U) &&
        ((now - s_last_edge_ms) > SIGNAL_ANALYZER_TIMEOUT_MS))
    {
        analyzer_clear_measurements();
        s_have_rising = 0U;
        s_last_error = DIAG_ERR_TIMEOUT;
    }
}

void SignalAnalyzer_GetSnapshot(SignalAnalyzer_Snapshot *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    analyzer_recompute_stats(out);
    out->verdict = SignalAnalyzer_Evaluate(&s_limits);
    s_verdict = out->verdict;
    __set_PRIMASK(primask);
}

void SignalAnalyzer_SetLimits(const SignalAnalyzer_Limits *limits)
{
    if (limits != NULL)
    {
        s_limits = *limits;
    }
}

MeasureVerdict SignalAnalyzer_Evaluate(const SignalAnalyzer_Limits *limits)
{
    uint32_t ppm;
    int32_t duty_delta;
    uint32_t freq_avg = 0U;
    uint32_t duty_avg = 0U;
    uint8_t i;
    uint32_t sum_freq = 0U;
    uint32_t sum_duty = 0U;
    uint16_t n = s_sample_count;

    if (limits == NULL)
    {
        return MEASURE_VERDICT_INVALID;
    }
    if (n == 0U)
    {
        return MEASURE_VERDICT_NO_SIGNAL;
    }

    for (i = 0U; i < n; i++)
    {
        uint8_t idx = (uint8_t)((s_sample_write + SIGNAL_ANALYZER_SAMPLE_COUNT - 1U - i) %
                                SIGNAL_ANALYZER_SAMPLE_COUNT);
        sum_freq += s_samples[idx].frequency_hz;
        sum_duty += s_samples[idx].duty_percent;
    }
    freq_avg = sum_freq / n;
    duty_avg = sum_duty / n;

    if (limits->target_frequency_hz == 0U)
    {
        return MEASURE_VERDICT_NONE;
    }

    ppm = (freq_avg > limits->target_frequency_hz) ?
          ((freq_avg - limits->target_frequency_hz) * 1000000UL /
           limits->target_frequency_hz) :
          ((limits->target_frequency_hz - freq_avg) * 1000000UL /
           limits->target_frequency_hz);

    if (ppm > limits->frequency_tolerance_ppm)
    {
        return MEASURE_VERDICT_FAIL;
    }

    duty_delta = (int32_t)duty_avg - (int32_t)limits->target_duty_percent;
    if (duty_delta < 0)
    {
        duty_delta = -duty_delta;
    }
    if ((uint32_t)duty_delta > limits->duty_tolerance_percent)
    {
        return MEASURE_VERDICT_FAIL;
    }

    return MEASURE_VERDICT_PASS;
}

void SignalAnalyzer_TIM3_IRQHandler(void)
{
    uint32_t status = TIM3->SR;

    if ((status & (1U << 1)) != 0U)
    {
        uint16_t rising = (uint16_t)TIM3->CCR1;

        s_last_edge_ms = HAL_GetTick();

        if (s_have_rising != 0U)
        {
            uint16_t delta = (uint16_t)(rising - s_last_rising);

            if (delta != 0U)
            {
                uint32_t period_us = (uint32_t)delta;
                uint32_t high_us = s_pending_high_valid ? s_pending_high_us : 0U;

                if (s_pending_high_valid != 0U)
                {
                    analyzer_commit_sample(period_us, high_us);
                }
            }
        }

        s_last_rising = rising;
        s_have_rising = 1U;
        s_pending_high_valid = 0U;
    }

    if ((status & (1U << 2)) != 0U)
    {
        uint16_t falling = (uint16_t)TIM3->CCR2;
        uint16_t high_delta;

        s_last_edge_ms = HAL_GetTick();

        if (s_have_rising != 0U)
        {
            high_delta = (uint16_t)(falling - s_last_rising);
            s_pending_high_us = (uint32_t)high_delta;
            s_pending_high_valid = 1U;
        }
    }

    TIM3->SR = ~((1U << 9) | (1U << 10));
}
