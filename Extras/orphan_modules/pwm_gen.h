/**
 * @file pwm_gen.h
 * @brief TIM8 CH1 PWM on PC6 — frequency/duty generation for diagnostics.
 */
#ifndef PWM_GEN_H
#define PWM_GEN_H

#include <stdint.h>
#include "diag_types.h"

#define PWM_GEN_MIN_FREQUENCY_HZ  100UL
#define PWM_GEN_MAX_FREQUENCY_HZ  10000UL
#define PWM_GEN_FREQUENCY_STEP_HZ 100UL

typedef struct
{
    uint32_t requested_frequency_hz;
    uint32_t actual_frequency_hz;
    uint8_t  duty_percent;
    uint8_t  output_enabled;
    uint16_t timer_psc;
    uint16_t timer_arr;
    uint16_t timer_ccr;
    ModuleState module_state;
} PwmGen_Config;

void PwmGen_Init(void);
void PwmGen_Start(void);
void PwmGen_Stop(void);
void PwmGen_SetFrequencyHz(uint32_t frequency_hz);
void PwmGen_SetDutyPercent(uint8_t duty_percent);
void PwmGen_GetConfig(PwmGen_Config *out);
uint32_t PwmGen_GetFrequencyHz(void);
uint8_t PwmGen_GetDutyPercent(void);
uint8_t PwmGen_IsEnabled(void);

#endif /* PWM_GEN_H */
