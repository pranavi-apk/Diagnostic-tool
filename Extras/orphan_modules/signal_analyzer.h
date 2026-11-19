/**
 * @file signal_analyzer.h
 * @brief TIM3 multi-cycle input capture on PA6.
 */
#ifndef SIGNAL_ANALYZER_H
#define SIGNAL_ANALYZER_H

#include <stdint.h>
#include "diag_types.h"

#define SIGNAL_ANALYZER_SAMPLE_COUNT 16U
#define SIGNAL_ANALYZER_TIMEOUT_MS   300U

typedef struct
{
    uint32_t target_frequency_hz;
    uint16_t frequency_tolerance_ppm;   /* ± ppm, e.g. 20000 = 2% */
    uint8_t  target_duty_percent;
    uint8_t  duty_tolerance_percent;    /* ± percentage points */
} SignalAnalyzer_Limits;

typedef struct
{
    uint32_t frequency_hz_avg;
    uint32_t frequency_hz_min;
    uint32_t frequency_hz_max;
    uint32_t frequency_variation_hz;
    uint32_t period_us_avg;
    uint32_t duty_percent_avg;
    uint16_t valid_sample_count;
    uint8_t  signal_valid;
    MeasureVerdict verdict;
    ModuleState module_state;
    DiagErrorCode last_error;
} SignalAnalyzer_Snapshot;

/* Legacy dashboard aliases (updated by module). */
extern volatile uint32_t Signal_Frequency_Hz;
extern volatile uint32_t Signal_Period_us;
extern volatile uint32_t Signal_Duty_Percent;
extern volatile uint8_t Signal_Detected;

void SignalAnalyzer_Init(void);
void SignalAnalyzer_Tick(void);
void SignalAnalyzer_GetSnapshot(SignalAnalyzer_Snapshot *out);
void SignalAnalyzer_SetLimits(const SignalAnalyzer_Limits *limits);
MeasureVerdict SignalAnalyzer_Evaluate(const SignalAnalyzer_Limits *limits);

void SignalAnalyzer_TIM3_IRQHandler(void);

#endif /* SIGNAL_ANALYZER_H */
