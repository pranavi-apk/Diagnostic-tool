/**
 * @file diag_sequence.h
 * @brief Automated diagnostic sequence coordinating PWM / analyzer / I2C / BME.
 */
#ifndef DIAG_SEQUENCE_H
#define DIAG_SEQUENCE_H

#include <stdint.h>
#include "diag_types.h"
#include "signal_analyzer.h"

#define DIAG_SEQ_TARGET_HZ           1000UL
#define DIAG_SEQ_TARGET_DUTY         50U
#define DIAG_SEQ_FREQ_TOL_PPM        20000U  /* ±2% */
#define DIAG_SEQ_DUTY_TOL_PERCENT    5U
#define DIAG_SEQ_SIGNAL_WAIT_MS      15000U /* time to connect PC6→PA6 */
#define DIAG_SEQ_SIGNAL_SETTLE_MS    400U   /* hold stable after first lock */
#define DIAG_SEQ_MIN_SAMPLES         8U

typedef enum
{
    DIAG_SEQ_IDLE = 0,
    DIAG_SEQ_RUNNING,
    DIAG_SEQ_DONE
} DiagSequence_RunState;

typedef struct
{
    DiagSequence_RunState run_state;
    ModuleState module_state;
    const char *step_name;

    /* PWM ↔ analyzer loopback */
    uint32_t expected_hz;
    uint32_t measured_hz;
    uint8_t  expected_duty;
    uint8_t  measured_duty;
    uint16_t sample_count;
    MeasureVerdict loop_verdict;
    uint8_t  loop_pass;

    /* I2C */
    uint8_t  i2c_device_count;
    uint8_t  i2c_pass;
    uint8_t  recovery_attempted;
    uint8_t  recovery_ok;

    /* BME280 */
    uint8_t  bme_chip_id;
    uint8_t  bme_pass;

    /* Overall */
    uint8_t  overall_pass;
    DiagErrorCode last_error;
    uint16_t wait_remaining_s; /* while waiting for loopback */
} DiagSequence_Report;

void DiagSequence_Start(void);
void DiagSequence_Tick(void);
uint8_t DiagSequence_IsBusy(void);
void DiagSequence_GetReport(DiagSequence_Report *out);

#endif /* DIAG_SEQUENCE_H */
