/**
 * @file diag_sequence.c
 * Non-blocking automated diagnostic: PWM → measure → I2C → BME → report.
 */
#include "diag_sequence.h"
#include "pwm_gen.h"
#include "signal_analyzer.h"
#include "i2c_scanner.h"
#include "sw_i2c.h"
#include "diag_iface.h"
#include "stm32f1xx_hal.h"

typedef enum
{
    STEP_IDLE = 0,
    STEP_PWM_SETUP,
    STEP_WAIT_SIGNAL,
    STEP_EVAL_LOOP,
    STEP_I2C_START,
    STEP_I2C_WAIT,
    STEP_BME_CHECK,
    STEP_FINISH
} DiagStep;

static DiagStep s_step = STEP_IDLE;
static DiagSequence_Report s_report;
static uint32_t s_deadline = 0U;
static uint32_t s_settle_start = 0U;
static uint8_t s_saw_signal = 0U;
static uint8_t s_pwm_was_on = 0U;
static uint32_t s_pwm_prev_hz = 1000U;
static uint8_t s_pwm_prev_duty = 50U;

static void report_clear(void)
{
    s_report.run_state = DIAG_SEQ_IDLE;
    s_report.module_state = MODULE_STATE_IDLE;
    s_report.step_name = "IDLE";
    s_report.expected_hz = DIAG_SEQ_TARGET_HZ;
    s_report.measured_hz = 0U;
    s_report.expected_duty = DIAG_SEQ_TARGET_DUTY;
    s_report.measured_duty = 0U;
    s_report.sample_count = 0U;
    s_report.loop_verdict = MEASURE_VERDICT_NONE;
    s_report.loop_pass = 0U;
    s_report.i2c_device_count = 0U;
    s_report.i2c_pass = 0U;
    s_report.recovery_attempted = 0U;
    s_report.recovery_ok = 0U;
    s_report.bme_chip_id = 0U;
    s_report.bme_pass = 0U;
    s_report.overall_pass = 0U;
    s_report.last_error = DIAG_OK;
    s_report.wait_remaining_s = (uint16_t)(DIAG_SEQ_SIGNAL_WAIT_MS / 1000U);
}

void DiagSequence_Start(void)
{
    report_clear();

    s_pwm_was_on = PwmGen_IsEnabled();
    s_pwm_prev_hz = PwmGen_GetFrequencyHz();
    s_pwm_prev_duty = PwmGen_GetDutyPercent();

    s_saw_signal = 0U;
    s_settle_start = 0U;
    s_report.run_state = DIAG_SEQ_RUNNING;
    s_report.module_state = MODULE_STATE_INIT;
    s_report.step_name = "PWM SETUP";
    s_step = STEP_PWM_SETUP;
}

uint8_t DiagSequence_IsBusy(void)
{
    return (uint8_t)(s_report.run_state == DIAG_SEQ_RUNNING);
}

void DiagSequence_GetReport(DiagSequence_Report *out)
{
    if (out != NULL)
    {
        *out = s_report;
    }
}

void DiagSequence_Tick(void)
{
    SignalAnalyzer_Snapshot snap;
    SignalAnalyzer_Limits lim;
    I2cScanner_Result i2c;
    uint8_t chip_id = 0U;

    if (s_report.run_state != DIAG_SEQ_RUNNING)
    {
        return;
    }

    switch (s_step)
    {
    case STEP_PWM_SETUP:
        PwmGen_SetFrequencyHz(DIAG_SEQ_TARGET_HZ);
        PwmGen_SetDutyPercent(DIAG_SEQ_TARGET_DUTY);
        PwmGen_Start();

        lim.target_frequency_hz = DIAG_SEQ_TARGET_HZ;
        lim.frequency_tolerance_ppm = DIAG_SEQ_FREQ_TOL_PPM;
        lim.target_duty_percent = DIAG_SEQ_TARGET_DUTY;
        lim.duty_tolerance_percent = DIAG_SEQ_DUTY_TOL_PERCENT;
        SignalAnalyzer_SetLimits(&lim);

        s_deadline = HAL_GetTick();
        s_saw_signal = 0U;
        s_settle_start = 0U;
        s_report.step_name = "WAIT LOOPBACK";
        s_report.module_state = MODULE_STATE_RUNNING;
        s_report.wait_remaining_s =
            (uint16_t)(DIAG_SEQ_SIGNAL_WAIT_MS / 1000U);
        s_step = STEP_WAIT_SIGNAL;
        break;

    case STEP_WAIT_SIGNAL:
    {
        uint32_t elapsed = HAL_GetTick() - s_deadline;
        uint32_t left_ms = (elapsed < DIAG_SEQ_SIGNAL_WAIT_MS) ?
                           (DIAG_SEQ_SIGNAL_WAIT_MS - elapsed) : 0U;
        s_report.wait_remaining_s = (uint16_t)((left_ms + 999U) / 1000U);

        SignalAnalyzer_GetSnapshot(&snap);

        if ((snap.valid_sample_count >= DIAG_SEQ_MIN_SAMPLES) &&
            (snap.signal_valid != 0U))
        {
            if (s_saw_signal == 0U)
            {
                /* First lock — allow wire to settle before scoring. */
                s_saw_signal = 1U;
                s_settle_start = HAL_GetTick();
                s_report.step_name = "SIGNAL LOCK";
            }
            else if ((HAL_GetTick() - s_settle_start) >= DIAG_SEQ_SIGNAL_SETTLE_MS)
            {
                s_report.wait_remaining_s = 0U;
                s_report.step_name = "EVAL LOOP";
                s_step = STEP_EVAL_LOOP;
                break;
            }
        }
        else
        {
            /* Lost / not connected yet — keep waiting until total timeout. */
            s_saw_signal = 0U;
            s_settle_start = 0U;
            s_report.step_name = "WAIT LOOPBACK";
        }

        if (elapsed > DIAG_SEQ_SIGNAL_WAIT_MS)
        {
            s_report.wait_remaining_s = 0U;
            s_report.loop_verdict = MEASURE_VERDICT_NO_SIGNAL;
            s_report.loop_pass = 0U;
            s_report.last_error = DIAG_ERR_TIMEOUT;
            s_report.step_name = "I2C SCAN";
            s_step = STEP_I2C_START;
        }
        break;
    }

    case STEP_EVAL_LOOP:
        SignalAnalyzer_GetSnapshot(&snap);
        s_report.measured_hz = snap.frequency_hz_avg;
        s_report.measured_duty = (uint8_t)snap.duty_percent_avg;
        s_report.sample_count = snap.valid_sample_count;
        s_report.loop_verdict = snap.verdict;
        s_report.loop_pass =
            (uint8_t)(snap.verdict == MEASURE_VERDICT_PASS);
        if (s_report.loop_pass == 0U)
        {
            s_report.last_error = DIAG_ERR_INVALID_DATA;
        }
        s_report.step_name = "I2C SCAN";
        s_step = STEP_I2C_START;
        break;

    case STEP_I2C_START:
        I2cScanner_Begin();
        s_report.step_name = "I2C SCAN";
        s_step = STEP_I2C_WAIT;
        break;

    case STEP_I2C_WAIT:
        if (I2cScanner_IsBusy())
        {
            I2cScanner_Tick();
            break;
        }
        I2cScanner_GetResult(&i2c);
        s_report.i2c_device_count = i2c.total_responding;
        s_report.recovery_attempted = i2c.recovery_attempted;
        s_report.recovery_ok = i2c.recovery_ok;
        /* Pass if at least one device ACKed (typically EEPROM and/or BME). */
        s_report.i2c_pass = (uint8_t)(i2c.total_responding > 0U);
        if (s_report.i2c_pass == 0U)
        {
            s_report.last_error = DIAG_ERR_NO_DEVICE;
        }
        s_report.step_name = "BME CHECK";
        s_step = STEP_BME_CHECK;
        break;

    case STEP_BME_CHECK:
        SwI2c_InitPins();
        if (SwI2c_ReadRegister(BME280_I2C_ADDR_7BIT, 0xD0U, &chip_id) == SW_I2C_ACK)
        {
            s_report.bme_chip_id = chip_id;
            s_report.bme_pass = (uint8_t)(chip_id == BME280_CHIP_ID);
            DiagPlatform_SetBmeStatus(
                s_report.bme_pass ? MODULE_STATE_RUNNING : MODULE_STATE_FAILED,
                s_report.bme_pass ? DIAG_OK : DIAG_ERR_INVALID_DATA,
                s_report.bme_pass);
        }
        else
        {
            s_report.bme_chip_id = 0U;
            s_report.bme_pass = 0U;
            s_report.last_error = DIAG_ERR_NO_DEVICE;
            DiagPlatform_SetBmeStatus(MODULE_STATE_FAILED, DIAG_ERR_NO_DEVICE, 0U);
        }
        s_report.step_name = "FINISH";
        s_step = STEP_FINISH;
        break;

    case STEP_FINISH:
        s_report.overall_pass = (uint8_t)(s_report.loop_pass &&
                                          s_report.i2c_pass &&
                                          s_report.bme_pass);
        s_report.run_state = DIAG_SEQ_DONE;
        s_report.module_state = s_report.overall_pass ?
                                MODULE_STATE_DONE : MODULE_STATE_FAILED;
        s_report.step_name = "DONE";
        s_step = STEP_IDLE;

        /* Restore prior PWM settings (leave output as user had it). */
        PwmGen_SetFrequencyHz(s_pwm_prev_hz);
        PwmGen_SetDutyPercent(s_pwm_prev_duty);
        if (s_pwm_was_on != 0U)
        {
            PwmGen_Start();
        }
        else
        {
            PwmGen_Stop();
        }
        break;

    case STEP_IDLE:
    default:
        break;
    }
}
