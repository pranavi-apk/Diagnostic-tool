/**
 * @file diag_iface.c
 */
#include "diag_iface.h"

static ModuleState s_bme_state = MODULE_STATE_IDLE;
static DiagErrorCode s_bme_error = DIAG_OK;
static uint8_t s_bme_initialized = 0U;

void DiagPlatform_SetBmeStatus(ModuleState state, DiagErrorCode err, uint8_t initialized)
{
    s_bme_state = state;
    s_bme_error = err;
    s_bme_initialized = initialized;
}

void DiagPlatform_GetStatus(DiagPlatform_Status *out)
{
    if (out == NULL)
    {
        return;
    }

    PwmGen_GetConfig(&out->pwm);
    SignalAnalyzer_GetSnapshot(&out->signal);
    I2cScanner_GetResult(&out->i2c);
    out->bme_state = s_bme_state;
    out->bme_error = s_bme_error;
    out->bme_initialized = s_bme_initialized;
}
