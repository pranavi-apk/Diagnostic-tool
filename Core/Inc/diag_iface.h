/**
 * @file diag_iface.h
 * @brief Common diagnostic status snapshot for all modules.
 */
#ifndef DIAG_IFACE_H
#define DIAG_IFACE_H

#include "diag_types.h"
#include "pwm_gen.h"
#include "signal_analyzer.h"
#include "i2c_scanner.h"

typedef struct
{
    PwmGen_Config pwm;
    SignalAnalyzer_Snapshot signal;
    I2cScanner_Result i2c;
    ModuleState bme_state;
    DiagErrorCode bme_error;
    uint8_t bme_initialized;
} DiagPlatform_Status;

void DiagPlatform_GetStatus(DiagPlatform_Status *out);
void DiagPlatform_SetBmeStatus(ModuleState state, DiagErrorCode err, uint8_t initialized);

#endif /* DIAG_IFACE_H */
