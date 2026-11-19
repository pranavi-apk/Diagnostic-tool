/**
 * @file diag_types.h
 * @brief Shared status codes for diagnostic platform modules.
 */
#ifndef DIAG_TYPES_H
#define DIAG_TYPES_H

#include <stdint.h>

typedef enum
{
    DIAG_OK = 0,
    DIAG_ERR_TIMEOUT,
    DIAG_ERR_NACK,
    DIAG_ERR_INVALID_DATA,
    DIAG_ERR_CAPTURE,
    DIAG_ERR_NO_DEVICE,
    DIAG_ERR_RECOVERY_FAILED,
    DIAG_ERR_BUSY,
    DIAG_ERR_NOT_INIT
} DiagErrorCode;

typedef enum
{
    MODULE_STATE_IDLE = 0,
    MODULE_STATE_INIT,
    MODULE_STATE_RUNNING,
    MODULE_STATE_FAILED,
    MODULE_STATE_RECOVERING,
    MODULE_STATE_DONE
} ModuleState;

typedef enum
{
    MEASURE_VERDICT_NONE = 0,
    MEASURE_VERDICT_PASS,
    MEASURE_VERDICT_FAIL,
    MEASURE_VERDICT_INVALID,
    MEASURE_VERDICT_NO_SIGNAL
} MeasureVerdict;

#endif /* DIAG_TYPES_H */
