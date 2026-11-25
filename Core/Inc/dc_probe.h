/**
 * @file dc_probe.h
 * @brief DC voltage probe via resistor divider on ADC1 (PC0..PC5).
 */
#ifndef DC_PROBE_H
#define DC_PROBE_H

#include <stdint.h>

#define DC_PROBE_ADC_FULL_SCALE_MV   16200U
#define DC_PROBE_DIVIDER_GAIN        (59.0f / 12.0f)
#define DC_PROBE_VREF_MV             3300.0f

typedef enum
{
    DC_PROBE_VERDICT_NONE = 0,
    DC_PROBE_OVER_RANGE,
    DC_PROBE_CHECK_POLARITY,
    DC_PROBE_RAIL_3V3_PASS,
    DC_PROBE_RAIL_3V3_FAIL,
    DC_PROBE_RAIL_5V_PASS,
    DC_PROBE_RAIL_5V_FAIL,
    DC_PROBE_LOGIC_LOW,
    DC_PROBE_LOGIC_HIGH,
    DC_PROBE_LOGIC_INVALID,
    DC_PROBE_MEASURE,
    DC_PROBE_OPEN
} DcProbe_Verdict;

typedef struct
{
    uint16_t raw_avg;
    uint16_t raw_min;
    uint16_t raw_max;
    uint32_t vin_mv;
    float    cal_factor;
    DcProbe_Verdict verdict;
    uint8_t  initialized;
    uint8_t  voltage_valid;
} DcProbe_Snapshot;

void DcProbe_Init(void);
void DcProbe_Tick(void);
void DcProbe_GetSnapshot(DcProbe_Snapshot *out);
const char *DcProbe_VerdictString(DcProbe_Verdict v);
void DcProbe_CalibrateWithKnownMv(uint32_t known_mv);

#endif /* DC_PROBE_H */