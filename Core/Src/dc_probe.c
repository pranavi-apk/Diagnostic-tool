/**
 * @file dc_probe.c
 * @brief DC voltage probe via resistor divider on ADC1 (PC0..PC5, see DC_PROBE_ADC_PIN).
 *
 * Hardware:
 *   red tip -> R1 47k -> NODE -> R2 12k (10k + 2k) -> GND
 *   C1 (2.2 nF to 100 nF) from NODE to GND
 *   NODE -> PC3 (short wire). The pin must have NO on-board pull-up or load.
 *   black lead -> board GND
 */
#include "dc_probe.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ------------------------------------------------------------------ config */

/* ADC input pin on port C. 0..5 = PC0..PC5 = ADC1_IN10..IN15.
 * Change this one number to move the probe to another pin.
 * MINI-V3 board: do NOT use PC0 (4.7k to 3V3 on board) or PC1 (VR via J10).
 * Default is PC3 = ADC1_IN13. */
#ifndef DC_PROBE_ADC_PIN
#define DC_PROBE_ADC_PIN            3U
#endif
#define DC_PROBE_ADC_CHANNEL        (10U + DC_PROBE_ADC_PIN)
#define DC_PROBE_ADC_CRL_SHIFT      (DC_PROBE_ADC_PIN * 4U)
#define DC_PROBE_ADC_SMP_SHIFT      (DC_PROBE_ADC_PIN * 3U)
#define DC_PROBE_SAMPLE_COUNT       48U
#define DC_PROBE_ADC_SMP_239CY5     7U
#define DC_PROBE_ADC_MAX_RAW        4095U
#define DC_PROBE_ADC_GUARD          20000U  /* loop passes before giving up */

/* At or below this average the tip is at 0 V, open, or reversed. ~24 mV. */
#define DC_PROBE_RAW_ZERO_MAX       6U

/* Calibration factor limits. Wide enough for 5% resistors plus VDDA error.
 * A wrong resistor value (4.7k for 47k, 100k for 10k) lands far outside. */
#define DC_PROBE_CAL_MIN            0.85f
#define DC_PROBE_CAL_MAX            1.15f
#define DC_PROBE_CAL_MIN_MV         500U

#if (DC_PROBE_ADC_PIN > 5U)
#error "DC_PROBE_ADC_PIN must be 0..5 (PC0..PC5)"
#endif

/* ------------------------------------------------------------------- state */

static ADC_TypeDef *const s_adc = ADC1;
static uint8_t s_initialized;
static float s_cal_factor = 1.0f;
static DcProbe_Snapshot s_snap;

/* --------------------------------------------------------------------- ADC */

static void dc_probe_adc_gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN;

    /* ADC pin: analog input (MODE = 00, CNF = 00) */
    GPIOC->CRL &= ~(0xFU << DC_PROBE_ADC_CRL_SHIFT);
}

/* Returns 1 when the ADC is powered and calibrated, 0 on timeout. */
static uint8_t dc_probe_adc_enable(void)
{
    uint32_t cfg;
    uint32_t guard;

    /* ADC prescaler PCLK2/6 -> 12 MHz at 72 MHz PCLK2 */
    cfg = RCC->CFGR;
    cfg &= ~RCC_CFGR_ADCPRE;
    cfg |= RCC_CFGR_ADCPRE_DIV6;
    RCC->CFGR = cfg;

    /* Single conversion, right aligned, software start.
     * On the F1, SWSTART only works with EXTTRIG set and EXTSEL = 111. */
    s_adc->CR1 = 0U;
    s_adc->CR2 = ADC_CR2_EXTSEL | ADC_CR2_EXTTRIG;
    s_adc->CR2 |= ADC_CR2_ADON;
    HAL_Delay(1);

    s_adc->CR2 |= ADC_CR2_RSTCAL;
    guard = DC_PROBE_ADC_GUARD;
    while ((s_adc->CR2 & ADC_CR2_RSTCAL) != 0U)
    {
        if (--guard == 0U)
        {
            return 0U;
        }
    }

    s_adc->CR2 |= ADC_CR2_CAL;
    guard = DC_PROBE_ADC_GUARD;
    while ((s_adc->CR2 & ADC_CR2_CAL) != 0U)
    {
        if (--guard == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

static void dc_probe_adc_config_channel(void)
{
    s_adc->SMPR1 = (s_adc->SMPR1 & ~(0x7U << DC_PROBE_ADC_SMP_SHIFT)) |
                   (DC_PROBE_ADC_SMP_239CY5 << DC_PROBE_ADC_SMP_SHIFT);
    s_adc->SQR1 = 0U;                       /* one conversion */
    s_adc->SQR3 = DC_PROBE_ADC_CHANNEL;
}

/* Returns 1 with a sample in *out, 0 if the conversion never finished. */
static uint8_t dc_probe_read_once(uint16_t *out)
{
    uint32_t guard = DC_PROBE_ADC_GUARD;

    (void)s_adc->DR;                        /* clears a stale EOC */
    s_adc->CR2 |= ADC_CR2_SWSTART;

    while ((s_adc->SR & ADC_SR_EOC) == 0U)
    {
        if (--guard == 0U)
        {
            return 0U;
        }
    }
    *out = (uint16_t)(s_adc->DR & 0xFFFU);
    return 1U;
}

/* Averages DC_PROBE_SAMPLE_COUNT conversions. Returns 0 on ADC timeout. */
static uint8_t dc_probe_sample(uint16_t *out_avg, uint16_t *out_min,
                               uint16_t *out_max)
{
    uint32_t sum = 0U;
    uint16_t raw = 0U;
    uint16_t rmin = (uint16_t)DC_PROBE_ADC_MAX_RAW;
    uint16_t rmax = 0U;
    uint8_t i;

    dc_probe_adc_config_channel();

    /* First conversion after a channel change is discarded. */
    if (dc_probe_read_once(&raw) == 0U)
    {
        return 0U;
    }

    for (i = 0U; i < DC_PROBE_SAMPLE_COUNT; i++)
    {
        if (dc_probe_read_once(&raw) == 0U)
        {
            return 0U;
        }
        sum += raw;
        if (raw < rmin)
        {
            rmin = raw;
        }
        if (raw > rmax)
        {
            rmax = raw;
        }
    }

    *out_avg = (uint16_t)(sum / DC_PROBE_SAMPLE_COUNT);
    *out_min = rmin;
    *out_max = rmax;
    return 1U;
}

/* -------------------------------------------------------------- conversion */

static float dc_probe_raw_to_mv_uncal(uint16_t raw)
{
    return ((float)raw / (float)DC_PROBE_ADC_MAX_RAW) *
           DC_PROBE_VREF_MV * DC_PROBE_DIVIDER_GAIN;
}

static uint32_t dc_probe_raw_to_mv(uint16_t raw)
{
    float vin = dc_probe_raw_to_mv_uncal(raw) * s_cal_factor;

    if (vin < 0.0f)
    {
        vin = 0.0f;
    }
    return (uint32_t)(vin + 0.5f);
}

/* Over-range is handled by the caller, from the raw maximum. */
static DcProbe_Verdict dc_probe_classify(uint16_t raw_avg, uint32_t vin_mv)
{
    if (raw_avg <= DC_PROBE_RAW_ZERO_MAX)
    {
        return DC_PROBE_OPEN;
    }
    if ((vin_mv >= 3135U) && (vin_mv <= 3465U))
    {
        return DC_PROBE_RAIL_3V3_PASS;
    }
    if ((vin_mv >= 4750U) && (vin_mv <= 5250U))
    {
        return DC_PROBE_RAIL_5V_PASS;
    }
    if ((vin_mv >= 2800U) && (vin_mv <= 3800U))
    {
        return DC_PROBE_RAIL_3V3_FAIL;
    }
    if ((vin_mv >= 4500U) && (vin_mv <= 5500U))
    {
        return DC_PROBE_RAIL_5V_FAIL;
    }
    if (vin_mv > 5500U)
    {
        return DC_PROBE_MEASURE;            /* above logic levels, e.g. 12 V */
    }
    if (vin_mv < 800U)
    {
        return DC_PROBE_LOGIC_LOW;
    }
    if (vin_mv > 2000U)
    {
        return DC_PROBE_LOGIC_HIGH;
    }
    return DC_PROBE_LOGIC_INVALID;
}

/* ------------------------------------------------------------------ public */

void DcProbe_Init(void)
{
    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.cal_factor = s_cal_factor;

    dc_probe_adc_gpio_init();

    /* If the ADC does not come up, initialized stays 0 and Tick does nothing. */
    s_initialized = dc_probe_adc_enable();
    s_snap.initialized = s_initialized;

    DcProbe_Tick();
}

void DcProbe_Tick(void)
{
    uint16_t avg;
    uint16_t rmin;
    uint16_t rmax;

    if (s_initialized == 0U)
    {
        return;
    }

    s_snap.cal_factor = s_cal_factor;
    s_snap.initialized = 1U;

    if (dc_probe_sample(&avg, &rmin, &rmax) == 0U)
    {
        /* ADC did not answer. Show nothing instead of a stale number. */
        s_snap.voltage_valid = 0U;
        s_snap.vin_mv = 0U;
        s_snap.verdict = DC_PROBE_VERDICT_NONE;
        return;
    }

    s_snap.raw_avg = avg;
    s_snap.raw_min = rmin;
    s_snap.raw_max = rmax;
    s_snap.vin_mv = dc_probe_raw_to_mv(avg);

    if (rmax >= DC_PROBE_ADC_MAX_RAW)
    {
        /* One clipped sample is enough. The number is not trustworthy. */
        s_snap.voltage_valid = 0U;
        s_snap.verdict = DC_PROBE_OVER_RANGE;
        return;
    }

    s_snap.voltage_valid = 1U;
    s_snap.verdict = dc_probe_classify(avg, s_snap.vin_mv);
}

void DcProbe_GetSnapshot(DcProbe_Snapshot *out)
{
    if (out != NULL)
    {
        *out = s_snap;
    }
}

const char *DcProbe_VerdictString(DcProbe_Verdict v)
{
    switch (v)
    {
    case DC_PROBE_OVER_RANGE:       return "OVER RANGE";
    case DC_PROBE_CHECK_POLARITY:   return "CHECK POLARITY";
    case DC_PROBE_RAIL_3V3_PASS:    return "3.3V PASS";
    case DC_PROBE_RAIL_3V3_FAIL:    return "3.3V FAIL";
    case DC_PROBE_RAIL_5V_PASS:     return "5V PASS";
    case DC_PROBE_RAIL_5V_FAIL:     return "5V FAIL";
    case DC_PROBE_LOGIC_LOW:        return "LOGIC LOW";
    case DC_PROBE_LOGIC_HIGH:       return "LOGIC HIGH";
    case DC_PROBE_LOGIC_INVALID:    return "LOGIC INVALID";
    case DC_PROBE_MEASURE:          return "DC OK";
    case DC_PROBE_OPEN:             return "0V OR OPEN";
    default:                        return "---";
    }
}

/* Single-point gain calibration against a multimeter reading.
 * On any failed check the old factor is kept. Compare cal_factor in the
 * snapshot before and after to see whether the calibration was accepted. */
void DcProbe_CalibrateWithKnownMv(uint32_t known_mv)
{
    uint16_t avg;
    uint16_t rmin;
    uint16_t rmax;
    float measured_mv;
    float factor;

    if (s_initialized == 0U)
    {
        return;
    }
    if ((known_mv < DC_PROBE_CAL_MIN_MV) ||
        (known_mv > DC_PROBE_ADC_FULL_SCALE_MV))
    {
        return;
    }
    if (dc_probe_sample(&avg, &rmin, &rmax) == 0U)
    {
        return;
    }
    if (rmax >= DC_PROBE_ADC_MAX_RAW)
    {
        return;                             /* clipped: cannot calibrate */
    }

    measured_mv = dc_probe_raw_to_mv_uncal(avg);
    if (measured_mv < (float)DC_PROBE_CAL_MIN_MV)
    {
        return;                             /* too small to be accurate */
    }

    factor = (float)known_mv / measured_mv;
    if ((factor < DC_PROBE_CAL_MIN) || (factor > DC_PROBE_CAL_MAX))
    {
        return;                             /* divider is wired wrong */
    }

    s_cal_factor = factor;
    s_snap.cal_factor = factor;
    DcProbe_Tick();
}
