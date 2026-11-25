/**
 * @file uart_cmd.c
 * @brief Text command parser for remote diagnostic control over UART.
 */
#include "uart_cmd.h"
#include "diag_iface.h"
#include "pwm_gen.h"
#include "signal_analyzer.h"
#include "i2c_scanner.h"
#include "dc_probe.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define UART_CMD_LINE_MAX  80U

static UartCmd_TxFn s_tx;
static UartCmd_PwmSetFn s_pwm_set;
static UartCmd_EnvReadFn s_env_read;
static char s_last_status[40];

static void uart_cmd_tx_line(const char *line)
{
    if ((s_tx != NULL) && (line != NULL))
    {
        s_tx(line);
        s_tx("\r\n");
    }
}

static void uart_cmd_set_status(const char *line)
{
    size_t n;

    if (line == NULL)
    {
        s_last_status[0] = '\0';
        return;
    }
    n = strlen(line);
    if (n >= sizeof(s_last_status))
    {
        n = sizeof(s_last_status) - 1U;
    }
    memcpy(s_last_status, line, n);
    s_last_status[n] = '\0';
}

static void uart_cmd_upper(char *s)
{
    while ((s != NULL) && (*s != '\0'))
    {
        if ((*s >= 'a') && (*s <= 'z'))
        {
            *s = (char)(*s - ('a' - 'A'));
        }
        s++;
    }
}

static void uart_cmd_trim(char *s)
{
    char *start;
    size_t len;

    if (s == NULL)
    {
        return;
    }
    start = s;
    while ((*start == ' ') || (*start == '\t'))
    {
        start++;
    }
    if (start != s)
    {
        memmove(s, start, strlen(start) + 1U);
    }
    len = strlen(s);
    while ((len > 0U) && ((s[len - 1U] == ' ') || (s[len - 1U] == '\t')))
    {
        s[len - 1U] = '\0';
        len--;
    }
}

static uint32_t uart_cmd_parse_u32(const char **pp)
{
    uint32_t v = 0U;
    const char *p = *pp;

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    while ((*p >= '0') && (*p <= '9'))
    {
        v = (v * 10U) + (uint32_t)(*p - '0');
        p++;
    }
    *pp = p;
    return v;
}

static const char *uart_cmd_verdict(MeasureVerdict v)
{
    switch (v)
    {
    case MEASURE_VERDICT_PASS: return "PASS";
    case MEASURE_VERDICT_FAIL: return "FAIL";
    case MEASURE_VERDICT_INVALID: return "INVALID";
    case MEASURE_VERDICT_NO_SIGNAL: return "NO_SIG";
    default: return "NONE";
    }
}

static void uart_cmd_do_help(void)
{
    uart_cmd_tx_line("OK HELP");
    uart_cmd_tx_line("  HELP");
    uart_cmd_tx_line("  STATUS");
    uart_cmd_tx_line("  GET_PWM");
    uart_cmd_tx_line("  SET_PWM <hz> <duty> [ON|OFF]");
    uart_cmd_tx_line("  PWM ON | PWM OFF");
    uart_cmd_tx_line("  GET_ANALYZER");
    uart_cmd_tx_line("  I2C SCAN");
    uart_cmd_tx_line("  GET_ENV");
    uart_cmd_tx_line("  GET_PROBE");
    uart_cmd_tx_line("  PROBE CAL <mV>");
    uart_cmd_set_status("OK HELP");
}

static void uart_cmd_do_status(void)
{
    char buf[96];
    DiagPlatform_Status st;

    DiagPlatform_GetStatus(&st);
    uart_cmd_tx_line("OK STATUS");
    snprintf(buf, sizeof(buf), " PWM REQ/ACT=%lu/%lu DUTY=%u OUT=%u",
             (unsigned long)st.pwm.requested_frequency_hz,
             (unsigned long)st.pwm.actual_frequency_hz,
             (unsigned int)st.pwm.duty_percent,
             (unsigned int)st.pwm.output_enabled);
    uart_cmd_tx_line(buf);
    snprintf(buf, sizeof(buf), " SIG HZ=%lu DUTY=%lu V=%s N=%u",
             (unsigned long)st.signal.frequency_hz_avg,
             (unsigned long)st.signal.duty_percent_avg,
             uart_cmd_verdict(st.signal.verdict),
             (unsigned int)st.signal.valid_sample_count);
    uart_cmd_tx_line(buf);
    snprintf(buf, sizeof(buf), " BME INIT=%u STATE=%u",
             (unsigned int)st.bme_initialized,
             (unsigned int)st.bme_state);
    uart_cmd_tx_line(buf);
    snprintf(buf, sizeof(buf), " STATUS OK");
    uart_cmd_set_status("OK STATUS");
}

static void uart_cmd_do_get_pwm(void)
{
    char buf[80];
    PwmGen_Config cfg;

    PwmGen_GetConfig(&cfg);
    snprintf(buf, sizeof(buf),
             "OK PWM REQ=%lu ACT=%lu DUTY=%u OUT=%u",
             (unsigned long)cfg.requested_frequency_hz,
             (unsigned long)cfg.actual_frequency_hz,
             (unsigned int)cfg.duty_percent,
             (unsigned int)cfg.output_enabled);
    uart_cmd_tx_line(buf);
    uart_cmd_set_status(buf);
}

static void uart_cmd_do_pwm_onoff(uint8_t on)
{
    char buf[48];

    if (s_pwm_set != NULL)
    {
        PwmGen_Config cfg;
        PwmGen_GetConfig(&cfg);
        s_pwm_set(cfg.requested_frequency_hz, cfg.duty_percent, on ? 1 : 0);
    }
    snprintf(buf, sizeof(buf), on ? "OK PWM ON" : "OK PWM OFF");
    uart_cmd_tx_line(buf);
    uart_cmd_set_status(buf);
}

static void uart_cmd_do_set_pwm(const char *args)
{
    uint32_t hz;
    uint32_t duty;
    int8_t on_flag = -1;
    char buf[80];
    const char *p = args;

    hz = uart_cmd_parse_u32(&p);
    duty = uart_cmd_parse_u32(&p);

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    if ((p[0] == 'O') && (p[1] == 'N') &&
        ((p[2] == '\0') || (p[2] == ' ')))
    {
        on_flag = 1;
    }
    else if ((p[0] == 'O') && (p[1] == 'F') && (p[2] == 'F'))
    {
        on_flag = 0;
    }

    if ((hz < PWM_GEN_MIN_FREQUENCY_HZ) || (hz > PWM_GEN_MAX_FREQUENCY_HZ) ||
        (duty > 100U))
    {
        uart_cmd_tx_line("ERR SET_PWM ARGS");
        uart_cmd_set_status("ERR SET_PWM");
        return;
    }

    if (s_pwm_set != NULL)
    {
        if (on_flag < 0)
        {
            PwmGen_Config cfg;
            PwmGen_GetConfig(&cfg);
            on_flag = cfg.output_enabled ? 1 : 0;
        }
        s_pwm_set(hz, (uint8_t)duty, on_flag);
    }

    {
        PwmGen_Config cfg;
        PwmGen_GetConfig(&cfg);
        snprintf(buf, sizeof(buf),
                 "OK SET_PWM REQ=%lu ACT=%lu DUTY=%u OUT=%u",
                 (unsigned long)cfg.requested_frequency_hz,
                 (unsigned long)cfg.actual_frequency_hz,
                 (unsigned int)cfg.duty_percent,
                 (unsigned int)cfg.output_enabled);
    }
    uart_cmd_tx_line(buf);
    uart_cmd_set_status("OK SET_PWM");
}

static void uart_cmd_do_analyzer(void)
{
    char buf[96];
    SignalAnalyzer_Snapshot snap;

    SignalAnalyzer_GetSnapshot(&snap);
    if (snap.signal_valid == 0U)
    {
        uart_cmd_tx_line("OK ANALYZER NO_SIGNAL");
        uart_cmd_set_status("NO_SIGNAL");
        return;
    }
    snprintf(buf, sizeof(buf),
             "OK ANALYZER HZ=%lu DUTY=%lu MIN=%lu MAX=%lu V=%s N=%u",
             (unsigned long)snap.frequency_hz_avg,
             (unsigned long)snap.duty_percent_avg,
             (unsigned long)snap.frequency_hz_min,
             (unsigned long)snap.frequency_hz_max,
             uart_cmd_verdict(snap.verdict),
             (unsigned int)snap.valid_sample_count);
    uart_cmd_tx_line(buf);
    uart_cmd_set_status("OK ANALYZER");
}

static void uart_cmd_do_i2c_scan(void)
{
    char buf[96];
    uint32_t guard;
    I2cScanner_Result res;
    uint8_t i;
    int pos;

    I2cScanner_Begin();
    guard = HAL_GetTick();
    while (I2cScanner_IsBusy() && ((HAL_GetTick() - guard) < 2500U))
    {
        I2cScanner_Tick();
    }
    I2cScanner_GetResult(&res);

    uart_cmd_tx_line("OK I2C");
    pos = snprintf(buf, sizeof(buf), " FOUND=%u",
                   (unsigned int)res.total_responding);
    for (i = 0U; i < res.address_count; i++)
    {
        if (pos > 0 && pos < (int)sizeof(buf) - 4)
        {
            pos += snprintf(buf + pos, sizeof(buf) - (size_t)pos,
                            " %02X", (unsigned int)res.addresses[i]);
        }
    }
    uart_cmd_tx_line(buf);
    snprintf(buf, sizeof(buf), " NACK=%lu TO=%lu RCV=%s BME_ID=%02X",
             (unsigned long)res.comm_stats.nack_count,
             (unsigned long)res.comm_stats.timeout_count,
             res.recovery_attempted ?
                 (res.recovery_ok ? "OK" : "FAIL") : "-",
             (unsigned int)res.bme280_chip_id);
    uart_cmd_tx_line(buf);
    uart_cmd_set_status("OK I2C SCAN");
}

static void uart_cmd_do_env(void)
{
    char buf[96];
    float t = 0.0f;
    float p = 0.0f;
    float h = 0.0f;
    int32_t t_centi;

    if ((s_env_read == NULL) || (s_env_read(&t, &p, &h) == 0U))
    {
        uart_cmd_tx_line("ERR ENV NOT_READY");
        uart_cmd_set_status("ERR ENV");
        return;
    }
    t_centi = (int32_t)(t * 100.0f);
    snprintf(buf, sizeof(buf), "OK ENV T=%ld.%02ldC P=%.0fPa H=%.1f%%",
             (long)(t_centi / 100),
             (long)(((t_centi < 0) ? -t_centi : t_centi) % 100),
             (double)p, (double)h);
    uart_cmd_tx_line(buf);
    uart_cmd_set_status("OK ENV");
}

static void uart_cmd_do_get_probe(void)
{
    char buf[96];
    DcProbe_Snapshot snap;
    int32_t whole;
    int32_t frac;

    DcProbe_GetSnapshot(&snap);
    if (snap.voltage_valid == 0U)
    {
        snprintf(buf, sizeof(buf),
                 "OK PROBE OPEN RAW=%u (%u-%u) V=%s",
                 (unsigned int)snap.raw_avg,
                 (unsigned int)snap.raw_min,
                 (unsigned int)snap.raw_max,
                 DcProbe_VerdictString(snap.verdict));
    }
    else
    {
        whole = (int32_t)(snap.vin_mv / 1000U);
        frac = (int32_t)((snap.vin_mv % 1000U) / 10U);
        snprintf(buf, sizeof(buf),
                 "OK PROBE V=%ld.%02ld RAW=%u V=%s",
                 (long)whole, (long)frac,
                 (unsigned int)snap.raw_avg,
                 DcProbe_VerdictString(snap.verdict));
    }
    uart_cmd_tx_line(buf);
    uart_cmd_set_status("OK PROBE");
}

static void uart_cmd_do_probe_cal(const char *args)
{
    const char *p = args;
    uint32_t mv;
    char buf[48];

    if ((args == NULL) || (strncmp(args, "CAL ", 4) != 0))
    {
        uart_cmd_tx_line("ERR PROBE CAL");
        uart_cmd_set_status("ERR PROBE");
        return;
    }
    p += 4;
    mv = uart_cmd_parse_u32(&p);
    if (mv < 500U)
    {
        uart_cmd_tx_line("ERR PROBE CAL MV");
        uart_cmd_set_status("ERR PROBE");
        return;
    }
    DcProbe_CalibrateWithKnownMv(mv);
    snprintf(buf, sizeof(buf), "OK PROBE CAL %lu", (unsigned long)mv);
    uart_cmd_tx_line(buf);
    uart_cmd_set_status("OK PROBE CAL");
}

void UartCmd_Init(UartCmd_TxFn tx, UartCmd_PwmSetFn pwm_set,
                  UartCmd_EnvReadFn env_read)
{
    s_tx = tx;
    s_pwm_set = pwm_set;
    s_env_read = env_read;
    s_last_status[0] = '\0';
}

void UartCmd_HandleLine(const char *line)
{
    char work[UART_CMD_LINE_MAX];
    char *verb;
    char *args;
    size_t n;

    if (line == NULL)
    {
        return;
    }
    n = strlen(line);
    if (n >= sizeof(work))
    {
        n = sizeof(work) - 1U;
    }
    memcpy(work, line, n);
    work[n] = '\0';
    uart_cmd_trim(work);
    if (work[0] == '\0')
    {
        return;
    }
    uart_cmd_upper(work);

    verb = work;
    args = strchr(work, ' ');
    if (args != NULL)
    {
        *args = '\0';
        args++;
        while ((*args == ' ') || (*args == '\t'))
        {
            args++;
        }
    }

    if ((strcmp(verb, "HELP") == 0) || (strcmp(verb, "?") == 0))
    {
        uart_cmd_do_help();
    }
    else if (strcmp(verb, "STATUS") == 0)
    {
        uart_cmd_do_status();
    }
    else if ((strcmp(verb, "GET_PWM") == 0) || (strcmp(verb, "PWM?") == 0))
    {
        uart_cmd_do_get_pwm();
    }
    else if (strcmp(verb, "SET_PWM") == 0)
    {
        uart_cmd_do_set_pwm((args != NULL) ? args : "");
    }
    else if (strcmp(verb, "PWM") == 0)
    {
        if (args == NULL)
        {
            uart_cmd_do_get_pwm();
        }
        else if (strcmp(args, "ON") == 0)
        {
            uart_cmd_do_pwm_onoff(1U);
        }
        else if (strcmp(args, "OFF") == 0)
        {
            uart_cmd_do_pwm_onoff(0U);
        }
        else
        {
            uart_cmd_do_set_pwm(args);
        }
    }
    else if ((strcmp(verb, "GET_ANALYZER") == 0) ||
             (strcmp(verb, "ANALYZE?") == 0))
    {
        uart_cmd_do_analyzer();
    }
    else if ((strcmp(verb, "I2C") == 0) && (args != NULL) &&
             (strcmp(args, "SCAN") == 0))
    {
        uart_cmd_do_i2c_scan();
    }
    else if ((strcmp(verb, "GET_ENV") == 0) || (strcmp(verb, "ENV") == 0) ||
             (strcmp(verb, "ENV?") == 0))
    {
        uart_cmd_do_env();
    }
    else if ((strcmp(verb, "GET_PROBE") == 0) || (strcmp(verb, "PROBE?") == 0))
    {
        uart_cmd_do_get_probe();
    }
    else if (strcmp(verb, "PROBE") == 0)
    {
        if ((args != NULL) && (strncmp(args, "CAL ", 4) == 0))
        {
            uart_cmd_do_probe_cal(args);
        }
        else
        {
            uart_cmd_do_get_probe();
        }
    }
    else
    {
        uart_cmd_tx_line("ERR UNKNOWN (HELP)");
        uart_cmd_set_status("ERR UNKNOWN");
    }
}

const char *UartCmd_LastStatusLine(void)
{
    return s_last_status;
}