#include "main.h"
#include "gpio.h"
#include "fsmc.h"
#include "lcd.h"
#include "pwm_gen.h"
#include "signal_analyzer.h"
#include "sw_i2c.h"
#include "i2c_scanner.h"
#include "diag_iface.h"
#include <stdint.h>
#include <stdio.h>
/* =========================================================
   ROTARY ENCODER VARIABLES
   ========================================================= */
volatile int32_t encoder_count = 0;
volatile uint32_t encoder_press_count = 0;
volatile uint8_t encoder_button_pressed = 0;
static uint8_t encoder_last_state = 0;
static int8_t encoder_step_count = 0;
static GPIO_PinState button_last = GPIO_PIN_SET;
static GPIO_PinState button_stable = GPIO_PIN_SET;
static uint32_t button_time = 0;
static uint32_t button_pressed_at = 0;
static uint8_t encoder_short_press_event = 0;
static uint8_t encoder_long_press_event = 0;
static uint8_t long_press_reported = 0;

static uint8_t bme280_initialized = 0U;
static uint32_t bme280_last_refresh = 0U;
static uint32_t analyzer_last_refresh = 0U;
static uint32_t analyzer_ui_last_hz = 0xFFFFFFFFUL;
static uint32_t analyzer_ui_last_duty = 0xFFFFFFFFUL;
static uint8_t analyzer_ui_last_verdict = 0xFFU;
static uint8_t analyzer_ui_last_have = 0xFFU;

#define ANALYZER_UI_HZ_DEADBAND_HZ    2U
#define ANALYZER_UI_DUTY_DEADBAND_PP  2U

static uint8_t Analyzer_UiValuesChanged(uint8_t have,
                                        uint32_t hz,
                                        uint32_t duty,
                                        MeasureVerdict verdict)
{
    uint32_t dh;
    uint32_t dd;

    if (have != analyzer_ui_last_have)
    {
        return 1U;
    }
    if ((uint8_t)verdict != analyzer_ui_last_verdict)
    {
        return 1U;
    }
    if (analyzer_ui_last_hz == 0xFFFFFFFFUL)
    {
        return 1U;
    }

    dh = (hz >= analyzer_ui_last_hz) ?
         (hz - analyzer_ui_last_hz) : (analyzer_ui_last_hz - hz);
    dd = (duty >= analyzer_ui_last_duty) ?
         (duty - analyzer_ui_last_duty) : (analyzer_ui_last_duty - duty);
    if ((dh > ANALYZER_UI_HZ_DEADBAND_HZ) || (dd > ANALYZER_UI_DUTY_DEADBAND_PP))
    {
        return 1U;
    }
    return 0U;
}

/* USART1 UART terminal: PA9 TX, PA10 RX, 115200 baud */
static char uart_rx_line[32];
static uint8_t uart_rx_length = 0U;
static uint8_t uart_line_ready = 0U;
static uint8_t uart_display_dirty = 0U;
static void UART_Init(void);
static void UART_SendString(const char *text);
static void UART_Process(void);
static void LCD_Show_UART_Terminal(void);
static void LCD_Update_UART_Terminal(void);
static void LCD_Update_UART_RX_Line(void);

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4, dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;
} BME280_Calibration;

static BME280_Calibration bme280_cal;
static uint8_t BME280_Init_SoftwareI2C(void);
static uint8_t BME280_ReadMeasurements(float *temperature,
                                       float *pressure,
                                       float *humidity);
static void LCD_Show_Environment_Screen(void);
static void LCD_Update_Environment_Screen(void);
/* =========================================================
   MENU VARIABLES
   ========================================================= */
#define MENU_ITEMS 6
/* Encoder order follows the dashboard from left to right, top to bottom. */
static const char *menu_options[MENU_ITEMS] =
{
    "PWM GENERATOR",
    "SIGNAL ANALYZER",
    "I2C SCANNER",
    "ENV MONITOR",
    "UART TERMINAL",
    "SYSTEM INFO"
};
static int8_t menu_selection = 0;
static int32_t previous_encoder_count = 0;
static uint8_t screen_mode = 0;
/* 0 = Main menu
   1 = Selected module */
/* =========================================================
   PWM VARIABLES
   TIM8 CHANNEL 1 - PC6
   ========================================================= */
static uint8_t pwm_duty = 50U;
static uint8_t pwm_enabled = 0U;
static uint32_t pwm_frequency_hz = 1000U;
static uint8_t pwm_focus = 0U;       /* 0=output, 1=frequency, 2=duty */
static uint8_t pwm_editing = 0U;
#define PWM_MIN_FREQUENCY_HZ  PWM_GEN_MIN_FREQUENCY_HZ
#define PWM_MAX_FREQUENCY_HZ  PWM_GEN_MAX_FREQUENCY_HZ
#define PWM_FREQUENCY_STEP_HZ PWM_GEN_FREQUENCY_STEP_HZ
/* =========================================================
   FUNCTION PROTOTYPES
   ========================================================= */
void SystemClock_Config(void);
void Error_Handler(void);
static void Encoder_Init(void);
static void Encoder_Update(void);
static void LCD_GPIO_Init(void);
static void PWM_Init(void);
static void PWM_Start(void);
static void PWM_Stop(void);
static void PWM_Update_Duty(void);
static void PWM_Update_Frequency(void);
static void PWM_Draw_Waveform(void);
static void PWM_Draw_Settings(void);
static void UART_Init(void);
static void UART_SendString(const char *text);
static void UART_Process(void);
static void LCD_Show_UART_Terminal(void);
static void LCD_Update_UART_Terminal(void);
static void LCD_Show_Signal_Analyzer(void);
static void LCD_Update_Signal_Analyzer(void);
static void LCD_Show_Main_Menu(void);
static void Dashboard_DrawCard(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               uint16_t accent, const char *title,
                               const char *value, const char *detail,
                               uint8_t selected);
static void Dashboard_DrawMain(void);
static void LCD_Update_Menu_Selection(int8_t old_selection,
                                      int8_t new_selection);
static void LCD_Show_Selected_Module(void);
static void LCD_Show_PWM_Screen(void);
static void LCD_Update_PWM_Display(void);
static void LCD_Show_I2C_Scanner(void);
static void LCD_Handle_Menu(void);
static void Screen_Begin(const char *title);
static void Screen_Footer(void);
static void Dashboard_DrawOneSelection(int8_t selection, uint8_t selected);
/* =========================================================
   MAIN FUNCTION
   ========================================================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    LCD_GPIO_Init();
    MX_FSMC_Init();
    LCD_INIT();
    Encoder_Init();
    PWM_Init();
    SignalAnalyzer_Init();
    SwI2c_InitPins();
    UART_Init();
    previous_encoder_count = encoder_count;
    LCD_Show_Main_Menu();
    while (1)
    {
        Encoder_Update();
        UART_Process();
        SignalAnalyzer_Tick();
        if (I2cScanner_IsBusy())
        {
            I2cScanner_Tick();
        }
        LCD_Handle_Menu();
        HAL_Delay(1);
    }
}
/* =========================================================
   LCD GPIO INITIALIZATION
   ========================================================= */
static void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /* LCD reset: PE1 */
    GPIO_InitStruct.Pin = LCD_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_RST_PORT, &GPIO_InitStruct);
    /* LCD backlight: PD12 */
    GPIO_InitStruct.Pin = LCD_BK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_BK_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_BK_PORT, LCD_BK_PIN, GPIO_PIN_SET);
}
/* =========================================================
   ENCODER INITIALIZATION
   ========================================================= */
static void Encoder_Init(void)
{
    uint8_t clk;
    uint8_t dt;
    clk = (uint8_t)HAL_GPIO_ReadPin(
        ENC_CLK_GPIO_Port,
        ENC_CLK_Pin
    );
    dt = (uint8_t)HAL_GPIO_ReadPin(
        ENC_DT_GPIO_Port,
        ENC_DT_Pin
    );
    encoder_last_state = (clk << 1) | dt;
    button_last = HAL_GPIO_ReadPin(
        ENC_SW_GPIO_Port,
        ENC_SW_Pin
    );
    button_stable = button_last;
    button_time = HAL_GetTick();
    encoder_count = 0;
    encoder_press_count = 0;
    encoder_button_pressed = 0;
    encoder_step_count = 0;
    encoder_short_press_event = 0;
    encoder_long_press_event = 0;
    long_press_reported = 0;
}
/* =========================================================
   ENCODER UPDATE
   ========================================================= */
static void Encoder_Update(void)
{
    uint8_t clk;
    uint8_t dt;
    uint8_t current_state;
    uint8_t transition;
    GPIO_PinState button_read;
    uint32_t current_time;
    static const int8_t encoder_table[16] =
    {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    clk = (uint8_t)HAL_GPIO_ReadPin(
        ENC_CLK_GPIO_Port,
        ENC_CLK_Pin
    );
    dt = (uint8_t)HAL_GPIO_ReadPin(
        ENC_DT_GPIO_Port,
        ENC_DT_Pin
    );
    current_state = (clk << 1) | dt;
    transition = (encoder_last_state << 2) | current_state;
    encoder_step_count += encoder_table[transition];
    encoder_last_state = current_state;
    if (encoder_step_count >= 4)
    {
        encoder_count++;
        encoder_step_count = 0;
    }
    else if (encoder_step_count <= -4)
    {
        encoder_count--;
        encoder_step_count = 0;
    }
    /* Button reading and debounce */
    button_read = HAL_GPIO_ReadPin(
        ENC_SW_GPIO_Port,
        ENC_SW_Pin
    );
    current_time = HAL_GetTick();
    if (button_read != button_last)
    {
        button_last = button_read;
        button_time = current_time;
    }
    if ((current_time - button_time) >= 30)
    {
        if (button_read != button_stable)
        {
            button_stable = button_read;
            if (button_stable == GPIO_PIN_RESET)
            {
                encoder_button_pressed = 1;
                button_pressed_at = current_time;
                long_press_reported = 0;
            }
            else
            {
                encoder_button_pressed = 0;
                if (!long_press_reported)
                {
                    encoder_short_press_event = 1;
                    encoder_press_count++;
                }
            }
        }
    }
    /* Detect long press: 1 second */
    if ((button_stable == GPIO_PIN_RESET) &&
        (!long_press_reported))
    {
        if ((current_time - button_pressed_at) >= 1000)
        {
            encoder_long_press_event = 1;
            long_press_reported = 1;
        }
    }
}
/* =========================================================
   PWM GENERATOR (module: pwm_gen.c, TIM8 CH1 PC6)
   ========================================================= */
static void PWM_Init(void)
{
    PwmGen_Init();
    pwm_duty = PwmGen_GetDutyPercent();
    pwm_frequency_hz = PwmGen_GetFrequencyHz();
    pwm_enabled = PwmGen_IsEnabled();
}

static void PWM_Start(void)
{
    PwmGen_SetFrequencyHz(pwm_frequency_hz);
    PwmGen_SetDutyPercent(pwm_duty);
    PwmGen_Start();
    pwm_enabled = PwmGen_IsEnabled();
}

static void PWM_Stop(void)
{
    PwmGen_Stop();
    pwm_enabled = PwmGen_IsEnabled();
}

static void PWM_Update_Duty(void)
{
    PwmGen_SetDutyPercent(pwm_duty);
    pwm_enabled = PwmGen_IsEnabled();
}

static void PWM_Update_Frequency(void)
{
    PwmGen_SetFrequencyHz(pwm_frequency_hz);
    pwm_frequency_hz = PwmGen_GetFrequencyHz();
    pwm_enabled = PwmGen_IsEnabled();
}

/* Signal analyzer hardware + multi-cycle engine: signal_analyzer.c */

static void Screen_Begin(const char *title)
{
    const uint16_t bg = 0x0862U;
    const uint16_t panel = 0x10E5U;
    LCD_Clear(0, 0, 320, 240, bg);
    LCD_Clear(8, 36, 304, 178, panel);
    LCD_DrawLine(8, 30, 312, 30, 0x2945U);
    LCD_SetTextColors(0xE71CU, bg);
    LCD_DrawString(12, 8, title);
    LCD_SetTextColors(0xE71CU, panel);
}

static void Screen_Footer(void)
{
    LCD_DrawLine(8, 218, 312, 218, 0x2945U);
    LCD_SetTextColors(0x9CF3U, 0x0862U);
    LCD_DrawString(12, 222, "HOLD: BACK");
    LCD_SetTextColors(0xE71CU, 0x10E5U);
}

static const char *Analyzer_VerdictText(MeasureVerdict v)
{
    switch (v)
    {
    case MEASURE_VERDICT_PASS: return "PASS";
    case MEASURE_VERDICT_FAIL: return "FAIL";
    case MEASURE_VERDICT_INVALID: return "INVALID";
    case MEASURE_VERDICT_NO_SIGNAL: return "NO SIGNAL";
    default: return "NONE";
    }
}

/* Draw N square-wave cycles from measured duty. Title band is reserved. */
static void Analyzer_DrawSquareCycles(uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t h,
                                      const char *title,
                                      uint16_t wave_color,
                                      uint32_t duty_percent,
                                      uint8_t cycles,
                                      uint8_t have_signal)
{
    const uint16_t bg = 0x0862U;
    const uint16_t frame = 0x2945U;
    const uint16_t title_h = 18U; /* keep wave below "WAVEFORM" / "FREQ TREND" */
    uint16_t plot_y = (uint16_t)(y + title_h);
    uint16_t plot_h = (uint16_t)(h - title_h);
    uint16_t y_high = (uint16_t)(plot_y + 4U);
    uint16_t y_low = (uint16_t)(plot_y + plot_h - 6U);
    uint16_t x0 = (uint16_t)(x + 6U);
    uint16_t x1 = (uint16_t)(x + w - 7U);
    uint16_t usable;
    uint16_t period_w;
    uint16_t high_w;
    uint8_t c;
    uint16_t cx;

    if (cycles == 0U)
    {
        cycles = 1U;
    }

    LCD_Clear(x, y, w, h, bg);
    LCD_DrawLine(x, y, (uint16_t)(x + w - 1U), y, frame);
    LCD_DrawLine(x, (uint16_t)(y + h - 1U), (uint16_t)(x + w - 1U),
                 (uint16_t)(y + h - 1U), frame);
    LCD_DrawLine(x, y, x, (uint16_t)(y + h - 1U), frame);
    LCD_DrawLine((uint16_t)(x + w - 1U), y, (uint16_t)(x + w - 1U),
                 (uint16_t)(y + h - 1U), frame);

    LCD_SetTextColors(0x9CF3U, bg);
    LCD_DrawString((uint16_t)(x + 6U), (uint16_t)(y + 2U), title);

    if ((plot_h < 12U) || (x1 <= x0))
    {
        return;
    }

    usable = (uint16_t)(x1 - x0);
    period_w = (uint16_t)(usable / cycles);
    if (period_w < 4U)
    {
        period_w = 4U;
        cycles = (uint8_t)(usable / period_w);
        if (cycles < 1U)
        {
            cycles = 1U;
        }
    }

    if (have_signal == 0U)
    {
        LCD_DrawLine(x0, y_low, x1, y_low, 0x52AAU);
        return;
    }

    if (duty_percent == 0U)
    {
        LCD_DrawLine(x0, y_low, x1, y_low, wave_color);
        return;
    }
    if (duty_percent >= 100U)
    {
        LCD_DrawLine(x0, y_high, x1, y_high, wave_color);
        return;
    }

    high_w = (uint16_t)(((uint32_t)period_w * duty_percent) / 100U);
    if (high_w < 1U)
    {
        high_w = 1U;
    }
    if (high_w >= period_w)
    {
        high_w = (uint16_t)(period_w - 1U);
    }

    cx = x0;
    for (c = 0U; c < cycles; c++)
    {
        uint16_t edge = (uint16_t)(cx + high_w);
        uint16_t next = (uint16_t)(cx + period_w);

        if (next > x1)
        {
            next = x1;
        }
        if (edge > next)
        {
            edge = next;
        }

        /* Rising edge → high → falling → low (one period). */
        LCD_DrawLine(cx, y_low, cx, y_high, wave_color);
        LCD_DrawLine(cx, y_high, edge, y_high, wave_color);
        LCD_DrawLine(edge, y_high, edge, y_low, wave_color);
        LCD_DrawLine(edge, y_low, next, y_low, wave_color);

        cx = next;
        if (cx >= x1)
        {
            break;
        }
    }
}

static void Analyzer_DrawWaveform(uint16_t x, uint16_t y,
                                  uint16_t w, uint16_t h,
                                  uint32_t duty_percent,
                                  uint8_t have_signal)
{
    Analyzer_DrawSquareCycles(x, y, w, h, "WAVEFORM", 0x07FFU,
                              duty_percent, 1U, have_signal);
}

/* Multi-cycle square wave (min 6 periods) so peak and base are clear. */
static void Analyzer_DrawFreqTrend(uint16_t x, uint16_t y,
                                   uint16_t w, uint16_t h,
                                   uint32_t duty_percent,
                                   uint8_t have_signal)
{
    Analyzer_DrawSquareCycles(x, y, w, h, "FREQ TREND", 0x07E0U,
                              duty_percent, 6U, have_signal);
}

static void LCD_Show_Signal_Analyzer(void)
{
    analyzer_ui_last_hz = 0xFFFFFFFFUL;
    analyzer_ui_last_duty = 0xFFFFFFFFUL;
    analyzer_ui_last_verdict = 0xFFU;
    analyzer_ui_last_have = 0xFFU;
    Screen_Begin("SIGNAL ANALYZER");
    LCD_Update_Signal_Analyzer();
    Screen_Footer();
    analyzer_last_refresh = HAL_GetTick();
    screen_mode = 1U;
}

static void LCD_Update_Signal_Analyzer(void)
{
    char text[32];
    SignalAnalyzer_Snapshot snap;
    PwmGen_Config pwm;
    SignalAnalyzer_Limits lim;
    uint8_t have;

    SignalAnalyzer_GetSnapshot(&snap);
    PwmGen_GetConfig(&pwm);

    lim.target_frequency_hz = pwm.output_enabled ? pwm.actual_frequency_hz : 1000U;
    lim.frequency_tolerance_ppm = 20000U;
    lim.target_duty_percent = pwm.output_enabled ? pwm.duty_percent : 50U;
    lim.duty_tolerance_percent = 5U;
    SignalAnalyzer_SetLimits(&lim);
    SignalAnalyzer_GetSnapshot(&snap);

    have = (uint8_t)((snap.signal_valid != 0U) && (snap.valid_sample_count != 0U));

    /* Skip redraw when Hz/duty wobble by 1–2 units (new sample every period). */
    if (Analyzer_UiValuesChanged(have, snap.frequency_hz_avg,
                               snap.duty_percent_avg, snap.verdict) == 0U)
    {
        return;
    }
    analyzer_ui_last_have = have;
    analyzer_ui_last_hz = snap.frequency_hz_avg;
    analyzer_ui_last_duty = snap.duty_percent_avg;
    analyzer_ui_last_verdict = (uint8_t)snap.verdict;

    LCD_Clear(12, 40, 296, 18, 0x10E5U);
    LCD_SetTextColors(0xE71CU, 0x10E5U);
    if (have == 0U)
    {
        snprintf(text, sizeof(text), "PA6  -- Hz  -- %%  %s",
                 Analyzer_VerdictText(MEASURE_VERDICT_NO_SIGNAL));
    }
    else
    {
        snprintf(text, sizeof(text), "PA6  %lu Hz  %lu %%  %s",
                 (unsigned long)snap.frequency_hz_avg,
                 (unsigned long)snap.duty_percent_avg,
                 Analyzer_VerdictText(snap.verdict));
    }
    LCD_DrawString(16, 42, text);

    Analyzer_DrawWaveform(12, 62, 296, 54,
                          have ? snap.duty_percent_avg : 0U, have);

    Analyzer_DrawFreqTrend(12, 122, 296, 88,
                           have ? snap.duty_percent_avg : 0U, have);
}


/* =========================================================
   UART TERMINAL - USART1
   PA9 = TX, PA10 = RX, 115200 baud, 8-N-1
   CH340 USB-serial connection is routed through board jumper J17.
   ========================================================= */
static void UART_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PA9: alternate-function push-pull output. */
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (0xBU << 4);

    /* PA10: floating input. */
    GPIOA->CRH &= ~(0xFU << 8);
    GPIOA->CRH |=  (0x4U << 8);

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;       /* 1 stop bit */
    USART1->CR3 = 0U;
    USART1->BRR = 625U;     /* 72 MHz PCLK2 / 115200 baud */
    USART1->CR1 = (1U << 13) | (1U << 3) | (1U << 2); /* UE, TE, RE */
}

static void UART_SendString(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    while (*text != '\0')
    {
        while ((USART1->SR & (1U << 7)) == 0U) { } /* Wait TXE */
        USART1->DR = (uint8_t)(*text++);
    }
}

static void LCD_Update_UART_RX_Line(void)
{
    char line[32];
    snprintf(line, sizeof(line), "%-29.29s", uart_rx_line);
    LCD_SetTextColors(0xE71CU, 0x10E5U);
    LCD_DrawString(16, 156, line);
}

static void LCD_Update_UART_Terminal(void)
{
    char line[32];

    LCD_SetTextColors(0xE71CU, 0x10E5U);
    LCD_DrawString(16, 52, "BAUD: 115200  8-N-1");
    LCD_DrawString(16, 84, "USB SERIAL: READY");
    LCD_DrawString(16, 124, "LAST RX:");
    snprintf(line, sizeof(line), "%-29.29s", uart_rx_line);
    LCD_DrawString(16, 156, line);
    LCD_DrawString(16, 188, "TYPE IN MAC TERMINAL");
}

static void LCD_Show_UART_Terminal(void)
{
    Screen_Begin("UART TERMINAL");
    uart_rx_line[0] = '\0';
    uart_rx_length = 0U;
    uart_line_ready = 0U;
    LCD_Update_UART_Terminal();
    UART_SendString("\r\nSTM32 UART READY - 115200 baud\r\n");
    UART_SendString("Type a message and press Enter.\r\n> ");
    screen_mode = 1U;
}

static void UART_Process(void)
{
    while ((USART1->SR & (1U << 5)) != 0U) /* RXNE */
    {
        char ch = (char)(USART1->DR & 0xFFU);

        /* Echo received characters back to the Mac terminal. */
        while ((USART1->SR & (1U << 7)) == 0U) { }
        USART1->DR = (uint8_t)ch;

        if ((ch == '\r') || (ch == '\n'))
        {
            if (uart_rx_length > 0U)
            {
                uart_rx_line[uart_rx_length] = '\0';
                uart_line_ready = 1U;
                uart_display_dirty = 1U;
                uart_rx_length = 0U;
                UART_SendString("\r\nSTM32 received your message.\r\n> ");
            }
        }
        else if ((ch == '\b') || (ch == 127))
        {
            if (uart_rx_length > 0U)
            {
                uart_rx_length--;
                uart_rx_line[uart_rx_length] = '\0';
                uart_display_dirty = 1U;
            }
        }
        else if ((ch >= 32) && (ch <= 126) &&
                 (uart_rx_length < (sizeof(uart_rx_line) - 1U)))
        {
            uart_rx_line[uart_rx_length++] = ch;
            uart_rx_line[uart_rx_length] = '\0';
            uart_display_dirty = 1U;
        }
    }

    if ((screen_mode == 1U) && (menu_selection == 4) && uart_display_dirty)
    {
        LCD_Update_UART_RX_Line();
        uart_display_dirty = 0U;
    }
}

/* =========================================================
   DRAW MAIN MENU
   ========================================================= */
static void Dashboard_DrawCard(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               uint16_t accent, const char *title,
                               const char *value, const char *detail,
                               uint8_t selected)
{
    uint16_t border = selected ? 0xFD20U : 0x2945U;
    uint16_t card_bg = selected ? 0x1928U : 0x10E5U;

    LCD_Clear(x, y, w, h, card_bg);
    LCD_DrawLine(x, y, x + w - 1U, y, border);
    LCD_DrawLine(x, y + h - 1U, x + w - 1U, y + h - 1U, border);
    LCD_DrawLine(x, y, x, y + h - 1U, border);
    LCD_DrawLine(x + w - 1U, y, x + w - 1U, y + h - 1U, border);

    LCD_SetTextColors(accent, card_bg);
    LCD_DrawString(x + 7U, y + 6U, title);
    LCD_SetTextColors(0xE71CU, card_bg);
    LCD_DrawString(x + 7U, y + 32U, value);
    LCD_SetTextColors(0x9CF3U, card_bg);
    LCD_DrawString(x + 7U, y + 56U, detail);
}

static void Dashboard_DrawMain(void)
{
    char pwm_value[16];
    char signal_value[16];
    char env_value[16];
    uint16_t c_pwm = 0xFD20U;   /* amber */
    uint16_t c_signal = 0x07FFU;/* cyan */
    uint16_t c_i2c = 0xA75FU;   /* violet */
    uint16_t c_env = 0x07F0U;   /* green */
    uint16_t c_uart = 0xF81FU;  /* magenta */
    uint16_t c_sys = 0x9CF3U;   /* muted blue-grey */

    snprintf(pwm_value, sizeof(pwm_value), "%u%% DUTY", (unsigned int)pwm_duty);
    if (Signal_Detected && Signal_Frequency_Hz > 0U)
        snprintf(signal_value, sizeof(signal_value), "%lu HZ", (unsigned long)Signal_Frequency_Hz);
    else
        snprintf(signal_value, sizeof(signal_value), "NO SIGNAL");
    snprintf(env_value, sizeof(env_value), "%s", bme280_initialized ? "BME OK" : "BME CHECK");

    LCD_Clear(0, 0, 320, 240, 0x0862U);

    /* Header */
    LCD_Clear(0, 0, 320, 34, 0x0862U);
    LCD_SetTextColors(0xE71CU, 0x0862U);
    LCD_DrawString(8, 8, "DIAGNOSTIC PLATFORM");
    LCD_SetTextColors(pwm_enabled ? c_env : c_pwm, 0x0862U);
    LCD_DrawString(224, 8, pwm_enabled ? "PWM ON" : "PWM OFF");

    /* Two rows of three compact cards, sized for 320 x 240. */
    Dashboard_DrawCard(8,   42, 96, 78, c_pwm,    "PWM GEN",   pwm_value,    "TIM8 PC6",   menu_selection == 0);
    Dashboard_DrawCard(112, 42, 96, 78, c_signal, "SIGNAL",    signal_value,  "PA6 TIM3",   menu_selection == 1);
    Dashboard_DrawCard(216, 42, 96, 78, c_i2c,    "I2C SCAN",  "2 FOUND",     "50 / 76",    menu_selection == 2);
    Dashboard_DrawCard(8,   128,96, 78, c_env,    "ENV",       env_value,     "BME280",     menu_selection == 3);
    Dashboard_DrawCard(112, 128,96, 78, c_uart,   "UART",      "115200",      "8-N-1",      menu_selection == 4);
    Dashboard_DrawCard(216, 128,96, 78, c_sys,    "SYSTEM",    "72 MHZ",      "STM32F103",  menu_selection == 5);

    /* Footer */
    LCD_DrawLine(8, 216, 312, 216, 0x2945U);
    LCD_SetTextColors(0x9CF3U, 0x0862U);
    LCD_DrawString(8, 220, "TURN: SELECT");
    LCD_DrawString(176, 220, "PRESS: OPEN");
}

static void LCD_Show_Main_Menu(void)
{
    Dashboard_DrawMain();
    screen_mode = 0U;
}

static void Dashboard_DrawOneSelection(int8_t selection, uint8_t selected)
{
    uint16_t x, y, accent;
    char value[16];
    const char *title;
    const char *detail;

    /* Selection indices match the visible clockwise encoder order. */
    switch (selection)
    {
        case 0: x=8; y=42; accent=0xFD20U; title="PWM GEN";
            snprintf(value,sizeof(value),"%u%% DUTY",(unsigned int)pwm_duty); detail="TIM8 PC6"; break;
        case 1: x=112; y=42; accent=0x07FFU; title="SIGNAL";
            if (Signal_Detected && Signal_Frequency_Hz>0U)
                snprintf(value,sizeof(value),"%lu HZ",(unsigned long)Signal_Frequency_Hz);
            else
                snprintf(value,sizeof(value),"NO SIGNAL");
            detail="PA6 TIM3"; break;
        case 2: x=216; y=42; accent=0xA75FU; title="I2C SCAN";
            snprintf(value,sizeof(value),"2 FOUND"); detail="50 / 76"; break;
        case 3: x=8; y=128; accent=0x07F0U; title="ENV";
            snprintf(value,sizeof(value),"%s",bme280_initialized?"BME OK":"BME CHECK"); detail="BME280"; break;
        case 4: x=112; y=128; accent=0xF81FU; title="UART";
            snprintf(value,sizeof(value),"115200"); detail="8-N-1"; break;
        case 5: x=216; y=128; accent=0x9CF3U; title="SYSTEM";
            snprintf(value,sizeof(value),"72 MHZ"); detail="STM32F103"; break;
        default: return;
    }
    Dashboard_DrawCard(x,y,96,78,accent,title,value,detail,selected);
}

static void LCD_Update_Menu_Selection(int8_t old_selection, int8_t new_selection)
{
    Dashboard_DrawOneSelection(old_selection, 0U);
    Dashboard_DrawOneSelection(new_selection, 1U);
}
/* =========================================================
   DRAW PWM SCREEN
   ========================================================= */
static void LCD_Show_PWM_Screen(void)
{
    pwm_focus = 0U;
    pwm_editing = 0U;
    Screen_Begin("PWM GENERATOR");
    PWM_Draw_Settings();
    Screen_Footer();
    screen_mode = 1U;
}

static void PWM_Draw_Settings(void)
{
    const uint16_t ys[3] = {62U, 101U, 140U};
    const char *names[3] = {"OUTPUT", "FREQUENCY", "DUTY CYCLE"};
    char value[24];
    uint8_t i;
    for (i = 0U; i < 3U; i++)
    {
        uint16_t bg = (i == pwm_focus) ? 0x1928U : 0x10E5U;
        uint16_t fg = (i == pwm_focus) ? 0xFFE0U : 0xE71CU;
        LCD_Clear(16U, ys[i], 288U, 30U, bg);
        LCD_SetTextColors(fg, bg);
        LCD_DrawString(25U, ys[i] + 7U, names[i]);
        if (i == 0U)
            snprintf(value, sizeof(value), "%s", pwm_enabled ? "ON" : "OFF");
        else if (i == 1U)
        {
            PwmGen_Config cfg;
            PwmGen_GetConfig(&cfg);
            snprintf(value, sizeof(value), "%lu/%lu",
                     (unsigned long)cfg.requested_frequency_hz,
                     (unsigned long)cfg.actual_frequency_hz);
        }
        else
            snprintf(value, sizeof(value), "%u %%", (unsigned int)pwm_duty);
        LCD_DrawString(205U, ys[i] + 7U, value);
        if ((i == pwm_focus) && pwm_editing)
            LCD_DrawString(286U, ys[i] + 7U, "*");
    }
}

static void PWM_Draw_Waveform(void)
{
    uint16_t x0 = 28U, x1 = 292U, yTop = 199U, yLow = 211U, yHigh = 191U;
    uint16_t highWidth = (uint16_t)(((uint32_t)(x1 - x0) * pwm_duty) / 100U);
    uint16_t edge = (uint16_t)(x0 + highWidth);
    LCD_Clear(16U, 188U, 288U, 28U, 0x10E5U);
    LCD_SetTextColors(0x9CF3U, 0x10E5U);
    LCD_DrawString(20U, 188U, "WAVEFORM");
    if (!pwm_enabled || pwm_duty == 0U)
    {
        LCD_DrawLine(x0, yLow, x1, yLow, 0x07FFU);
    }
    else if (pwm_duty >= 100U)
    {
        LCD_DrawLine(x0, yHigh, x1, yHigh, 0x07FFU);
    }
    else
    {
        LCD_DrawLine(x0, yLow, x0, yHigh, 0x07FFU);
        LCD_DrawLine(x0, yHigh, edge, yHigh, 0x07FFU);
        LCD_DrawLine(edge, yHigh, edge, yLow, 0x07FFU);
        LCD_DrawLine(edge, yLow, x1, yLow, 0x07FFU);
    }
}

static void LCD_Update_PWM_Display(void)
{
    PWM_Draw_Settings();
}
/* =========================================================
   I2C / BME280 — software bus via sw_i2c + i2c_scanner modules
   PB10 = SCL, PB11 = SDA
   ========================================================= */
static uint8_t SW_BME280_ReadRegister(uint8_t reg, uint8_t *value)
{
    return (SwI2c_ReadRegister(0x76U, reg, value) == SW_I2C_ACK) ? 1U : 0U;
}

static uint8_t SW_BME280_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
    return (SwI2c_ReadRegisters(0x76U, reg, data, length) == SW_I2C_ACK) ? 1U : 0U;
}

static uint8_t SW_BME280_WriteRegister(uint8_t reg, uint8_t value)
{
    return (SwI2c_WriteRegister(0x76U, reg, value) == SW_I2C_ACK) ? 1U : 0U;
}

static int16_t BME280_Signed12(uint16_t value)
{
    if (value & 0x0800U)
    {
        value |= 0xF000U;
    }
    return (int16_t)value;
}

static uint8_t BME280_ReadCalibration(void)
{
    uint8_t a[26];
    uint8_t h[7];

    if (!SW_BME280_ReadRegisters(0x88U, a, 26U) ||
        !SW_BME280_ReadRegisters(0xE1U, h, 7U))
    {
        return 0U;
    }

    bme280_cal.dig_T1 = (uint16_t)(a[1] << 8) | a[0];
    bme280_cal.dig_T2 = (int16_t)((uint16_t)(a[3] << 8) | a[2]);
    bme280_cal.dig_T3 = (int16_t)((uint16_t)(a[5] << 8) | a[4]);

    bme280_cal.dig_P1 = (uint16_t)(a[7] << 8) | a[6];
    bme280_cal.dig_P2 = (int16_t)((uint16_t)(a[9] << 8) | a[8]);
    bme280_cal.dig_P3 = (int16_t)((uint16_t)(a[11] << 8) | a[10]);
    bme280_cal.dig_P4 = (int16_t)((uint16_t)(a[13] << 8) | a[12]);
    bme280_cal.dig_P5 = (int16_t)((uint16_t)(a[15] << 8) | a[14]);
    bme280_cal.dig_P6 = (int16_t)((uint16_t)(a[17] << 8) | a[16]);
    bme280_cal.dig_P7 = (int16_t)((uint16_t)(a[19] << 8) | a[18]);
    bme280_cal.dig_P8 = (int16_t)((uint16_t)(a[21] << 8) | a[20]);
    bme280_cal.dig_P9 = (int16_t)((uint16_t)(a[23] << 8) | a[22]);
    bme280_cal.dig_H1 = a[25];

    bme280_cal.dig_H2 = (int16_t)((uint16_t)(h[1] << 8) | h[0]);
    bme280_cal.dig_H3 = h[2];
    bme280_cal.dig_H4 = BME280_Signed12((uint16_t)(((uint16_t)h[3] << 4) |
                                                   (h[4] & 0x0FU)));
    bme280_cal.dig_H5 = BME280_Signed12((uint16_t)(((uint16_t)h[5] << 4) |
                                                   (h[4] >> 4)));
    bme280_cal.dig_H6 = (int8_t)h[6];

    return (bme280_cal.dig_T1 != 0U && bme280_cal.dig_P1 != 0U);
}

static uint8_t BME280_Init_SoftwareI2C(void)
{
    uint8_t chip_id = 0U;

    SwI2c_InitPins();

    if (!SW_BME280_ReadRegister(0xD0U, &chip_id) || (chip_id != 0x60U))
    {
        DiagPlatform_SetBmeStatus(MODULE_STATE_FAILED, DIAG_ERR_NO_DEVICE, 0U);
        return 0U;
    }

    if (!BME280_ReadCalibration())
    {
        DiagPlatform_SetBmeStatus(MODULE_STATE_FAILED, DIAG_ERR_INVALID_DATA, 0U);
        return 0U;
    }

    /* Soft reset, then configure humidity, filter and measurements. */
    if (!SW_BME280_WriteRegister(0xE0U, 0xB6U))
    {
        return 0U;
    }
    HAL_Delay(10);

    if (!SW_BME280_WriteRegister(0xF2U, 0x01U)) /* humidity x1 */
    {
        return 0U;
    }
    if (!SW_BME280_WriteRegister(0xF5U, 0xA0U)) /* standby 1000 ms */
    {
        return 0U;
    }
    if (!SW_BME280_WriteRegister(0xF4U, 0x27U)) /* temp x1, pressure x1, normal */
    {
        return 0U;
    }

    HAL_Delay(100);
    bme280_initialized = 1U;
    DiagPlatform_SetBmeStatus(MODULE_STATE_RUNNING, DIAG_OK, 1U);
    return 1U;
}

static uint8_t BME280_ReadMeasurements(float *temperature,
                                       float *pressure,
                                       float *humidity)
{
    uint8_t d[8];
    int32_t adc_T, adc_P, adc_H;
    int32_t var1, var2, t_fine;
    int64_t pvar1, pvar2, p;
    int32_t h;

    if ((temperature == NULL) || (pressure == NULL) || (humidity == NULL))
    {
        return 0U;
    }

    if (!SW_BME280_ReadRegisters(0xF7U, d, 8U))
    {
        return 0U;
    }

    adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    adc_H = ((int32_t)d[6] << 8) | d[7];

    /* Bosch reference temperature compensation; t_fine is used below. */
    var1 = ((((adc_T >> 3) - ((int32_t)bme280_cal.dig_T1 << 1))) *
            (int32_t)bme280_cal.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)bme280_cal.dig_T1) *
              ((adc_T >> 4) - (int32_t)bme280_cal.dig_T1)) >> 12) *
            (int32_t)bme280_cal.dig_T3) >> 14;
    t_fine = var1 + var2;
    bme280_cal.t_fine = t_fine;
    *temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    /* Bosch 64-bit pressure compensation. Result is in Pa. */
    pvar1 = (int64_t)t_fine - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)bme280_cal.dig_P6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)bme280_cal.dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)bme280_cal.dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)bme280_cal.dig_P3) >> 8) +
            ((pvar1 * (int64_t)bme280_cal.dig_P2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1) *
             (int64_t)bme280_cal.dig_P1) >> 33;

    if (pvar1 == 0)
    {
        return 0U;
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = ((int64_t)bme280_cal.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = ((int64_t)bme280_cal.dig_P8 * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) +
        (((int64_t)bme280_cal.dig_P7) << 4);
    *pressure = (float)p / 256.0f; /* Pa */

    /* Bosch humidity compensation. Result is %RH. */
    h = t_fine - 76800;
    h = (((((adc_H << 14) - ((int32_t)bme280_cal.dig_H4 << 20) -
            ((int32_t)bme280_cal.dig_H5 * h)) + 16384) >> 15) *
         (((((((h * (int32_t)bme280_cal.dig_H6) >> 10) *
              (((h * (int32_t)bme280_cal.dig_H3) >> 11) + 32768)) >> 10) +
            2097152) * (int32_t)bme280_cal.dig_H2 + 8192) >> 14));
    h = h - (((((h >> 15) * (h >> 15)) >> 7) *
              (int32_t)bme280_cal.dig_H1) >> 4);

    if (h < 0)
    {
        h = 0;
    }
    else if (h > 419430400)
    {
        h = 419430400;
    }

    *humidity = (float)(h >> 12) / 1024.0f;

    return 1U;
}

static void LCD_Update_Environment_Screen(void)
{
    char line[32];
    float temperature, pressure, humidity;
    int32_t temp_centi;
    uint32_t temp_abs;
    uint32_t pressure_tenth_hpa;
    uint32_t humidity_tenth;

    if (!bme280_initialized)
    {
        LCD_SetTextColors(0xF81FU, 0x10E5U);
        LCD_DrawString(16, 100, "BME280 INIT FAILED     ");
        LCD_SetTextColors(0xE71CU, 0x10E5U);
        return;
    }

    if (BME280_ReadMeasurements(&temperature, &pressure, &humidity))
    {
        /* Use integer formatting so this also works without printf float support. */
        temp_centi = (int32_t)(temperature * 100.0f);
        temp_abs = (uint32_t)((temp_centi < 0) ? -temp_centi : temp_centi);
        snprintf(line, sizeof(line), "TEMP: %s%lu.%02lu C   ",
                 (temp_centi < 0) ? "-" : "",
                 (unsigned long)(temp_abs / 100U),
                 (unsigned long)(temp_abs % 100U));
        LCD_SetTextColors(0xE71CU, 0x10E5U);
        LCD_DrawString(16, 80, line);

        pressure_tenth_hpa = (uint32_t)(pressure / 10.0f);
        snprintf(line, sizeof(line), "PRESS: %lu.%lu hPa   ",
                 (unsigned long)(pressure_tenth_hpa / 10U),
                 (unsigned long)(pressure_tenth_hpa % 10U));
        LCD_DrawString(16, 120, line);

        humidity_tenth = (uint32_t)(humidity * 10.0f);
        snprintf(line, sizeof(line), "HUMID: %lu.%lu %%     ",
                 (unsigned long)(humidity_tenth / 10U),
                 (unsigned long)(humidity_tenth % 10U));
        LCD_DrawString(16, 160, line);

        LCD_SetTextColors(0x07F0U, 0x10E5U);
        LCD_DrawString(16, 192, "BME280: CONNECTED");
        LCD_SetTextColors(0xE71CU, 0x10E5U);
    }
    else
    {
        LCD_SetTextColors(0xF81FU, 0x10E5U);
        LCD_DrawString(16, 100, "SENSOR READ FAILED   ");
        LCD_SetTextColors(0xE71CU, 0x10E5U);
    }
}

static void LCD_Show_Environment_Screen(void)
{
    Screen_Begin("ENVIRONMENT MONITOR");
    LCD_DrawString(16, 48, "BME280 SENSOR");

    bme280_initialized = BME280_Init_SoftwareI2C();
    LCD_Update_Environment_Screen();
    bme280_last_refresh = HAL_GetTick();
    screen_mode = 1U;
}

static void LCD_Show_I2C_Scanner(void)
{
    char line[32];
    I2cScanner_Result res;
    I2cScanner_PinTest pt;
    uint8_t i;
    int pos;
    uint32_t guard;

    I2cScanner_Begin();
    guard = HAL_GetTick();
    while (I2cScanner_IsBusy() && ((HAL_GetTick() - guard) < 2000U))
    {
        I2cScanner_Tick();
    }
    I2cScanner_GetResult(&res);
    I2cScanner_GetPinTest(&pt);

    Screen_Begin("I2C BUS DIAGNOSTICS");

    LCD_Clear(12, 42, 296, 64, 0x10E5U);
    LCD_DrawLine(12, 42, 307, 42, 0x2945U);
    LCD_DrawLine(12, 105, 307, 105, 0x2945U);
    LCD_DrawLine(12, 42, 12, 105, 0x2945U);
    LCD_DrawLine(307, 42, 307, 105, 0x2945U);
    LCD_SetTextColors(0x07FFU, 0x10E5U);
    LCD_DrawString(20, 50, "PIN TEST  PB10/PB11");
    LCD_SetTextColors(0xE71CU, 0x10E5U);
    snprintf(line, sizeof(line), "IDLE SCL:%c SDA:%c",
             pt.scl_idle ? 'H' : 'L', pt.sda_idle ? 'H' : 'L');
    LCD_DrawString(20, 70, line);
    snprintf(line, sizeof(line), "LOW S:%s C:%s SHORT:%s",
             pt.sda_low_ok ? "OK" : "FAIL",
             pt.scl_low_ok ? "OK" : "FAIL",
             (pt.sda_drags_scl || pt.scl_drags_sda) ? "YES" : "NO");
    LCD_DrawString(20, 90, line);

    LCD_Clear(12, 116, 192, 64, 0x10E5U);
    LCD_DrawLine(12, 116, 203, 116, 0x2945U);
    LCD_DrawLine(12, 179, 203, 179, 0x2945U);
    LCD_DrawLine(12, 116, 12, 179, 0x2945U);
    LCD_DrawLine(203, 116, 203, 179, 0x2945U);
    LCD_SetTextColors(0x07FFU, 0x10E5U);
    LCD_DrawString(20, 124, "I2C SCAN");
    LCD_SetTextColors(0xE71CU, 0x10E5U);
    if (res.total_responding == 0U)
    {
        snprintf(line, sizeof(line), "FOUND 0: NONE");
    }
    else
    {
        pos = snprintf(line, sizeof(line), "FOUND %u:",
                       (unsigned int)res.total_responding);
        for (i = 0; i < res.address_count; i++)
        {
            if (pos > 0 && pos < (int)sizeof(line) - 4)
            {
                pos += snprintf(line + pos, sizeof(line) - (size_t)pos,
                                " %02X", (unsigned int)res.addresses[i]);
            }
        }
    }
    LCD_DrawString(20, 144, line);
    snprintf(line, sizeof(line), "NACK:%lu TO:%lu RCV:%s",
             (unsigned long)res.comm_stats.nack_count,
             (unsigned long)res.comm_stats.timeout_count,
             res.recovery_attempted ? (res.recovery_ok ? "OK" : "FAIL") : "-");
    LCD_DrawString(20, 164, line);

    LCD_Clear(212, 116, 96, 64, 0x10E5U);
    LCD_DrawLine(212, 116, 307, 116, 0x2945U);
    LCD_DrawLine(212, 179, 307, 179, 0x2945U);
    LCD_DrawLine(212, 116, 212, 179, 0x2945U);
    LCD_DrawLine(307, 116, 307, 179, 0x2945U);
    LCD_SetTextColors(0x07FFU, 0x10E5U);
    LCD_DrawString(220, 124, "BME280");
    LCD_SetTextColors(0xE71CU, 0x10E5U);
    if (res.bme280_id_ok)
    {
        snprintf(line, sizeof(line), "ID:%02X %s",
                 (unsigned int)res.bme280_chip_id,
                 res.bme280_is_bme ? "OK" : "CHK");
    }
    else
    {
        snprintf(line, sizeof(line), "ID: FAIL");
    }
    LCD_DrawString(220, 144, line);

    Screen_Footer();
    screen_mode = 1U;
}

/* =========================================================
   DISPLAY SELECTED MODULE
   ========================================================= */
static void LCD_Show_Selected_Module(void)
{
    if (menu_selection == 0)
    {
        /* Preserve both settings and the actual ON/OFF state on re-entry. */
        LCD_Show_PWM_Screen();
        return;
    }
    switch (menu_selection)
    {
        case 1:
            LCD_Show_Signal_Analyzer();
            return;
        case 2:
            LCD_Show_I2C_Scanner();
            break;
        case 3:
            LCD_Show_Environment_Screen();
            return;
        case 4:
            LCD_Show_UART_Terminal();
            return;
        case 5:
            Screen_Begin("SYSTEM INFORMATION");
            LCD_DrawString(16, 64, "STM32F103VET6");
            LCD_DrawString(16, 100, "CLOCK: 72 MHZ");
            LCD_DrawString(16, 136, "LCD: READY");
            LCD_DrawString(16, 172, "ENCODER: READY");
            break;
        default:
            break;
    }
    Screen_Footer();
    screen_mode = 1;
}

/* =========================================================
   MENU AND PWM INPUT HANDLING
   ========================================================= */
static void LCD_Handle_Menu(void)
{
    int32_t difference;
    int8_t old_selection;
    /* Check encoder rotation */
    difference = encoder_count - previous_encoder_count;
    if (difference != 0)
    {
        previous_encoder_count = encoder_count;
        if (screen_mode == 0)
        {
            old_selection = menu_selection;
            menu_selection += (int8_t)difference;
            while (menu_selection >= MENU_ITEMS)
            {
                menu_selection -= MENU_ITEMS;
            }
            while (menu_selection < 0)
            {
                menu_selection += MENU_ITEMS;
            }
            if (old_selection != menu_selection)
            {
                LCD_Update_Menu_Selection(
                    old_selection,
                    menu_selection
                );
            }
        }
        else if ((menu_selection == 0) && (screen_mode == 1U))
        {
            if (!pwm_editing)
            {
                while (difference > 0) { pwm_focus = (uint8_t)((pwm_focus + 1U) % 3U); difference--; }
                while (difference < 0) { pwm_focus = (uint8_t)((pwm_focus + 2U) % 3U); difference++; }
            }
            else if (pwm_focus == 0U)
            {
                if (difference & 1) { if (pwm_enabled) PWM_Stop(); else PWM_Start(); }
            }
            else if (pwm_focus == 1U)
            {
                while (difference > 0) { if (pwm_frequency_hz < PWM_MAX_FREQUENCY_HZ) pwm_frequency_hz += PWM_FREQUENCY_STEP_HZ; difference--; }
                while (difference < 0) { if (pwm_frequency_hz > PWM_MIN_FREQUENCY_HZ) pwm_frequency_hz -= PWM_FREQUENCY_STEP_HZ; difference++; }
                PWM_Update_Frequency();
            }
            else
            {
                while (difference > 0) { if (pwm_duty < 100U) pwm_duty++; difference--; }
                while (difference < 0) { if (pwm_duty > 0U) pwm_duty--; difference++; }
                PWM_Update_Duty();
            }
            LCD_Update_PWM_Display();
        }
    }
    /* Refresh BME280 values once per second while its screen is open. */
    if ((screen_mode == 1U) && (menu_selection == 3) &&
        ((HAL_GetTick() - bme280_last_refresh) >= 1000U))
    {
        LCD_Update_Environment_Screen();
        bme280_last_refresh = HAL_GetTick();
    }
    /* Poll every main-loop pass; redraws only when content changes. */
    if ((screen_mode == 1U) && (menu_selection == 1))
    {
        LCD_Update_Signal_Analyzer();
        analyzer_last_refresh = HAL_GetTick();
    }

    /* Short press event */
    if (encoder_short_press_event)
    {
        encoder_short_press_event = 0;
        if (screen_mode == 0)
        {
            LCD_Show_Selected_Module();
        }
        else if ((menu_selection == 0) && (screen_mode == 1U))
        {
            if (pwm_editing)
            {
                pwm_editing = 0U; /* Confirm the edited value. */
            }
            else
            {
                pwm_editing = 1U; /* Enter edit mode for selected row. */
            }
            LCD_Update_PWM_Display();
        }
    }
    /* Long press event */
    if (encoder_long_press_event)
    {
        encoder_long_press_event = 0;
        if (screen_mode == 1)
        {
            /* PWM output is intentionally left as it was, so it keeps
               running while other screens (e.g. the Signal Analyzer)
               are open. Press on the PWM screen to turn it off. */
            LCD_Show_Main_Menu();
        }
    }
}
/* =========================================================
   SYSTEM CLOCK CONFIGURATION
   STM32F103VET6 - 72 MHz
   ========================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue =
        RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL =
        RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(
        &RCC_ClkInitStruct,
        FLASH_LATENCY_2
    ) != HAL_OK)
    {
        Error_Handler();
    }
}
/* =========================================================
   ERROR HANDLER
   ========================================================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
