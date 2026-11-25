/**
 * @file uart_cmd.h
 * @brief Host command protocol on USART1 (line-oriented, uppercase verbs).
 */
#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>

typedef void (*UartCmd_TxFn)(const char *text);

typedef void (*UartCmd_PwmSetFn)(uint32_t freq_hz, uint8_t duty_percent, int8_t output_on);
/* Return 1 if OK. duty/percent only if ok. */
typedef uint8_t (*UartCmd_EnvReadFn)(float *temp_c, float *press_pa, float *humidity_rh);

void UartCmd_Init(UartCmd_TxFn tx, UartCmd_PwmSetFn pwm_set, UartCmd_EnvReadFn env_read);
void UartCmd_HandleLine(const char *line);
const char *UartCmd_LastStatusLine(void);

#endif /* UART_CMD_H */