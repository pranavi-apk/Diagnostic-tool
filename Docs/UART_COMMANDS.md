# UART host command protocol (USART1, 115200 8-N-1)

RX uses **USART1 receive interrupt** (RXNE); command parsing runs in the main loop. TX is polled.

Type a command at the `>` prompt and press Enter. Lines are case-insensitive.

| Command | Action |
|---------|--------|
| `HELP` | List commands |
| `STATUS` | PWM + analyzer + BME summary |
| `GET_PWM` or `PWM?` | Read PWM config |
| `SET_PWM <hz> <duty> [ON\|OFF]` | Set frequency/duty; optional output state |
| `PWM ON` / `PWM OFF` | Enable/disable PWM output |
| `GET_ANALYZER` | Measured Hz, duty, PASS/FAIL |
| `I2C SCAN` | Run bus scan (may take ~2 s) |
| `GET_ENV` | BME280 temperature, pressure, humidity |
| `GET_PROBE` | DC probe voltage (mV), raw ADC, verdict |
| `PROBE CAL <mV>` | Calibrate against known voltage at probe tip |

Examples:

```
SET_PWM 1000 50 ON
GET_ANALYZER
I2C SCAN
```

Responses start with `OK` or `ERR`. The UART menu shows the last command and last status line.