# Group 28 — Diagnostic Platform handoff

Repo: https://github.com/pranavi-apk/Diagnostic-tool.git  
MCU: **STM32F103VET6** @ **72 MHz**  
IDE: STM32CubeIDE

## What this project is

A **bench diagnostic tool** on one board (not FreeRTOS). Six menu apps share one UI (LCD + rotary encoder) and show different **interfaces**: timer PWM, timer capture, software I²C, UART, **ADC DC probe**, FSMC LCD.

## Features (current)

| Menu | What it does | Key hardware |
|------|----------------|--------------|
| PWM Generator | Set freq / duty; requested vs actual Hz; safe 0%/100%/OFF | **TIM8 CH1 → PC6** |
| Signal Analyzer | Input capture, 16-cycle avg, PASS/FAIL vs PWM | **TIM3 → PA6** |
| I2C Scanner | Pin test, address scan, NACK/timeout/recovery, BME ID | **SW I²C PB10 SCL / PB11 SDA** |
| ENV | BME280 temp / pressure / humidity (factory cal) | Same I²C bus, addr **0x76** |
| UART Terminal | 115200 8-N-1 echo + LCD | **USART1 PA9 TX / PA10 RX** |
| DC Probe | Divider-scaled voltage, rail/logic verdict | **ADC1** on **PC3** (configurable) |

Integration demo: banana lead **PC6 → PA6** with PWM **ON** and duty **~50%** (not 0%/100% — those are DC levels).

## Important source files

- `Core/Src/main.c` — UI, encoder, UART, BME280
- `Core/Src/pwm_gen.c` — PWM module
- `Core/Src/signal_analyzer.c` — capture + stats + ISR
- `Core/Src/sw_i2c.c` — bit-bang I²C
- `Core/Src/i2c_scanner.c` — diagnostics FSM
- `Core/Src/diag_iface.c` — status snapshot API
- `Core/Src/dc_probe.c` — ADC DC probe (47k+12k divider, cal)
- `Extras/orphan_modules/` — old DIAG RUN sequence (not in build)

## Open in CubeIDE

1. Clone the repo.
2. File → Open Projects from File System / Import existing STM32 project.
3. Build **Debug**, flash with ST-Link.

## Warnings

- **Do not regenerate CubeMX** without backing up `main.c` and custom modules — regen has wiped custom code before.
- TIM8 / TIM3 are configured largely in **application code**, not only in the `.ioc`.
- Software I²C needs **3.3 V pull-ups** on SCL/SDA for BME280.

## Future work (agreed direction)

1. UART as a **command protocol** (PC controls PWM / analyzer / I²C), not only echo.
2. **Closed-loop PWM** using analyzer feedback (PC6↔PA6).
3. Optional **DMA** for UART logging.

## Docs

- `Docs/VALIDATION_RECORD.md` — bench tables (fill measured results)
- `Docs/HARDWARE_VALIDATION.md` — hardware notes

## Contact

Previous owner: Pranavi (Group 28). Teammate takes ongoing firmware + report from here.
