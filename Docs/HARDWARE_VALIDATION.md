# Hardware validation (fill in after bench testing)

| Test | Procedure | Result |
|------|-----------|--------|
| PWM vs analyzer loopback | PC6→PA6, compare PWM set freq/duty to analyzer averages | Not performed |
| Analyzer missing signal | Disconnect PA6, confirm NO SIGNAL / timeout | Not performed |
| I2C scan | Open I2C screen, verify 0x50 / 0x76 and BME ID 0x60 | Not performed |
| I2C disconnect | Remove BME280, confirm timeouts / recovery stats | Not performed |
| LCD responsiveness | Navigate menu during I2C scan | Not performed |
| Build | STM32CubeIDE Debug build | Not performed (run locally) |
