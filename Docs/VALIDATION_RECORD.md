# Group 28 Validation Record

Backup before integration: `../diagnostic_platform_1_backup_pre_integration`

## How to run the automated sequence (firmware)

1. Wire **PC6 (PWM) → PA6 (analyzer)** for loopback.
2. Wire **BME280** to **PB10=SCL / PB11=SDA** with pull-ups to 3.3 V.
3. On the dashboard open **DIAG RUN**.
4. Wait for the report screen: **OVERALL PASS/FAIL** plus LOOP / I2C / BME lines.
5. Copy the on-screen numbers into the tables below.

Target used by firmware (`diag_sequence.h`):
- Frequency **1000 Hz**, duty **50%**
- Frequency tolerance **±2% (20000 ppm)**
- Duty tolerance **±5 percentage points**
- Minimum **8** analyzer samples before evaluation
- Up to **15 s** to connect PC6→PA6, then **400 ms** settle after lock

## Module integration status

| Module | Sources | Role in DIAG RUN |
|--------|---------|------------------|
| PWM | `pwm_gen.c` | Generates 1 kHz / 50% test signal |
| Analyzer | `signal_analyzer.c` | Multi-cycle measure + PASS/FAIL |
| I2C | `sw_i2c.c`, `i2c_scanner.c` | Scan + recovery stats |
| BME280 | `SwI2c_*` + chip ID 0xD0 | ID must be `0x60` |
| Sequence | `diag_sequence.c` | Coordinates steps + report |
| Status API | `diag_iface.c` | Aggregated module snapshot |

## A. Reference vs measured (fill on bench)

| Item | Reference / expected | Measured | Error % | Pass? | Notes |
|------|----------------------|----------|---------|-------|-------|
| PWM configured frequency | 1000 Hz | | | | From DIAG RUN / PWM screen |
| Analyzer average frequency | ≈1000 Hz (±2%) | | | | LOOP line |
| PWM configured duty | 50% | | | | |
| Analyzer average duty | ≈50% (±5 pp) | | | | |
| Analyzer sample count | ≥8 | | | | |
| I2C devices found | ≥1 (often 0x50, 0x76) | | | | |
| BME280 chip ID | 0x60 | | | | |
| DIAG RUN overall | PASS | | | | |

Error % formula (frequency):  
`|measured − expected| / expected × 100`

## B. Fault-injection tests (fill on bench)

| Test | Action | Expected | Observed | Pass? |
|------|--------|----------|----------|-------|
| Missing signal | Disconnect PA6, run DIAG | LOOP FAIL / NO SIG | | |
| I2C disconnect | Unplug BME (+EEPROM if present), run DIAG | I2C and/or BME FAIL | | |
| I2C recovery | Briefly short/release SDA safely if appropriate | recovery flag OK or FAIL noted | | |
| PWM OFF extremes | Set duty 0% / 100% / OFF on PWM screen | Steady low / high / low | | |
| LCD responsive | Turn encoder during DIAG RUN | UI updates step / report | | |

## C. Manual module checks (optional, outside DIAG RUN)

| Test | Result | Notes |
|------|--------|-------|
| Analyzer graphs (waveform + 6-cycle trend) | Pending | |
| I2C pin test IDLE H/H | Pending | |
| ENV monitor T/H/P | Pending | |

## Sign-off

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Tester | | | |
| Reviewer | | | |
