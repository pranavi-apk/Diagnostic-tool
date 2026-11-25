# DC voltage probe — hardware hookup

Sense pin default: **PC3** (`DC_PROBE_ADC_PIN` in `dc_probe.c`). Avoid **PC0/PC1** on MINI-V3 if board loads those pins.

## Divider (no external Schottky)

```
PROBE TIP ── R1 47k ── NODE ── R2 12k (10k+2k) ── GND
                         │
                         ├── C1 (2.2nF–100nF) ── GND
                         └── PC3 (ADC)
```

Black lead = target **GND**. Tool must be **powered on** before touching the target.

## After assembly

1. Open **DC PROBE**; on **3.3 V** expect **RAW ~800** and **3.3V PASS**.
2. `PROBE CAL 3300` over UART if you want to match a multimeter exactly.

Full-scale about **16 V** with 47k+12k and 3.3 V ADC reference.