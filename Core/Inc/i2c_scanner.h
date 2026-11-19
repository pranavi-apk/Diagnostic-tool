/**
 * @file i2c_scanner.h
 * @brief Software I2C bus scan/diagnostics on PB10/PB11.
 */
#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include <stdint.h>
#include "diag_types.h"
#include "sw_i2c.h"

#define I2C_SCANNER_MAX_FOUND  8U
#define BME280_I2C_ADDR_7BIT   0x76U
#define BME280_CHIP_ID         0x60U

typedef struct
{
    uint8_t scl_idle;
    uint8_t sda_idle;
    uint8_t sda_low_ok;
    uint8_t scl_low_ok;
    uint8_t sda_drags_scl;
    uint8_t scl_drags_sda;
    uint8_t release_ok;
} I2cScanner_PinTest;

typedef struct
{
    uint8_t addresses[I2C_SCANNER_MAX_FOUND];
    uint8_t address_count;
    uint8_t total_responding;
    uint8_t bme280_chip_id;
    uint8_t bme280_id_ok;
    uint8_t bme280_is_bme;
    SwI2c_Stats comm_stats;
    uint8_t recovery_attempted;
    uint8_t recovery_ok;
    ModuleState state;
    DiagErrorCode last_error;
    uint8_t scan_complete;
} I2cScanner_Result;

void I2cScanner_Begin(void);
void I2cScanner_Tick(void);
uint8_t I2cScanner_IsBusy(void);
void I2cScanner_GetResult(I2cScanner_Result *out);
void I2cScanner_GetPinTest(I2cScanner_PinTest *out);

#endif /* I2C_SCANNER_H */
