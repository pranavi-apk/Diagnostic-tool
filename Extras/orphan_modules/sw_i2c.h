/**
 * @file sw_i2c.h
 * @brief Bit-banged open-drain I2C on PB10 (SCL) / PB11 (SDA) — same as I2C2.
 */
#ifndef SW_I2C_H
#define SW_I2C_H

#include <stdint.h>
#include "diag_types.h"
#include "stm32f1xx_hal.h"

#define SW_I2C_SCL_PIN   GPIO_PIN_10
#define SW_I2C_SDA_PIN   GPIO_PIN_11
#define SW_I2C_GPIO_PORT GPIOB

#define SW_I2C_DEFAULT_TIMEOUT_MS  5U
#define SW_I2C_DEFAULT_RETRY_COUNT 2U

typedef enum
{
    SW_I2C_ACK = 0,
    SW_I2C_NACK,
    SW_I2C_TIMEOUT,
    SW_I2C_BUS_ERROR
} SwI2cResult;

typedef struct
{
    uint8_t recovery_attempted;
    uint8_t recovery_succeeded;
    uint32_t timeout_count;
    uint32_t nack_count;
    uint32_t failed_transactions;
} SwI2c_Stats;

void SwI2c_InitPins(void);
SwI2cResult SwI2c_ProbeAddress(uint8_t addr7);
SwI2cResult SwI2c_ReadRegister(uint8_t addr7, uint8_t reg, uint8_t *value);
SwI2cResult SwI2c_ReadRegisters(uint8_t addr7, uint8_t reg, uint8_t *data, uint8_t length);
SwI2cResult SwI2c_WriteRegister(uint8_t addr7, uint8_t reg, uint8_t value);
DiagErrorCode SwI2c_RecoverBus(void);
void SwI2c_GetStats(SwI2c_Stats *out);
void SwI2c_ResetStats(void);
ModuleState SwI2c_GetState(void);

#endif /* SW_I2C_H */
