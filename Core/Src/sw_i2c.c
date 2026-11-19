/**
 * @file sw_i2c.c
 */
#include "sw_i2c.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

static SwI2c_Stats s_stats;
static ModuleState s_state = MODULE_STATE_IDLE;
static uint8_t s_prepared = 0U;

static void sw_delay(void)
{
    volatile uint32_t n = 150U;
    while (n--)
    {
    }
}

static uint8_t sw_wait_level(GPIO_PinState level, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(SW_I2C_GPIO_PORT, SW_I2C_SDA_PIN) != level)
    {
        if ((HAL_GetTick() - start) > timeout_ms)
        {
            s_stats.timeout_count++;
            return 0U;
        }
    }
    return 1U;
}

static void sw_scl(uint8_t level)
{
    HAL_GPIO_WritePin(SW_I2C_GPIO_PORT, SW_I2C_SCL_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    sw_delay();
}

static void sw_sda(uint8_t level)
{
    HAL_GPIO_WritePin(SW_I2C_GPIO_PORT, SW_I2C_SDA_PIN,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    sw_delay();
}

static void sw_start(void)
{
    sw_sda(1);
    sw_scl(1);
    sw_sda(0);
    sw_scl(0);
}

static void sw_stop(void)
{
    sw_sda(0);
    sw_scl(1);
    sw_sda(1);
}

static SwI2cResult sw_write_byte_once(uint8_t value)
{
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        sw_sda((value & 0x80U) ? 1U : 0U);
        sw_scl(1);
        sw_scl(0);
        value <<= 1;
    }

    sw_sda(1);
    sw_scl(1);

    if (!sw_wait_level(GPIO_PIN_RESET, SW_I2C_DEFAULT_TIMEOUT_MS))
    {
        if (HAL_GPIO_ReadPin(SW_I2C_GPIO_PORT, SW_I2C_SDA_PIN) == GPIO_PIN_SET)
        {
            s_stats.nack_count++;
            sw_scl(0);
            return SW_I2C_NACK;
        }
        sw_scl(0);
        return SW_I2C_TIMEOUT;
    }

    sw_scl(0);
    return SW_I2C_ACK;
}

static SwI2cResult sw_write_byte(uint8_t value)
{
    uint8_t attempt;
    SwI2cResult r;

    for (attempt = 0U; attempt <= SW_I2C_DEFAULT_RETRY_COUNT; attempt++)
    {
        r = sw_write_byte_once(value);
        if (r == SW_I2C_ACK)
        {
            return r;
        }
        if (r == SW_I2C_NACK)
        {
            return r;
        }
        (void)SwI2c_RecoverBus();
    }

    s_stats.failed_transactions++;
    return SW_I2C_TIMEOUT;
}

static SwI2cResult sw_read_byte(uint8_t send_ack, uint8_t *out)
{
    uint8_t i;
    uint8_t value = 0U;

    if (out == NULL)
    {
        return SW_I2C_BUS_ERROR;
    }

    sw_sda(1);

    for (i = 0U; i < 8U; i++)
    {
        value <<= 1;
        sw_scl(1);
        if (HAL_GPIO_ReadPin(SW_I2C_GPIO_PORT, SW_I2C_SDA_PIN) == GPIO_PIN_SET)
        {
            value |= 1U;
        }
        sw_scl(0);
    }

    sw_sda(send_ack ? 0U : 1U);
    sw_scl(1);
    sw_scl(0);
    sw_sda(1);
    *out = value;
    return SW_I2C_ACK;
}

void SwI2c_InitPins(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(SW_I2C_GPIO_PORT, SW_I2C_SCL_PIN | SW_I2C_SDA_PIN, GPIO_PIN_SET);
    g.Pin = SW_I2C_SCL_PIN | SW_I2C_SDA_PIN;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SW_I2C_GPIO_PORT, &g);
    s_prepared = 1U;
    s_state = MODULE_STATE_RUNNING;
}

SwI2cResult SwI2c_ProbeAddress(uint8_t addr7)
{
    SwI2cResult r;

    if (!s_prepared)
    {
        SwI2c_InitPins();
    }

    sw_start();
    r = sw_write_byte((uint8_t)(addr7 << 1));
    sw_stop();

    if (r == SW_I2C_ACK)
    {
        return SW_I2C_ACK;
    }
    return r;
}

SwI2cResult SwI2c_ReadRegister(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    SwI2cResult r;

    if (value == NULL)
    {
        return SW_I2C_BUS_ERROR;
    }
    if (!s_prepared)
    {
        SwI2c_InitPins();
    }

    sw_start();
    r = sw_write_byte((uint8_t)(addr7 << 1));
    if (r != SW_I2C_ACK)
    {
        sw_stop();
        s_stats.failed_transactions++;
        return r;
    }

    r = sw_write_byte(reg);
    if (r != SW_I2C_ACK)
    {
        sw_stop();
        s_stats.failed_transactions++;
        return r;
    }

    sw_sda(1);
    sw_scl(1);
    sw_sda(0);
    sw_scl(0);

    r = sw_write_byte((uint8_t)((addr7 << 1) | 1U));
    if (r != SW_I2C_ACK)
    {
        sw_stop();
        s_stats.failed_transactions++;
        return r;
    }

    r = sw_read_byte(0U, value);
    sw_stop();
  return (r == SW_I2C_ACK) ? SW_I2C_ACK : r;
}

SwI2cResult SwI2c_ReadRegisters(uint8_t addr7, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t i;
    SwI2cResult r;
    uint8_t b;

    if ((data == NULL) || (length == 0U))
    {
        return SW_I2C_BUS_ERROR;
    }
    if (!s_prepared)
    {
        SwI2c_InitPins();
    }

    sw_start();
    r = sw_write_byte((uint8_t)(addr7 << 1));
    if (r != SW_I2C_ACK)
    {
        sw_stop();
        s_stats.failed_transactions++;
        return r;
    }

    r = sw_write_byte(reg);
    if (r != SW_I2C_ACK)
    {
        sw_stop();
        s_stats.failed_transactions++;
        return r;
    }

    sw_sda(1);
    sw_scl(1);
    sw_sda(0);
    sw_scl(0);

    r = sw_write_byte((uint8_t)((addr7 << 1) | 1U));
    if (r != SW_I2C_ACK)
    {
        sw_stop();
        s_stats.failed_transactions++;
        return r;
    }

    for (i = 0U; i < length; i++)
    {
        r = sw_read_byte((i < (uint8_t)(length - 1U)) ? 1U : 0U, &b);
        if (r != SW_I2C_ACK)
        {
            sw_stop();
            s_stats.failed_transactions++;
            return r;
        }
        data[i] = b;
    }

    sw_stop();
    return SW_I2C_ACK;
}

SwI2cResult SwI2c_WriteRegister(uint8_t addr7, uint8_t reg, uint8_t value)
{
    SwI2cResult r;

    if (!s_prepared)
    {
        SwI2c_InitPins();
    }

    sw_start();
    r = sw_write_byte((uint8_t)(addr7 << 1));
    if (r == SW_I2C_ACK)
    {
        r = sw_write_byte(reg);
    }
    if (r == SW_I2C_ACK)
    {
        r = sw_write_byte(value);
    }
    sw_stop();

    if (r != SW_I2C_ACK)
    {
        s_stats.failed_transactions++;
    }
    return r;
}

DiagErrorCode SwI2c_RecoverBus(void)
{
    uint8_t i;

    s_stats.recovery_attempted = 1U;
    s_state = MODULE_STATE_RECOVERING;
    SwI2c_InitPins();

    sw_sda(1);
    sw_scl(1);

    if (HAL_GPIO_ReadPin(SW_I2C_GPIO_PORT, SW_I2C_SDA_PIN) == GPIO_PIN_SET)
    {
        s_stats.recovery_succeeded = 1U;
        s_state = MODULE_STATE_RUNNING;
        return DIAG_OK;
    }

    for (i = 0U; i < 9U; i++)
    {
        sw_scl(0);
        sw_scl(1);
    }

    sw_stop();

    if (HAL_GPIO_ReadPin(SW_I2C_GPIO_PORT, SW_I2C_SDA_PIN) == GPIO_PIN_SET)
    {
        s_stats.recovery_succeeded = 1U;
        s_state = MODULE_STATE_RUNNING;
        return DIAG_OK;
    }

    s_stats.recovery_succeeded = 0U;
    s_state = MODULE_STATE_FAILED;
    return DIAG_ERR_RECOVERY_FAILED;
}

void SwI2c_GetStats(SwI2c_Stats *out)
{
    if (out != NULL)
    {
        *out = s_stats;
    }
}

void SwI2c_ResetStats(void)
{
    s_stats.timeout_count = 0U;
    s_stats.nack_count = 0U;
    s_stats.failed_transactions = 0U;
    s_stats.recovery_attempted = 0U;
    s_stats.recovery_succeeded = 0U;
}

ModuleState SwI2c_GetState(void)
{
    return s_state;
}
