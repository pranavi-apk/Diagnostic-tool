/**
 * @file i2c.c
 * @brief Hardware I2C2 on PB10 (SCL) / PB11 (SDA).
 */
#include "i2c.h"
#include "gpio.h"

I2C_HandleTypeDef hi2c1; /* Cube name; Instance = I2C2 */

#define I2C_HW               I2C2
#define I2C_HW_SCL_PIN       GPIO_PIN_10
#define I2C_HW_SDA_PIN       GPIO_PIN_11
#define I2C_HW_GPIO          GPIOB
#define BME280_ADDR7         0x76U

void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C_HW;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  I2cHw_PrepareBus();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  GPIO_InitTypeDef g = {0};
  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    g.Pin = I2C_HW_SCL_PIN | I2C_HW_SDA_PIN;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_HW_GPIO, &g);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle)
{
  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(I2C_HW_GPIO, I2C_HW_SCL_PIN | I2C_HW_SDA_PIN);
  }
}

static uint8_t wait_gpio(uint16_t pin, GPIO_PinState want, uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while (HAL_GPIO_ReadPin(I2C_HW_GPIO, pin) != want)
  {
    if ((HAL_GetTick() - t0) > ms)
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t wait_sr1(uint32_t mask, uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((I2C_HW->SR1 & mask) == 0U)
  {
    if ((HAL_GetTick() - t0) > ms)
    {
      return 0U;
    }
  }
  return 1U;
}

static void pins_gpio_od(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = I2C_HW_SCL_PIN | I2C_HW_SDA_PIN;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_HW_GPIO, &g);
}

static void pins_af_od(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin = I2C_HW_SCL_PIN | I2C_HW_SDA_PIN;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_HW_GPIO, &g);
}

/* STM32F103 errata 2.9.7 busy-flag unlock */
static HAL_StatusTypeDef errata_unlock(void)
{
  I2C_HW->CR1 &= (uint16_t)~I2C_CR1_PE;
  HAL_Delay(1);
  pins_gpio_od();
  HAL_GPIO_WritePin(I2C_HW_GPIO, I2C_HW_SCL_PIN | I2C_HW_SDA_PIN, GPIO_PIN_SET);
  if (!wait_gpio(I2C_HW_SCL_PIN, GPIO_PIN_SET, 50) ||
      !wait_gpio(I2C_HW_SDA_PIN, GPIO_PIN_SET, 50))
  {
    return HAL_TIMEOUT;
  }
  HAL_GPIO_WritePin(I2C_HW_GPIO, I2C_HW_SDA_PIN, GPIO_PIN_RESET);
  if (!wait_gpio(I2C_HW_SDA_PIN, GPIO_PIN_RESET, 50))
  {
    return HAL_TIMEOUT;
  }
  HAL_GPIO_WritePin(I2C_HW_GPIO, I2C_HW_SCL_PIN, GPIO_PIN_RESET);
  if (!wait_gpio(I2C_HW_SCL_PIN, GPIO_PIN_RESET, 50))
  {
    return HAL_TIMEOUT;
  }
  HAL_GPIO_WritePin(I2C_HW_GPIO, I2C_HW_SCL_PIN, GPIO_PIN_SET);
  if (!wait_gpio(I2C_HW_SCL_PIN, GPIO_PIN_SET, 50))
  {
    return HAL_TIMEOUT;
  }
  HAL_GPIO_WritePin(I2C_HW_GPIO, I2C_HW_SDA_PIN, GPIO_PIN_SET);
  if (!wait_gpio(I2C_HW_SDA_PIN, GPIO_PIN_SET, 50))
  {
    return HAL_TIMEOUT;
  }
  pins_af_od();
  I2C_HW->CR1 |= I2C_CR1_SWRST;
  HAL_Delay(1);
  I2C_HW->CR1 &= (uint16_t)~I2C_CR1_SWRST;
  HAL_Delay(1);
  return HAL_OK;
}

static void program_100k(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t fr = pclk1 / 1000000U;
  uint32_t ccr;
  if (fr < 2U)
  {
    fr = 2U;
  }
  if (fr > 36U)
  {
    fr = 36U;
  }
  I2C_HW->CR1 &= (uint16_t)~I2C_CR1_PE;
  I2C_HW->CR2 = (uint16_t)fr;
  I2C_HW->TRISE = (uint16_t)(fr + 1U);
  ccr = pclk1 / (100000U * 2U);
  if (ccr < 4U)
  {
    ccr = 4U;
  }
  I2C_HW->CCR = (uint16_t)ccr;
  I2C_HW->OAR1 = (uint16_t)(1U << 14);
  I2C_HW->OAR2 = 0U;
  I2C_HW->CR1 = 0U;
}

HAL_StatusTypeDef I2cHw_ClearBusyFlagErratum(uint32_t timeout_ms)
{
  (void)timeout_ms;
  if (errata_unlock() != HAL_OK)
  {
    return HAL_TIMEOUT;
  }
  program_100k();
  I2C_HW->CR1 |= I2C_CR1_PE;
  HAL_Delay(1);
  hi2c1.Instance = I2C_HW;
  hi2c1.State = HAL_I2C_STATE_READY;
  hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
  if ((I2C_HW->SR2 & I2C_SR2_BUSY) != 0U)
  {
    hi2c1.ErrorCode = HAL_I2C_WRONG_START;
    return HAL_ERROR;
  }
  return HAL_OK;
}

void I2cHw_PrepareBus(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();
  I2C_HW->CR1 &= (uint16_t)~I2C_CR1_PE;
  __HAL_RCC_I2C2_FORCE_RESET();
  HAL_Delay(1);
  __HAL_RCC_I2C2_RELEASE_RESET();
  HAL_Delay(1);
  hi2c1.Instance = I2C_HW;
  (void)I2cHw_ClearBusyFlagErratum(50U);
}

HAL_StatusTypeDef I2cHw_ProbeDevice(uint16_t addr_shifted, uint32_t trials, uint32_t timeout_ms)
{
  I2cHw_PrepareBus();
  return HAL_I2C_IsDeviceReady(&hi2c1, addr_shifted, trials, timeout_ms);
}

static HAL_StatusTypeDef bare_mem_read8(uint8_t addr7, uint8_t reg, uint8_t *out)
{
  if (out == NULL)
  {
    return HAL_ERROR;
  }
  I2C_HW->CR1 |= I2C_CR1_STOP;
  HAL_Delay(1);
  if ((I2C_HW->SR2 & I2C_SR2_BUSY) != 0U)
  {
    (void)I2cHw_ClearBusyFlagErratum(50U);
  }

  I2C_HW->CR1 |= I2C_CR1_START;
  if (!wait_sr1(I2C_SR1_SB, 50U))
  {
    hi2c1.ErrorCode = HAL_I2C_WRONG_START;
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_TIMEOUT;
  }
  I2C_HW->DR = (uint8_t)(addr7 << 1);
  if (!wait_sr1(I2C_SR1_ADDR | I2C_SR1_AF, 50U))
  {
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_TIMEOUT;
  }
  if ((I2C_HW->SR1 & I2C_SR1_AF) != 0U)
  {
    I2C_HW->SR1 = (uint16_t)~I2C_SR1_AF;
    I2C_HW->CR1 |= I2C_CR1_STOP;
    hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
    return HAL_ERROR;
  }
  (void)I2C_HW->SR1;
  (void)I2C_HW->SR2;

  if (!wait_sr1(I2C_SR1_TXE, 50U))
  {
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_TIMEOUT;
  }
  I2C_HW->DR = reg;
  if (!wait_sr1(I2C_SR1_BTF | I2C_SR1_AF, 50U))
  {
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_TIMEOUT;
  }
  if ((I2C_HW->SR1 & I2C_SR1_AF) != 0U)
  {
    I2C_HW->SR1 = (uint16_t)~I2C_SR1_AF;
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_ERROR;
  }

  I2C_HW->CR1 |= I2C_CR1_START;
  if (!wait_sr1(I2C_SR1_SB, 50U))
  {
    hi2c1.ErrorCode = HAL_I2C_WRONG_START;
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_TIMEOUT;
  }
  I2C_HW->DR = (uint8_t)((addr7 << 1) | 1U);
  if (!wait_sr1(I2C_SR1_ADDR | I2C_SR1_AF, 50U))
  {
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_TIMEOUT;
  }
  if ((I2C_HW->SR1 & I2C_SR1_AF) != 0U)
  {
    I2C_HW->SR1 = (uint16_t)~I2C_SR1_AF;
    I2C_HW->CR1 |= I2C_CR1_STOP;
    return HAL_ERROR;
  }

  I2C_HW->CR1 &= (uint16_t)~I2C_CR1_ACK;
  (void)I2C_HW->SR1;
  (void)I2C_HW->SR2;
  I2C_HW->CR1 |= I2C_CR1_STOP;
  if (!wait_sr1(I2C_SR1_RXNE, 50U))
  {
    return HAL_TIMEOUT;
  }
  *out = (uint8_t)I2C_HW->DR;
  hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c1.State = HAL_I2C_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef I2cHw_ReadBme280ChipId(uint8_t *chip_id)
{
  uint8_t i;
  HAL_StatusTypeDef st = HAL_ERROR;
  if (chip_id == NULL)
  {
    return HAL_ERROR;
  }
  for (i = 0U; i < 3U; i++)
  {
    I2cHw_PrepareBus();
    st = bare_mem_read8(BME280_ADDR7, 0xD0U, chip_id);
    if (st == HAL_OK)
    {
      return HAL_OK;
    }
  }
  return st;
}
