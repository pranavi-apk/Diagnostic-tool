#include "fsmc.h"

/*
 * STM32F103VET6 FSMC setup for the existing 16-bit LCD interface.
 * The project does not contain the STM32F1 SRAM HAL driver, so FSMC Bank 1
 * is configured directly through the CMSIS peripheral registers.
 *
 * LCD command address: 0x60000000
 * LCD data address:    0x60020000
 */
void MX_FSMC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks for the FSMC GPIO pins and FSMC peripheral. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    RCC->AHBENR |= RCC_AHBENR_FSMCEN;

    /* FSMC pins on GPIOE: D4-D12. */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                          GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* FSMC pins on GPIOD: D0-D3, D13-D15, A16, NOE, NWE and NE1. */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 |
                          GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 |
                          GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* BTCR[0] is BCR1; BTCR[1] is BTR1. Disable the bank while configuring. */
    FSMC_Bank1->BTCR[0] = 0U;

    /* BCR1: 16-bit bus (MWID=01), asynchronous SRAM, writes enabled.
       Address/data multiplexing and extended mode remain disabled. */
    FSMC_Bank1->BTCR[0] = (1U << 4) | (1U << 12);

    /* BTR1: conservative asynchronous timing (ADDSET=15, DATAST=255). */
    FSMC_Bank1->BTCR[1] = 15U | (255U << 8);

    /* Enable Bank 1 after all settings have been written. */
    FSMC_Bank1->BTCR[0] |= 1U;
}
