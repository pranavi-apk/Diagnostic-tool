/**
 * @file i2c_scanner.c
 */
#include "i2c_scanner.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

typedef enum
{
    SCAN_PHASE_IDLE = 0,
    SCAN_PHASE_HW_PROBE,
    SCAN_PHASE_DEINIT,
    SCAN_PHASE_PIN_TEST_SDA,
    SCAN_PHASE_PIN_TEST_SCL,
    SCAN_PHASE_PIN_TEST_RELEASE,
    SCAN_PHASE_SCAN_ADDR,
    SCAN_PHASE_CHIP_ID,
    SCAN_PHASE_REINIT_HW,
    SCAN_PHASE_DONE
} ScanPhase;

static ScanPhase s_phase = SCAN_PHASE_IDLE;
static I2cScanner_Result s_result;
static I2cScanner_PinTest s_pin_test;
static uint8_t s_scan_addr = 0x08U;
static uint8_t s_pin_step = 0U;
static uint32_t s_step_deadline = 0U;

extern I2C_HandleTypeDef hi2c1;

static void scanner_pin_test_begin(void)
{
    s_pin_step = 0U;
    s_step_deadline = HAL_GetTick();
}

static uint8_t scanner_pin_test_tick(void)
{
    if ((HAL_GetTick() - s_step_deadline) < 2U)
    {
        return 0U;
    }

    switch (s_pin_step)
    {
    case 0U:
        s_pin_test.scl_idle =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SCL_PIN) == GPIO_PIN_SET);
        s_pin_test.sda_idle =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SDA_PIN) == GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, SW_I2C_SDA_PIN, GPIO_PIN_RESET);
        s_pin_step = 1U;
        break;
    case 1U:
        s_pin_test.sda_low_ok =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SDA_PIN) == GPIO_PIN_RESET);
        s_pin_test.sda_drags_scl =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SCL_PIN) == GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, SW_I2C_SDA_PIN, GPIO_PIN_SET);
        s_pin_step = 2U;
        break;
    case 2U:
        HAL_GPIO_WritePin(GPIOB, SW_I2C_SCL_PIN, GPIO_PIN_RESET);
        s_pin_step = 3U;
        break;
    case 3U:
        s_pin_test.scl_low_ok =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SCL_PIN) == GPIO_PIN_RESET);
        s_pin_test.scl_drags_sda =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SDA_PIN) == GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, SW_I2C_SCL_PIN, GPIO_PIN_SET);
        s_pin_step = 4U;
        break;
    case 4U:
        s_pin_test.release_ok =
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SDA_PIN) == GPIO_PIN_SET) &&
            (HAL_GPIO_ReadPin(GPIOB, SW_I2C_SCL_PIN) == GPIO_PIN_SET);
        return 1U;
    default:
        return 1U;
    }

    s_step_deadline = HAL_GetTick();
    return 0U;
}

void I2cScanner_Begin(void)
{
    s_phase = SCAN_PHASE_HW_PROBE;
    s_scan_addr = 0x08U;
    s_result = (I2cScanner_Result){0};
    s_pin_test = (I2cScanner_PinTest){0};
    s_result.state = MODULE_STATE_INIT;
    s_result.scan_complete = 0U;
    SwI2c_ResetStats();
}

void I2cScanner_Tick(void)
{
    SwI2cResult sw;

    switch (s_phase)
    {
    case SCAN_PHASE_IDLE:
        return;

    case SCAN_PHASE_HW_PROBE:
        s_result.hw_bme_id_ok = 0U;
        s_result.hw_bme_is_bme = 0U;
        s_result.hw_probe_error = HAL_I2C_ERROR_NONE;
        s_result.hw_probe_status = HAL_ERROR;

        I2cHw_PrepareBus(); /* HAL init + F103 errata 2.9.7 BUSY unlock */

        if (I2cHw_ReadBme280ChipId(&s_result.hw_bme_chip_id) == HAL_OK)
        {
            s_result.hw_bme_id_ok = 1U;
            s_result.hw_bme_is_bme =
                (s_result.hw_bme_chip_id == BME280_CHIP_ID);
            s_result.hw_probe_status = HAL_OK;
            s_result.hw_probe_error = HAL_I2C_ERROR_NONE;
        }
        else
        {
            s_result.hw_probe_status = HAL_TIMEOUT;
            s_result.hw_probe_error = hi2c1.ErrorCode;
        }
        /* PB10/PB11 live in CRL/CRH high nibble of CRH bits [15:8]. */
        s_result.gpio_crl_snapshot = (GPIOB->CRH >> 8) & 0xFFU;
        __HAL_RCC_AFIO_CLK_ENABLE();
        s_result.afio_remap_snapshot = 0U; /* I2C2 has no remap */
        s_phase = SCAN_PHASE_DEINIT;
        break;

    case SCAN_PHASE_DEINIT:
        HAL_I2C_DeInit(&hi2c1);
        SwI2c_InitPins();
        scanner_pin_test_begin();
        s_phase = SCAN_PHASE_PIN_TEST_SDA;
        break;

    case SCAN_PHASE_PIN_TEST_SDA:
    case SCAN_PHASE_PIN_TEST_SCL:
    case SCAN_PHASE_PIN_TEST_RELEASE:
        if (scanner_pin_test_tick())
        {
            s_scan_addr = 0x08U;
            s_phase = SCAN_PHASE_SCAN_ADDR;
        }
        break;

    case SCAN_PHASE_SCAN_ADDR:
        sw = SwI2c_ProbeAddress(s_scan_addr);
        if (sw == SW_I2C_ACK)
        {
            s_result.total_responding++;
            if (s_result.address_count < I2C_SCANNER_MAX_FOUND)
            {
                s_result.addresses[s_result.address_count++] = s_scan_addr;
            }
        }
        else if (sw == SW_I2C_TIMEOUT)
        {
            s_result.recovery_attempted = 1U;
            if (SwI2c_RecoverBus() == DIAG_OK)
            {
                s_result.recovery_ok = 1U;
            }
        }

        s_scan_addr++;
        if (s_scan_addr > 0x77U)
        {
            s_phase = SCAN_PHASE_CHIP_ID;
        }
        break;

    case SCAN_PHASE_CHIP_ID:
        s_result.bme280_id_ok = 0U;
        if (SwI2c_ReadRegister(BME280_I2C_ADDR_7BIT, 0xD0U, &s_result.bme280_chip_id) == SW_I2C_ACK)
        {
            s_result.bme280_id_ok = 1U;
            s_result.bme280_is_bme = (s_result.bme280_chip_id == BME280_CHIP_ID);
        }
        s_phase = SCAN_PHASE_REINIT_HW;
        break;

    case SCAN_PHASE_REINIT_HW:
        SwI2c_GetStats(&s_result.comm_stats);
        MX_I2C1_Init();
        s_result.scan_complete = 1U;
        s_result.state = MODULE_STATE_DONE;
        s_phase = SCAN_PHASE_DONE;
        break;

    case SCAN_PHASE_DONE:
    default:
        break;
    }
}

uint8_t I2cScanner_IsBusy(void)
{
    return (s_phase != SCAN_PHASE_IDLE) && (s_phase != SCAN_PHASE_DONE);
}

void I2cScanner_GetResult(I2cScanner_Result *out)
{
    if (out != NULL)
    {
        *out = s_result;
    }
}

void I2cScanner_GetPinTest(I2cScanner_PinTest *out)
{
    if (out != NULL)
    {
        *out = s_pin_test;
    }
}
