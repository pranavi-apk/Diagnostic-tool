#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c1; /* I2C2 PB10/PB11 */

void MX_I2C1_Init(void); /* brings up I2C2 */

void I2cHw_PrepareBus(void);
HAL_StatusTypeDef I2cHw_ClearBusyFlagErratum(uint32_t timeout_ms);
HAL_StatusTypeDef I2cHw_ProbeDevice(uint16_t addr_shifted, uint32_t trials, uint32_t timeout_ms);
HAL_StatusTypeDef I2cHw_ReadBme280ChipId(uint8_t *chip_id);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
