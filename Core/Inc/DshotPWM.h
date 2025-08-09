#ifndef DSHOTPWM_H
#define DSHOTPWM_H


#include "stm32h7xx_hal.h"

#define DMA_BUFFER_LENGTH 75
#define MEM_BUFFER_LENGTH 16
#define BIT_0_CCR_REG_VALUE	210
#define BIT_1_CCR_REG_VALUE	420

extern uint32_t Memory_Buffer[MEM_BUFFER_LENGTH];
extern uint32_t DMA_Buffer[DMA_BUFFER_LENGTH];
extern uint8_t swapFlag;

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);

void Dshot_MemoryBuffer_init(uint32_t *MemoryBuffer);
void Dshot_DMABuffer_init(uint32_t *MemoryBuffer);

uint16_t Dshot_CalculateCRCandTelemtryBit(uint16_t value);
uint16_t Dshot_GetDshotFrame(uint16_t value);

void Dshot_DshotFrame_to_buffer(uint16_t DshotFrame, uint32_t *MemoryBuffer);

#endif
