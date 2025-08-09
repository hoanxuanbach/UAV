#include "DshotPWM.h"

uint32_t Memory_Buffer[MEM_BUFFER_LENGTH] = {0};
uint32_t DMA_Buffer[DMA_BUFFER_LENGTH] = {0};
uint8_t swapFlag = 0;

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	if (swapFlag) {
		for (int i=0; i<MEM_BUFFER_LENGTH; i++) {
			DMA_Buffer[i]=Memory_Buffer[i];
		}
		swapFlag=0;
	}
}

uint16_t Dshot_CalculateCRCandTelemtryBit(uint16_t value){
	value = value << 1;
	return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
}
uint16_t Dshot_GetDshotFrame(uint16_t value){
	return ((value << 5) |  Dshot_CalculateCRCandTelemtryBit(value));
}


void Dshot_DMABuffer_init(uint32_t *MemoryBuffer){
	for (int i = 0; i < MEM_BUFFER_LENGTH ; i++){
		MemoryBuffer[i] = BIT_0_CCR_REG_VALUE;
	}

}
void Dshot_MemoryBuffer_init(uint32_t *MemoryBuffer){
	for (int i = MEM_BUFFER_LENGTH; i < DMA_BUFFER_LENGTH; i++){
		MemoryBuffer[i] = 0;
	}

}

void Dshot_DshotFrame_to_buffer(uint16_t DshotFrame, uint32_t *MemoryBuffer){
	for (int i = 0; i < MEM_BUFFER_LENGTH; i++)
	{
		MemoryBuffer[MEM_BUFFER_LENGTH - 1 - i] = ((DshotFrame&0x01) ? BIT_1_CCR_REG_VALUE : BIT_0_CCR_REG_VALUE);
		DshotFrame>>=1;
	}
}
