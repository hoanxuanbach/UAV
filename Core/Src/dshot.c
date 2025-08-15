#include "dshot.h"



uint16_t Dshot_CalculateCRCandTelemtryBit(uint16_t value) {
    value = value << 1;
    return ((value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
}

uint16_t Dshot_GetDshotFrame(uint16_t value) {
    return ((value << 5) | Dshot_CalculateCRCandTelemtryBit(value));
}

void Dshot_DMABuffer_init(uint32_t *MemoryBuffer){
	for (int i = 0; i < MEM_BUFFER_LENGTH ; i++){
		MemoryBuffer[i] = BIT_0_CCR_REG_VALUE;
	}

}
void Dshot_MemoryBuffer_init(uint32_t *dmaBuffer){
	for (int i = MEM_BUFFER_LENGTH; i < DMA_BUFFER_LENGTH; i++){
		dmaBuffer[i] = 0;
	}

}
void Dshot_DshotFrame_to_buffer(uint16_t DshotFrame, uint32_t *mem_buffer) {
    for (int i = 0; i < MEM_BUFFER_LENGTH; i++) {
        mem_buffer[MEM_BUFFER_LENGTH - 1 - i] =
            (DshotFrame & 0x01) ? BIT_1_CCR_REG_VALUE : BIT_0_CCR_REG_VALUE;
        DshotFrame >>= 1;
    }
}

void Dshot_Calibrate(uint32_t *mem_buffer) {
    for (int i = 0; i < MEM_BUFFER_LENGTH; i++) {
        mem_buffer[i] = BIT_0_CCR_REG_VALUE;
    }
}

void Dshot_PrepareFrame(uint16_t throttleValue, uint32_t *mem_buffer) {
	throttleValue += 48; //vì Dshot đi từ 48 -> 2047
    uint16_t frame = Dshot_GetDshotFrame(throttleValue);

    for (int i = 0; i < MEM_BUFFER_LENGTH; i++) {
        mem_buffer[MEM_BUFFER_LENGTH - 1 - i] =
            (frame & 0x01) ? BIT_1_CCR_REG_VALUE : BIT_0_CCR_REG_VALUE;
        frame >>= 1;
    }
}
