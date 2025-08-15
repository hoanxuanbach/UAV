#ifndef DSHOTPWM_H
#define DSHOTPWM_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>

/*
 * Ý tưởng của thư viện: Sử dụng chế độ sinh xung PWM của TIM để sinh ra xung DSHOT.
 * 1 bit trong xung DSHOT dù là 0 hay 1 thì đều có độ dài như nhau (VD: DSHOT 300 có độ dài xung là 3.333us)
 * Bit 0 và 1 khác nhau ở điểm phần TIME LOW. Với bit 1, phần này rộng hơn; với bit 0 phần này rộng 1,25us
 * Code điều khiển giá trị của thanh ghi CCR. Khi TIM đếm đến số của thanh ghi này, xung PWM sinh ra chuyển từ high->low.
 * Các giá trị của thanh ghi CCR được điều khiển bởi DMA theo chế độ memory-to-peripheral.
 * Code sử dụng 2 buffer để nạp dữ liệu và cập nhật dữ liệu vào DMA.
 */

#define DMA_BUFFER_LENGTH 32
#define MEM_BUFFER_LENGTH 16
#define BIT_0_CCR_REG_VALUE 120
#define BIT_1_CCR_REG_VALUE 240

extern bool Dshot_swapFlag;

// Khởi tạo nội dung buffer dùng cho DMA
void Dshot_MemoryBuffer_init(uint32_t *memoryBuffer);
void Dshot_DMABuffer_init(uint32_t *dmaBuffer);

// Chuyển giá trị throttle thành frame DSHOT (16 bit)
uint16_t Dshot_CalculateCRCandTelemtryBit(uint16_t value);
uint16_t Dshot_GetDshotFrame(uint16_t value);

// Nạp frame DSHOT vào buffer
void Dshot_DshotFrame_to_buffer(uint16_t dshotFrame, uint32_t *memoryBuffer);
void Dshot_Calibrate(uint32_t *memoryBuffer);
void Dshot_PrepareFrame(uint16_t throttleValue, uint32_t *memoryBuffer);

#endif
