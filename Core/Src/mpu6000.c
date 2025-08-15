#include "mpu6000.h"

uint16_t MPU6000_Read(MPU6000 *dev,uint8_t reg) {
    // Pull CS low to select the device
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

    // Transmit the register address with the read bit set (0x80)
    uint8_t tx[2] = {reg|0x80,0x00};
    uint8_t rx[2] = {0x00,0x00};
    HAL_SPI_TransmitReceive_DMA(dev->hspi,tx,rx,2);

    while (!dev->spi_transfer_done);
    dev->spi_transfer_done=false;
    // Pull CS high to deselect the device
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);

    // Combine the high byte and low byte to form a 16-bit value
    //return (uint16_t)((data[0] << 8) | data[1]);
    return rx[1];
}

void MPU6000_Write(MPU6000 *dev,uint8_t reg,uint8_t data) {
	HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

	// Transmit the register address with the read bit set (0x80)
	dev->tx_buffer[0]=reg;
	dev->tx_buffer[1]=data;
	HAL_SPI_Transmit_DMA(dev->hspi, dev->tx_buffer, 2);

	while (!dev->spi_transfer_done);
	dev->spi_transfer_done=false;

	// Pull CS high to deselect the device
	HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
}


void MPU6000_Init(MPU6000 *dev,SPI_HandleTypeDef *hspi){
	dev->spi_transfer_done=false;
	dev->hspi = hspi;

	dev->acc[0] = 0.0f;
	dev->acc[1] = 0.0f;
	dev->acc[2] = 0.0f;

	HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
	HAL_Delay(100);

	MPU6000_Write(dev,MPU6000_PWR_MGMT_1,0x80);
	HAL_Delay(100);

	MPU6000_Write(dev,MPU6000_PWR_MGMT_1,0x00);
	HAL_Delay(10);
	MPU6000_Write(dev,MPU6000_GYRO_CONFIG,0x00);
	MPU6000_Write(dev,MPU6000_ACCEL_CONFIG,0x00);

	uint8_t whoami = MPU6000_Read(dev,MPU6000_WHO_AM_I);
	printf("Who am I: %d\n",whoami);

	// Pull CS low to select the device
	HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);
}

void MPU6000_Start_DMA(MPU6000 *dev){
    // Pull CS low to select the device
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

    // Transmit the register address with the read bit set (0x80)
    dev->tx_buffer[0]=0x3B|0x80;
    //SCB_CleanDCache_by_Addr((uint32_t*)dev->tx_buffer, 15);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(dev->hspi,dev->tx_buffer,dev->dma_buffer,15);

    while(!dev->spi_transfer_done);
    dev->spi_transfer_done=false;

    //SCB_InvalidateDCache_by_Addr((uint32_t*)dev->dma_buffer, 15);

    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
}

void MPU6000_Process_DMA(MPU6000 *dev) {
    int16_t raw_acc_x = (dev->dma_buffer[1] << 8) | dev->dma_buffer[2];
    int16_t raw_acc_y = (dev->dma_buffer[3] << 8) | dev->dma_buffer[4];
    int16_t raw_acc_z = (dev->dma_buffer[5] << 8) | dev->dma_buffer[6];

    int16_t raw_temp  = (dev->dma_buffer[7] << 8) | dev->dma_buffer[8];

    int16_t raw_gyro_x = (dev->dma_buffer[9] << 8) | dev->dma_buffer[10];
    int16_t raw_gyro_y = (dev->dma_buffer[11] << 8) | dev->dma_buffer[12];
    int16_t raw_gyro_z = (dev->dma_buffer[13] << 8) | dev->dma_buffer[14];

    dev->acc[0] = (float)raw_acc_x / ACCEL_SCALE;   // ±4g scale
    dev->acc[1] = -(float)raw_acc_y / ACCEL_SCALE;
    dev->acc[2] = -(float)raw_acc_z / ACCEL_SCALE;

    dev->temp = ((float)raw_temp) / 340.0f + 36.53f;

    dev->gyro[0] = (float)raw_gyro_x / GYRO_SCALE;   // ±500°/s
    dev->gyro[1] = -(float)raw_gyro_y / GYRO_SCALE;
    dev->gyro[2] = -(float)raw_gyro_z / GYRO_SCALE;

    if (dev->calibrated){
        for (uint8_t i = 0; i < 3; i++){
        	dev->acc[i] -= dev->acc_offset[i];
        	dev->gyro[i] -= dev->gyro_offset[i];
        }
    }
}

void MPU6000_Calibrate(MPU6000 *dev) {
    const uint16_t samples = 3000;

    float acc_sum[3] = {0}, gyro_sum[3] = {0};

    for (uint16_t i = 0; i < samples; i++) {
        // Read one set of sensor values
        MPU6000_Start_DMA(dev);
        MPU6000_Process_DMA(dev);

        for (uint8_t axis = 0; axis < 3; axis++) {
            acc_sum[axis]  += dev->acc[axis];
            gyro_sum[axis] += dev->gyro[axis];
        }

        HAL_Delay(1); // Adjust delay depending on your sampling rate
    }

    // Compute average offsets
    for (uint8_t axis = 0; axis < 3; axis++) {
        dev->acc_offset[axis]  = acc_sum[axis] / samples;
        dev->gyro_offset[axis] = gyro_sum[axis] / samples;
    }

    // For accelerometer Z axis, remove gravity (assuming +Z is upward)
    dev->acc_offset[2] -= 1.0f;

    dev->calibrated = true;


}



