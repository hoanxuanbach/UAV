/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6000.h"
#include "filter.h"
#include "pid.h"
#include "dshot.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_KP_PITCH_INNER		1.0f
#define PID_KD_PITCH_INNER		0.0f
#define PID_KI_PITCH_INNER		0.0f
#define PID_KP_PITCH_OUTER		5.0f
#define PID_KD_PITCH_OUTER		0.0f
#define PID_KI_PITCH_OUTER		0.0f

#define PID_KP_ROLL_INNER		1.0f
#define PID_KD_ROLL_INNER		0.0f
#define PID_KI_ROLL_INNER		0.0f
#define PID_KP_ROLL_OUTER		5.0f
#define PID_KD_ROLL_OUTER		0.0f
#define PID_KI_ROLL_OUTER		0.0f

#define PID_KP_YAW				5.0f
#define PID_KD_YAW				0.0f
#define PID_KI_YAW				0.0f
#define PID_KP_YAW_RATE			5.0f
#define PID_KD_YAW_RATE			0.0f
#define PID_KI_YAW_RATE			0.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float dt = 0.001f;
float acc_x = 0.0f;
float acc_y = 0.0f;
float acc_z = 0.0f;
float gyro_p = 0.0f;
float gyro_q = 0.0f;
float gyro_r = 0.0f;

float pitch_target = 0.0f;
float roll_target = 0.0f;
float yaw_target = 0.0f;

float pitch,roll,yaw,yawHat_acc_rad;
float roll_rad, pitch_rad, yaw_rad;
Double_PID_Controller PID_Controller_Roll, PID_Controller_Pitch;
PID_Controller PID_Controller_Yaw, PID_Controller_Yaw_Rate;
uint16_t throttle = 1000;

MPU6000 mpu;
float rollHat_acc_rad;
float pitchHat_acc_rad;
IIR_Filter_3D acc_filtered;
IIR_Filter_3D gyro_filtered;

uint32_t DShot_MemoryBufferMotor1[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor1[DMA_BUFFER_LENGTH] = {0};
uint32_t DShot_MemoryBufferMotor2[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor2[DMA_BUFFER_LENGTH] = {0};
uint32_t DShot_MemoryBufferMotor3[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor3[DMA_BUFFER_LENGTH] = {0};
uint32_t DShot_MemoryBufferMotor4[MEM_BUFFER_LENGTH] = {0};
uint32_t DShot_DMABufferMotor4[DMA_BUFFER_LENGTH] = {0};

bool motor_armed;

float m1,m2,m3,m4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI4) {

        mpu.spi_transfer_done=true;
    }
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI4) {
    	mpu.state = 2;
        mpu.spi_transfer_done=true;
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if (!mpu.state)
        {
        	mpu.state = 1;
            MPU6000_Start_DMA(&mpu);
        }
    }
}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            memcpy(DShot_DMABufferMotor1, DShot_MemoryBufferMotor1,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor1[0]));
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            memcpy(DShot_DMABufferMotor2, DShot_MemoryBufferMotor2,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor2[0]));
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            memcpy(DShot_DMABufferMotor3, DShot_MemoryBufferMotor3,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor3[0]));
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            memcpy(DShot_DMABufferMotor4, DShot_MemoryBufferMotor4,
                   MEM_BUFFER_LENGTH * sizeof(DShot_DMABufferMotor4[0]));
        }
    }
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float get_pitch(float Ax, float Ay, float Az) {
    return atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / M_PI;
}

float get_roll(float Ax, float Az) {
    return atan2f(-Ax, Az) * 180.0f / M_PI;
}

void init_PIDs(void)
{
//    PID_Init(&pid_roll,  1.5f, 0.0f, 0.05f, 400.0f, 100.0f);
//    PID_Init(&pid_pitch, 1.5f, 0.0f, 0.05f, 400.0f, 100.0f);
//    PID_Init(&pid_yaw,   2.0f, 0.0f, 0.10f, 400.0f, 100.0f);
	PID_Init(&PID_Controller_Pitch.inner_loop, PID_KP_PITCH_INNER, PID_KI_PITCH_INNER, PID_KD_PITCH_INNER, 400.0f, 100.0f);
	PID_Init(&PID_Controller_Roll.inner_loop, PID_KP_ROLL_INNER, PID_KI_ROLL_INNER, PID_KD_ROLL_INNER, 400.0f, 100.0f);
	PID_Init(&PID_Controller_Yaw, PID_KP_YAW, PID_KI_YAW, PID_KD_YAW, 400.0f, 100.0f);

	PID_Init(&PID_Controller_Pitch.outer_loop, PID_KP_PITCH_OUTER, PID_KI_PITCH_OUTER, PID_KD_PITCH_OUTER, 400.0f, 100.0f);
	PID_Init(&PID_Controller_Roll.outer_loop, PID_KP_ROLL_OUTER, PID_KI_ROLL_OUTER, PID_KD_ROLL_OUTER, 400.0f, 100.0f);
	PID_Init(&PID_Controller_Yaw_Rate, PID_KP_YAW_RATE, PID_KI_YAW_RATE, PID_KD_YAW_RATE, 400.0f, 100.0f);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  IIR_Filter_3D_Init(&acc_filtered, IIR_ACC_ALPHA, IIR_ACC_BETA);
  IIR_Filter_3D_Init(&gyro_filtered, IIR_GYR_ALPHA, IIR_GYR_BETA);
  MPU6000_Init(&mpu, &hspi4);

  mpu.state=0;
  for(int i=0;i<=14;i++) mpu.tx_buffer[i]=0xFF;
  init_PIDs();
  MPU6000_Calibrate(&mpu);
  MPU6000_Start_DMA(&mpu);

  HAL_TIM_Base_Start_IT(&htim2);

  Dshot_DMABuffer_init(DShot_DMABufferMotor1);
  Dshot_DMABuffer_init(DShot_DMABufferMotor2);
  Dshot_DMABuffer_init(DShot_DMABufferMotor3);
  Dshot_DMABuffer_init(DShot_DMABufferMotor4);


  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor1);
  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor2);
  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor3);
  Dshot_MemoryBuffer_init(DShot_MemoryBufferMotor4);

  Dshot_Calibrate(DShot_DMABufferMotor1);
  Dshot_Calibrate(DShot_DMABufferMotor2);
  Dshot_Calibrate(DShot_DMABufferMotor3);
  Dshot_Calibrate(DShot_DMABufferMotor4);

  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1, DShot_DMABufferMotor1, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, DShot_DMABufferMotor2, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, DShot_DMABufferMotor3, DMA_BUFFER_LENGTH);
  HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4, DShot_DMABufferMotor4, DMA_BUFFER_LENGTH);

  HAL_Delay(2000);
  motor_armed = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (!motor_armed){
		  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor1);
		  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor2);
		  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor3);
		  Dshot_PrepareFrame(0, DShot_MemoryBufferMotor4);

	  }
	  else{
		  //0 <= m1, m2, m3, m4 <= 2000
		  Dshot_PrepareFrame(m1, DShot_MemoryBufferMotor1);
		  Dshot_PrepareFrame(m2, DShot_MemoryBufferMotor2);
		  Dshot_PrepareFrame(m3, DShot_MemoryBufferMotor3);
		  Dshot_PrepareFrame(m4, DShot_MemoryBufferMotor4);
	  }

	  if (mpu.state==2){
		  MPU6000_Process_DMA(&mpu);
		  mpu.state = 0;


		  /*low-pass filter*/
		  IIR_Filter_3D_Update(&acc_filtered, mpu.acc[0], mpu.acc[1], mpu.acc[2], &acc_x, &acc_y, &acc_z);
		  IIR_Filter_3D_Update(&gyro_filtered, mpu.gyro[0], mpu.gyro[1], mpu.gyro[2], &gyro_p, &gyro_q, &gyro_r);

		  /*Estimate pitch and roll*/
		  rollHat_acc_rad = atan2f(acc_y, acc_z);
		  pitchHat_acc_rad = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z));

		  float yawDot_rad = gyro_r * (M_PI / 180.0f);
		  float rollDot_rad = (gyro_p * (M_PI / 180.0f) + tanf(pitchHat_acc_rad) * sinf(rollHat_acc_rad) * gyro_q * (M_PI / 180.0f) + tanf(pitchHat_acc_rad) * cosf(rollHat_acc_rad) * gyro_r * (M_PI / 180.0f));
		  float pitchDot_rad = (cosf(rollHat_acc_rad) * gyro_q * (M_PI / 180.0f) - sinf(rollHat_acc_rad) * gyro_r * (M_PI / 180.0f));

		  //Complementary filter
		  roll_rad = (1.0f - COMP_ALPHA) * rollHat_acc_rad + COMP_ALPHA * (roll_rad + rollDot_rad * dt );
		  pitch_rad = (1.0f - COMP_ALPHA) * pitchHat_acc_rad + COMP_ALPHA * (pitch_rad + pitchDot_rad * dt );
		  yaw_rad = yaw_rad + yawDot_rad*dt;

		  while (yaw_rad >= 2.0f * M_PI) yaw_rad -= 2.0f * M_PI;
		  while (yaw_rad < 0.0f)         yaw_rad += 2.0f * M_PI;

		  float yawDot = yawDot_rad * (180.0f / M_PI);
		  float rollDot = rollDot_rad * (180.0f / M_PI);
		  float pitchDot = pitchDot_rad * (180.0f / M_PI);

		  roll = roll_rad * (180.0f / M_PI);
		  pitch = pitch_rad * (180.0f / M_PI);
		  yaw = yaw_rad * (180.0f / M_PI);



		  float roll_out = PID_Double_Calculation(&PID_Controller_Roll, roll_target, roll, rollDot, dt);
		  float pitch_out = PID_Double_Calculation(&PID_Controller_Pitch, pitch_target, pitch, pitchDot, dt);
		  float yaw_out = PID_Yaw_Angle_Calculation(&PID_Controller_Yaw, yaw_target, yaw, yawDot, dt);


		  m1 = throttle + pitch_out - roll_out + yaw_out;
		  m2 = throttle + pitch_out + roll_out - yaw_out;
		  m3 = throttle - pitch_out + roll_out + yaw_out;
		  m4 = throttle - pitch_out - roll_out - yaw_out;


	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
