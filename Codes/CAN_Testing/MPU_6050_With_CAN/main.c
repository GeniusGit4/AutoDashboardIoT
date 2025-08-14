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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR       (0x68 << 1)      // datasheet page 33

/*The MPU-60X0 always operates as a slave device when communicating to the system processor, which thus
acts as the master. SDA and SCL lines typically need pull-up resistors to VDD. The maximum bus speed is
400 kHz.
The slave address of the MPU-60X0 is b110100X which is 7 bits long. The LSB bit of the 7 bit address is
2
determined by the logic level on pin AD0. This allows two MPU-60X0s to be connected to the same I C bus.
When used in this configuration, the address of the one of the devices should be b1101000 (pin AD0 is logic
low) and the address of the other should be b1101001 (pin AD0 is logic high).

STM32 HAL uses 8-bit addressing format internally:

    The 7-bit address is shifted left by 1

    The LSB is automatically set to 0 (write) or 1 (read) depending on the operation
*/

#define MPU6050_REG_ACCEL  0x3B
#define ACCEL_SCALE        2048.0f
#define JERK_THRESHOLD     20.0f
#define LED_PIN            GPIO_PIN_13
#define LED_GPIO_PORT      GPIOD
#define LED_BLINK_INTERVAL 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8]; // To hold float data
uint32_t TxMailbox;
uint8_t rawData[14];
int16_t accelX_raw, accelY_raw, accelZ_raw;
float accelX_g, accelY_g, accelZ_g;
float prevAccelX_g = 0.0f, prevAccelY_g = 0.0f, prevAccelZ_g = 0.0f;
float jerkX, jerkY, jerkZ;
uint32_t prevTime = 0, lastBlinkTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
//void I2C_Scanner(void);
void MPU6050_Init(void);
void MPU6050_ReadAccel(void);
void CalculateJerkAndHandleAlert(void);
void SendJerkOverCAN(float jerk, uint8_t axisId);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //I2C_Scanner();
  MPU6050_Init();
  prevTime = HAL_GetTick();


  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x123 << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
      Error_Handler();
  }

  TxHeader.StdId = 0x123;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
 // TxHeader.DLC = 8; // Sending 3 floats: 4+4+4 = 12 bytes
  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      MPU6050_ReadAccel();
      CalculateJerkAndHandleAlert();
      HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MPU6050_Init(void)
{
    uint8_t data;

    data = 0x00;  // Wake up   register map pdf 41
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x00;  // Sample rate = 1kHz / (1 + 7) = 125Hz    register map pdf 21
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x19, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x03;  // DLPF 42Hz  register map pdf 13
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x00;  // Gyro ±250°/s   register map pdf 14
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x18;  // Accel ±16g   register map pdf 15
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        Error_Handler(); // Add a breakpoint or toggle an LED here
    }

}

/* Read Accelerometer --------------------------------------------------------*/
void MPU6050_ReadAccel(void)
{
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, MPU6050_REG_ACCEL, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

    accelX_raw = (int16_t)((rawData[0] << 8) | rawData[1]);
    accelY_raw = (int16_t)((rawData[2] << 8) | rawData[3]);
    accelZ_raw = (int16_t)((rawData[4] << 8) | rawData[5]);

    accelX_g = accelX_raw / ACCEL_SCALE;
    accelY_g = accelY_raw / ACCEL_SCALE;
    accelZ_g = accelZ_raw / ACCEL_SCALE;
}

void SendJerkOverCAN(float jerk, uint8_t axisId)
{


    // Put the float jerk into first 4 bytes
	memcpy(TxData, &jerk, sizeof(float));
    // Use axisId in 5th byte to distinguish axes: 0=X, 1=Y, 2=Z
    TxData[4] = axisId;
    TxHeader.DLC = 5;  // Only 5 bytes used



    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        Error_Handler();
    }
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
}


/* Calculate Jerk and Handle Alert -------------------------------------------*/
void CalculateJerkAndHandleAlert(void)
{
    uint32_t currentTime = HAL_GetTick();
    float deltaTime = (currentTime - prevTime) / 1000.0f;

    if (deltaTime <= 0.0f)
        return;

    jerkX = (accelX_g - prevAccelX_g) / deltaTime;
    jerkY = (accelY_g - prevAccelY_g) / deltaTime;
    jerkZ = (accelZ_g - prevAccelZ_g) / deltaTime;

    if (fabs(jerkX) > JERK_THRESHOLD ||
        fabs(jerkY) > JERK_THRESHOLD ||
        fabs(jerkZ) > JERK_THRESHOLD)
    {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);

        SendJerkOverCAN(jerkX, 0);  // 0 for X
        SendJerkOverCAN(jerkY, 1);  // 1 for Y
        SendJerkOverCAN(jerkZ, 2);  // 2 for Z
    }

    prevAccelX_g = accelX_g;
    prevAccelY_g = accelY_g;
    prevAccelZ_g = accelZ_g;
    prevTime = currentTime;
}


/*
void I2C_Scanner(void)
{
    for (uint8_t i = 1; i < 128; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2, i << 1, 1, 10) == HAL_OK)
        {
            // Device found
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);  // Blink LED to indicate found
            HAL_Delay(1000);
        }
    }
}
*/
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
