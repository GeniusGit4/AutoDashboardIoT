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

//---------------MPU6050--------------------------------------------------------
#include <stdint.h>
#include <math.h>
#include <string.h>


//----------------------FreeRTOS---------------------------------//
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "SEGGER_SYSVIEW.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Adjust these values based on your specific STM32's HCLK frequency
// For 16MHz HCLK, 1us = 16 cycles
#define HCLK_FREQ_MHZ             	(HAL_RCC_GetHCLKFreq() / 1000000)

//----------------------UV--------------------------------------
#define ULTRASONIC_TRIG_PIN       	GPIO_PIN_0
#define ULTRASONIC_TRIG_PORT      	GPIOA

//------------------MPU-------------------------------------------
#define MPU6050_ADDR       (0x68 << 1)      // datasheet page 33
#define MPU6050_REG_ACCEL  0x3B
#define ACCEL_SCALE        2048.0f
#define JERK_THRESHOLD     20.0f
#define LED_PIN            GPIO_PIN_13
#define LED_GPIO_PORT      GPIOD
#define LED_BLINK_INTERVAL 200

//-------------------CAN------------------------------------------
#define CAN_ID_TEMPERATURE 			0x100 // Updated CAN ID for Temperature
#define CAN_ID_DISTANCE    			0x101 // Updated CAN ID for Distance
#define CAN_ID_MPU					0x102 // Updated CAN ID for MPU

//-----------------------Speed-----------------------------------
#define WHEEL_DIAMETER 0.06f
#define CAN_ID_SPEED 0x103
#define PULSES_PER_REV    1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

//----------------------Ultrasonic Sensor Variables----------------------
volatile uint32_t ic_timestamp_rising = 0;
volatile uint32_t ic_timestamp_falling = 0;
volatile uint8_t capture_rising_edge_done = 0; // State machine for echo capture
volatile uint32_t measured_distance_cm = 0;    // Latest raw measurement from ISR (updated by ISR)

//-----------------------MPU-----------------------------------------
uint8_t rawData[14];
int16_t accelX_raw, accelY_raw, accelZ_raw;
float accelX_g, accelY_g, accelZ_g;
float prevAccelX_g = 0.0f, prevAccelY_g = 0.0f, prevAccelZ_g = 0.0f;
float jerkX, jerkY, jerkZ;
uint32_t prevTime = 0, lastBlinkTime = 0;
int data = 0;
//----------------------Temperature Sensor Variables-------------------------------
volatile float tempC = 0.0f;     // Stores the calculated temperature in Celsius (float)
volatile float adcval = 0.0f;    // Stores the raw ADC conversion value (float for calculation)
int temp_int = 0;                // Stores the integer part of the temperature for CAN transmission

//---------------------CAN----------------------
// CAN Transmission Variables
CAN_TxHeaderTypeDef TxHeader;    // CAN Data Frame header fields (used for both temperature and distance)
uint8_t TxData[8];               // Actual Payload (8 bytes, max for CAN)
uint32_t TxMailbox;              // Buffer for Tx Messages


//-------------dummy------------------//

int real_distance=0;

//--------------------speed----------------//
//---- HC-89 Speed Sensor variables ----
volatile uint32_t pulse_count = 0;            // Counts pulses from HC-89 (1 per revolution)
volatile float speed_kph = 0.0f;
uint32_t SPEED_FINAL = 0;
SemaphoreHandle_t speedMutex;                 // Protects speed_kph and SPEED_FINAL
const uint32_t pulses_per_revolution = 1;     // For HC-89, if 1 pulse/rev, otherwise adjust


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//---------------------------UV-------------------------------
void DWT_Init(void);
void DelayMicroseconds(uint32_t us);// Renamed prototype for clarity and consistency
void TriggerUltrasonicMeasurement(void);

//-----------------------MPU6050----------------------------
void MPU6050_Init(void);
void MPU6050_ReadAccel(void);
void CalculateJerkAndHandleAlert(void);
void MPU6050_Jerk(void);
void Speed_Task(void);

//-------------------checking------------------------------
void LED_Display();

//------------------Temperature------------------------------
void Read_Temperature(void);





//---------------------CAN---------------------------------
void Send_TxData_CAN(uint32_t data,int ID);
void CAN_Filter_Config(CAN_HandleTypeDef* hcan);

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
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //------------------UV-------------------------------------------------
  	  DWT_Init(); // Initialize DWT for microsecond delays

  //--------------------MPU6050------------------------------------------
  	  MPU6050_Init();


  //---------------CAN---------------------------------------------------
  // Start the CAN peripheral
     if (HAL_CAN_Start(&hcan1) != HAL_OK)
     {
   	  /* CAN Start Error */
   	  Error_Handler();
     }
     CAN_Filter_Config(&hcan1);

     uint32_t lastMPUTick = HAL_GetTick();
     uint32_t lastUltraTempTick = HAL_GetTick();
     const uint32_t MPU_INTERVAL_MS = 20; // 20ms for MPU
     const uint32_t ULTRA_TEMP_INTERVAL_MS = 250; // 250ms for Ultra/Temp


     // ---- Create mutex/semaphore for speed variables ----
          speedMutex = xSemaphoreCreateMutex();
          assert_param(speedMutex != NULL);


//-----------------freertos---------------------//
     SEGGER_SYSVIEW_Conf();
     HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//ensure proper priority grouping for freeRTOS

     assert_param(xTaskCreate(TriggerUltrasonicMeasurement, "UltraTask", 256, NULL, 2, NULL) == pdPASS);

     assert_param(xTaskCreate(Read_Temperature, "TempTask", 256, NULL, 1, NULL) == pdPASS);

     assert_param(xTaskCreate(MPU6050_Jerk,"MPU6050",256,NULL,0, NULL)==pdPASS);



     // ---- Create the Speed_Task ----
     assert_param(xTaskCreate(Speed_Task, "SpeedTask", 256, NULL, 2, NULL) == pdPASS);



     vTaskStartScheduler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hcan1.Init.Prescaler = 3;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//-----------------------------UV-----------------------------------
/**
 * @brief Initializes the DWT for microsecond delays.
 * Required for DelayMicroseconds function.
 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace and debug block
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable cycle counter
    DWT->CYCCNT = 0; // Reset counter
}

/**
 * @brief Provides a blocking delay in microseconds using DWT.
 * @param us Number of microseconds to delay.
 */
void DelayMicroseconds(uint32_t us)
{
    uint32_t start_cycle = DWT->CYCCNT;
    // Calculate cycles to wait based on HCLK frequency
    uint32_t cycles_to_wait = us * HCLK_FREQ_MHZ; // HCLK_FREQ_MHZ is in MHz, so multiply by us
    while ((DWT->CYCCNT - start_cycle) < cycles_to_wait);
}

/**
 * @brief Triggers a single ultrasonic measurement and waits for its completion.
 * The result is stored in the global 'measured_distance_cm' by the ISR.
 */
void TriggerUltrasonicMeasurement(void) // Renamed function
{
	while(1)
	{
    // Reset timer counter before starting a new measurement
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    capture_rising_edge_done = 0; // Reset state machine for new measurement
    measured_distance_cm = 0;     // Reset the raw measurement before starting

    // Ensure input capture polarity is set to RISING for the start of echo pulse
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

    // Generate trigger pulse
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
    DelayMicroseconds(2);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_SET);
    DelayMicroseconds(10); // 10us pulse
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);

    // Re-initialize counter right before starting capture, crucial for accurate first edge timestamp
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); // Start input capture interrupt

    // --- Timeout mechanism for this specific measurement ---
    // Wait for the ISR to complete a measurement (capture_rising_edge_done becomes 0)
    // or for a timeout.
    uint32_t start_tick = HAL_GetTick();
    uint32_t timeout_ms = 100; // Max expected echo time (e.g., ~17 meters)

    // CORRECTED: Loop while the capture is *in progress* (capture_rising_edge_done is 1)
    // AND before timeout.
    while (capture_rising_edge_done == 1 && (HAL_GetTick() - start_tick) < timeout_ms)
    {
        HAL_Delay(1); // Small delay to avoid excessive busy-waiting
    }

    // If the loop exited due to timeout (capture_rising_edge_done is still 1 and timeout occurred)
    if (capture_rising_edge_done == 1) // Means capture was not completed by ISR
    {
        measured_distance_cm = 0; // Force measured_distance_cm to 0 to indicate invalid reading.
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2); // Stop IC interrupt to prevent phantom triggers
        //printf("Measurement Timeout.\r\n"); // Print timeout message here
    }
    // If the loop exited because capture_rising_edge_done became 0, then measured_distance_cm
    // was updated by the ISR successfully.

    Send_TxData_CAN(real_distance, CAN_ID_DISTANCE);
    vTaskDelay(1500/portTICK_PERIOD_MS);
	}
}


/**
 * @brief TIM2 Interrupt Handler.
 * This function handles input capture events for TIM2 Channel 2.
 * (This should be called from TIM2_IRQHandler in stm32f4xx_it.c)
 */
void TIM2_IRQHandler(void)
{
    // Ensure this is the input capture interrupt for Channel 2
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC2) != RESET)
        {
            // Clear the interrupt flag
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);

            if (capture_rising_edge_done == 0)
            {
                // First edge: Rising edge of echo pulse
                ic_timestamp_rising = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
                capture_rising_edge_done = 1; // Indicate that rising edge is captured
                // Switch to capture falling edge for the next interrupt
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else
            {
                // Second edge: Falling edge of echo pulse
                ic_timestamp_falling = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

                // Calculate pulse duration (in microseconds, as timer tick is 1us)
                uint32_t pulse_duration;
                if (ic_timestamp_falling >= ic_timestamp_rising)
                {
                    pulse_duration = ic_timestamp_falling - ic_timestamp_rising;
                }
                else
                {
                    // Handle timer overflow (unlikely for short ultrasonic pulses but robust)
                    pulse_duration = (htim2.Init.Period + 1 - ic_timestamp_rising) + ic_timestamp_falling;
                }

                // Calculate distance in cm: (time in us * speed of sound in cm/us) / 2
                // Speed of sound approx 0.0343 cm/us at 20°C
                measured_distance_cm = (uint32_t)((double)pulse_duration * 0.0343 / 2.0);
                //Send_TxData_CAN(measured_distance_cm, CAN_ID_DISTANCE);
                real_distance=measured_distance_cm;
                //measured_distance_cm = (uint32_t)((double)pulse_duration * 0.0343 / 2.0) + DISTANCE_OFFSET;
                // Reset state for next measurement (capture complete)
                capture_rising_edge_done = 0;
                // Switch back to capture rising edge for the next *new* measurement cycle
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
                // Disable the input capture interrupt until the next trigger
                HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2); // Use HAL_TIM_IC_Stop_IT for clean disable
            }
        }
    }
}

//--------------------------------------MPU6050---------------------
/**
  * @brief  initialization of the MPU6050 for I2C.
  * @param  None
  * @retval None
  */
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


//----------------- Read Accelerometer --------------------------------------------------------*/
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


//-----------------Calculate Jerk and Handle Alert ---------------------------------
void CalculateJerkAndHandleAlert(void)
{
    uint32_t currentTime = xTaskGetTickCount();
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
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
       // HAL_Delay(100);
       // HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
        data = 55;
        Send_TxData_CAN(data, CAN_ID_MPU);  // 0 for X
       // HAL_Delay(200);

    }
    data=0;
    prevAccelX_g = accelX_g;
    prevAccelY_g = accelY_g;
    prevAccelZ_g = accelZ_g;
    prevTime = currentTime;
}

void MPU6050_Jerk(void)
{
	while(1)
	{
		MPU6050_ReadAccel();
		CalculateJerkAndHandleAlert();
		vTaskDelay(20/portTICK_PERIOD_MS);
	}
}

//-------------------------------------- temperature from the LM35---------------------
/**
  * @brief  Reads the temperature from the LM35 sensor via ADC.
  * @param  None
  * @retval None
  */
void Read_Temperature(void)
{
	while(1)
	{
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK) {
        adcval = HAL_ADC_GetValue(&hadc2);
        // Convert ADC value to temperature in Celsius
        // Assuming 3.3V reference and 10mV/°C sensor sensitivity (0.01V/°C)
        tempC = (adcval * 3.3f / 4095.0f) / 0.01f;
        temp_int = (int)tempC;
    }
    HAL_ADC_Stop(&hadc2);

    Send_TxData_CAN(temp_int, CAN_ID_TEMPERATURE);

    vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}


//---------------------------speed-----------------------------------//

//------ Task for computing and sending speed via CAN ------
void Speed_Task(void)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(750)); // Every 100ms

        uint32_t local_pulse_count;

        taskENTER_CRITICAL();
        local_pulse_count = pulse_count;
        pulse_count = 0;
        taskEXIT_CRITICAL();

        uint32_t rpm = ((local_pulse_count * 10) * 60) / pulses_per_revolution;
        float speed_mps = (3.14159f * WHEEL_DIAMETER * rpm) / 60.0f;
        float new_speed_kph = speed_mps * 3.6f;
        uint32_t new_SPEED_FINAL = (uint32_t)new_speed_kph;

        if (xSemaphoreTake(speedMutex, portMAX_DELAY))
        {
            speed_kph = new_speed_kph;
            SPEED_FINAL = new_SPEED_FINAL/10;
            xSemaphoreGive(speedMutex);
        }

        // Now send over CAN
        Send_TxData_CAN(SPEED_FINAL, CAN_ID_SPEED);
        // Optionally: Blink LED or other status
    }
}

// ---- HC-89 pulse counting via EXTI ----
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_4) // Change pin if using another
    {
        static uint32_t last_interrupt_time = 0;
        uint32_t now = HAL_GetTick();
        if ((now - last_interrupt_time) > 5) // 5ms debounce
        {
            pulse_count++;
            last_interrupt_time = now;
        }
    }

    // If you have other interrupts on EXTI lines, handle here too
}

//--------------------------------------CAN--------------------------------

void CAN_Filter_Config(CAN_HandleTypeDef* hcan)
{
	// Configure CAN Filter (for reception, even if not explicitly receiving, required by HAL)
	     CAN_FilterTypeDef sFilterConfig;
	     sFilterConfig.FilterBank = 0;                     // Use filter bank 0
	     sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Mask mode
	     sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit filter scale
	     sFilterConfig.FilterIdHigh = 0x0000;              // Accept all IDs for simplicity in this example
	     sFilterConfig.FilterIdLow = 0x0000;
	     sFilterConfig.FilterMaskIdHigh = 0x0000;          // Mask all bits (accept all)
	     sFilterConfig.FilterMaskIdLow = 0x0000;
	     sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO 0
	     sFilterConfig.FilterActivation = ENABLE;          // Enable the filter
	     sFilterConfig.SlaveStartFilterBank = 14;          // Not relevant for single CAN

	     if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	     {
	         /* CAN Filter configuration Error */
	         Error_Handler();
	     }

}

/**
  * @brief  Sending the data using CAN.
  * @param  data , Message ID
  * @retval None
  */
void Send_TxData_CAN(uint32_t data,int ID)
{
    // Configure TxHeader for distance message
    TxHeader.StdId = ID; // Use specific ID for distance (0x101)
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4; // Data Length Code: 4 bytes for integer distance
    TxHeader.TransmitGlobalTime = DISABLE;

    // Prepare 4-byte unsigned integer data to transmit
    TxData[0] = (uint8_t)(data & 0xFF);         // LSB
    TxData[1] = (uint8_t)((data >> 8) & 0xFF);
    TxData[2] = (uint8_t)((data >> 16) & 0xFF);
    TxData[3] = (uint8_t)((data >> 24) & 0xFF); // MSB

    /* Add the message to the CAN Tx mailbox for transmission */
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        /* Transmission request Error */
        Error_Handler(); // Red LED will indicate error
    }
    else
    {
    	LED_Display();
    }


    // No LED feedback here, as per your provided code. You can add one if desired.
}

//-------------------------------Checking-----------------------------
void LED_Display()
{
  // This function can be removed or simplified for the transmitter if only one LED is used for TX status
  // Keeping it minimal here as it's not directly used by the Tx logic
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET); // Green LED on (usually)
    HAL_Delay(100); // Briefly show the LED
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET); // Turn off the LED
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */



  /* USER CODE END Callback 1 */
}

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
