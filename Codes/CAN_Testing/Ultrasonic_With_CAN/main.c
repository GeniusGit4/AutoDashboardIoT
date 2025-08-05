/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Authors: Aditya Gadhave
  * CDAC ACTS,pune DESD(June 2025)
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
#include "string.h"
#include "stdio.h" // For printf (ensure UART/SWV is configured for output)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//---------- ultrasonic sensor --------------------------------------------
#define ULTRASONIC_TRIG_PIN       GPIO_PIN_0
#define ULTRASONIC_TRIG_PORT      GPIOA
// Adjust these values based on your specific STM32's HCLK frequency
// For 16MHz HCLK, 1us = 16 cycles
#define HCLK_FREQ_MHZ             (HAL_RCC_GetHCLKFreq() / 1000000)
//------------------CAN----------------------------------------------------
#define CAN_ID_DISTANCE    0x101 // Updated CAN ID for Distance

//#define DISTANCE_OFFSET 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

//----------------------ultrasonic sensor------------------------------------
volatile uint32_t ic_timestamp_rising = 0;
volatile uint32_t ic_timestamp_falling = 0;
volatile uint8_t capture_rising_edge_done = 0; // State machine for echo capture
volatile uint32_t measured_distance_cm = 0;    // Latest raw measurement from ISR (updated by ISR)

// ---------------CAN Transmission Variables-----------------------------
CAN_TxHeaderTypeDef TxHeader;    // CAN Data Frame header fields (used for both temperature and distance)
uint8_t TxData[8];               // Actual Payload (8 bytes, max for CAN)
uint32_t TxMailbox;              // Buffer for Tx Messages

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

//----------------------ultrasonic sensor------------------------------------
void DWT_Init(void);
void DelayMicroseconds(uint32_t us);
// Renamed prototype for clarity and consistency
void TriggerUltrasonicMeasurement(void);

// ---------------CAN Transmission -----------------------------
void Send_Distance_CAN(void);

// ---------------Checking transmission-----------------------------
void LED_Display();

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
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  //----------------------ultrasonic sensor------------------------------------
  DWT_Init(); // Initialize DWT for microsecond delays

  // ---------------CAN -----------------------------
  // Start the CAN peripheral
   if (HAL_CAN_Start(&hcan1) != HAL_OK)
   {
 	  /* CAN Start Error */
 	  Error_Handler();
   }

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //----------------------ultrasonic sensor------------------------------------
	  // --- Step 1: Trigger a new ultrasonic measurement periodically ---
	  // This calls the function that pings the sensor and waits for a result in measured_distance_cm.
	  TriggerUltrasonicMeasurement(); // Use the renamed function


	  // --- Step 3: Overall main loop delay ---
	  // This delay controls how often a *new measurement is triggered*.
	  // It should be longer than the maximum echo time + sensor recovery time (e.g., 60ms-100ms minimum for HC-SR04).
	  // A 300ms delay here means you initiate a new ping every 300ms.
	  HAL_Delay(300);

	  // ---------------CAN -----------------------------
	  Send_Distance_CAN();
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//----------------------ultrasonic sensor------------------------------------
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
        printf("Measurement Timeout.\r\n"); // Print timeout message here
    }
    // If the loop exited because capture_rising_edge_done became 0, then measured_distance_cm
    // was updated by the ISR successfully.
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


// ---------------CAN Transmission -----------------------------
void Send_Distance_CAN(void) {
    // Configure TxHeader for distance message
    TxHeader.StdId = CAN_ID_DISTANCE; // Use specific ID for distance (0x101)
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4; // Data Length Code: 4 bytes for integer distance
    TxHeader.TransmitGlobalTime = DISABLE;

    // Prepare 4-byte unsigned integer data to transmit
    TxData[0] = (uint8_t)(measured_distance_cm & 0xFF);         // LSB
    TxData[1] = (uint8_t)((measured_distance_cm >> 8) & 0xFF);
    TxData[2] = (uint8_t)((measured_distance_cm >> 16) & 0xFF);
    TxData[3] = (uint8_t)((measured_distance_cm >> 24) & 0xFF); // MSB

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

// ---------------Checking transmission-----------------------------
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
