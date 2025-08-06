/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Authors: Aditya Gadhave
  * CDAC ACTS,pune DESD(June 2025)
  * @file           : main.c
  * @brief          : Main program body - Integrated Temperature and Single Ultrasonic Sensor to CAN Transmitter
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed by STMicroelectronics under BSD 3-Clause license.
  *
  * This software is provided "AS IS" without warranty of any kind, either express or implied,
  * including, but not limited to, the implied warranties of merchantability, fitness for a particular purpose,
  * and non-infringement.
  * In no event shall STMicroelectronics be liable for any direct, indirect, incidental, special, exemplary, or
  * consequential damages (including, but not limited to, procurement of substitute goods or services; loss of use, data,
  * or profits; or business interruption) however caused and on any theory of liability, whether in contract, strict
  * liability, or tort (including negligence or otherwise) arising in any way out of the use of this software,
  * even if advised of the possibility of such damage.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // For sprintf and printf (if using SWV/UART for debug)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// CAN IDs for different sensor data
#define CAN_ID_TEMPERATURE 0x100 // Updated CAN ID for Temperature

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

// Temperature Sensor Variables
volatile float tempC = 0.0f;     // Stores the calculated temperature in Celsius (float)
volatile float adcval = 0.0f;    // Stores the raw ADC conversion value (float for calculation)
int temp_int = 0;                // Stores the integer part of the temperature for CAN transmission

// CAN Transmission Variables
CAN_TxHeaderTypeDef TxHeader;    // CAN Data Frame header fields (used for both temperature and distance)
uint8_t TxData[8];               // Actual Payload (8 bytes, max for CAN)
uint32_t TxMailbox;              // Buffer for Tx Messages

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

void LED_Display();


// Sensor reading and CAN transmission functions
void Read_Temperature(void);
void Send_Temp_CAN(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* µs delay using DWT cycle counter */
/* 10-µs pulse on PA0 */
// #define ECHO_TIMEOUT 100 // Timeout for echo in milliseconds (not directly used in this blocking loop, but good for reference)
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */


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
	  // Read and send temperature data
	  Read_Temperature();
	  Send_Temp_CAN();
	  HAL_Delay(500); // Delay after temperature transmission

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Set to Alternate Function Push-Pull
  GPIO_InitStruct.Pull = GPIO_NOPULL;     // No pull-up/down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // IMPORTANT: Set correct Alternate Function for TIM2_CH2 on PB3
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Reads the temperature from the LM35 sensor via ADC.
  * @param  None
  * @retval None
  */
void Read_Temperature(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adcval = HAL_ADC_GetValue(&hadc1);
        // Convert ADC value to temperature in Celsius
        // Assuming 3.3V reference and 10mV/°C sensor sensitivity (0.01V/°C)
        tempC = (adcval * 3.3f / 4095.0f) / 0.01f;
        temp_int = (int)tempC;
    }
    HAL_ADC_Stop(&hadc1);
}

/**
  * @brief  Sends the current temperature data over CAN.
  * @param  None
  * @retval None
  */
void Send_Temp_CAN(void) {
    // Configure TxHeader for temperature message
    TxHeader.StdId = CAN_ID_TEMPERATURE; // Use specific ID for temperature (0x100)
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4; // Data Length Code: 4 bytes for integer temperature
    TxHeader.TransmitGlobalTime = DISABLE;

    // Prepare 4-byte signed integer data to transmit
    TxData[0] = (uint8_t)(temp_int & 0xFF);         // LSB
    TxData[1] = (uint8_t)((temp_int >> 8) & 0xFF);
    TxData[2] = (uint8_t)((temp_int >> 16) & 0xFF);
    TxData[3] = (uint8_t)((temp_int >> 24) & 0xFF); // MSB

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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET); // Turn on Red LED for error
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
