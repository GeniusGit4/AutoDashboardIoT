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
// For SWV output: include "itm_debug.h" if you have it, or just use printf
// Ensure you enable SWV in your STM32CubeIDE debug configuration if you plan to use printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */

int sensor_data = 100; // Simulate a 4-byte sensor reading, initialized to an example value

CAN_TxHeaderTypeDef TxHeader; // CAN Data Frame header fields
uint8_t TxData[8]; // Actual Payload
uint32_t TxMailbox; // Buffer for Tx Messages

//CAN_RxHeaderTypeDef RxHeader; // CAN Data Frame header fields for reception
//uint8_t RxData[8];           // Buffer for received payload
//int received_sensor_data;    // Variable to store the reconstructed signed int

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void LED_Display(); // Keep for status LEDs
// *** NEW: CAN Rx Callback declaration ***
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
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
  /* USER CODE BEGIN 2 */

  // Start the CAN peripheral
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  /* Start Error */
	  Error_Handler();
  }

  // *** NEW: Configure CAN Filter for Reception ***
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;                     // Use filter bank 0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // Mask mode
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit filter scale
  sFilterConfig.FilterIdHigh = 0x123 << 5;          // Filter for StdId 0x123 (shifted left by 5 bits)
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x7FF << 5;      // Mask for StdId 0x7FF (all bits care)
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // Assign to FIFO 0
  sFilterConfig.FilterActivation = ENABLE;          // Enable the filter
  sFilterConfig.SlaveStartFilterBank = 14;          // Not relevant for single CAN (can be 0 or any unused bank)

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
      /* Filter configuration Error */
      Error_Handler();
  }

  // *** NEW: Activate CAN Rx notification interrupt ***
  //if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  //{
      /* Notification activation Error */
    //  Error_Handler();
  //}


  /* Configure Transmission process */
    TxHeader.StdId = 0x123; // This is the ID the receiver will listen for
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4; // Data Length Code: 4 bytes
    TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check if the user button (PA0) is pressed
	       if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
	       {
	           HAL_Delay(50); // Debouncing Delay
	           if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) // Confirm button press
	           {
	         	  // Simulate changing signed sensor data
	               sensor_data += 100; // Increment the sensor data for each press (can go positive or negative)
	               // Example to show negative values again, wrap around or specific ranges for testing
	               if (sensor_data > 100000) {
	                   sensor_data = -100000;
	               }


	         	  // Prepare 4-byte signed integer data to transmit
	               // The bitwise operations work the same way for signed or unsigned types
	               // as they operate on the raw binary representation.
	               TxData[0] = (uint8_t)(sensor_data & 0xFF);         // LSB
	               TxData[1] = (uint8_t)((sensor_data >> 8) & 0xFF);
	               TxData[2] = (uint8_t)((sensor_data >> 16) & 0xFF);
	               TxData[3] = (uint8_t)((sensor_data >> 24) & 0xFF); // MSB

	               /* Start the Transmission process */
	               if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	               {
	                   /* Transmission request Error */
	                   Error_Handler(); // Red LED will indicate error
	               }
	               else
	               {
	                   // Successfully queued for transmission: Indicate with green LED
	                   LED_Display(); // This LED will turn on when message is added to Tx mailbox
	                   // No specific LED for successful RX here, it's handled in the callback
	               }

	               // Wait until button is released to prevent multiple transmissions from one press
	               //while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
	               //HAL_Delay(100); // Debounce release
	           }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LED_Display()
{
  // This function can be removed or simplified for the transmitter if only one LED is used for TX status
  // Keeping it minimal here as it's not directly used by the Tx logic
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET); // Green LED on (usually)
    HAL_Delay(100); // Briefly show the LED
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET); // Turn off the LED
}

// *** NEW: CAN Rx Callback Function ***
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        // Reception Error
        Error_Handler(); // This error means a message was pending but couldn't be read.
    }
    else
    {
        // Check if it's our expected ID and DLC
        if (RxHeader.StdId == 0x123 && RxHeader.DLC == 4)
        {
            // Reconstruct the 32-bit signed integer from the 4 received bytes.
            // Assuming little-endian (LSB first) as sent:
            received_sensor_data = (int32_t)(RxData[0] |
                                             ((int32_t)RxData[1] << 8) |
                                             ((int32_t)RxData[2] << 16) |
                                             ((int32_t)RxData[3] << 24));

            // Indicate successful reception and processing with an LED (e.g., Orange LED)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET); // Turn on Orange LED
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET); // Turn off Orange LED

            // You can print this value via SWV (Serial Wire Viewer) if configured in CubeIDE
            // Example: printf("Received Loopback Data: %d\r\n", received_sensor_data);
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
