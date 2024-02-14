/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"

#include "string.h"
#include "battbaby.h"
#include "stm32f0xx_hal_i2c.h"
#include "BQ27441_Definitions.h"
#include "stddef.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define BATTERY_MONITOR_ADDRESS 0b1010101
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RX_Buffer [1] ;
static BQ27441_ctx_t ctx = {0};
BQ27441_ctx_t BQ27441 = {
            .BQ27441_i2c_address = BQ27441_I2C_ADDRESS,
            .read_reg = BQ27441_i2cReadBytes,
            .write_reg = BQ27441_i2cWriteBytes,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef BQ27441_i2cWriteBytes(uint16_t memAddress, uint8_t *pData, uint16_t Size) {
    return HAL_I2C_Mem_Write(&hi2c1, BQ27441_I2C_ADDRESS << 1, memAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY);
}
HAL_StatusTypeDef BQ27441_i2cReadBytes(uint16_t memAddress, uint8_t *pData, uint16_t Size) {
    return HAL_I2C_Mem_Read(&hi2c1, BQ27441_I2C_ADDRESS << 1, memAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY);
}
uint16_t BQ27441_readWord(uint16_t subAddress) {
    uint8_t data[2];
    BQ27441_i2cReadBytes(subAddress, data, 2);
    return ((uint16_t) data[1] << 8) | data[0];
}

uint16_t BQ27441_soc(soc_measure type) {
    //type = FILTERED;
    uint16_t socRet = 0;
    switch (type) {
        case FILTERED:
            socRet = BQ27441_readWord(BQ27441_COMMAND_SOC);
            break;
        case UNFILTERED:
            socRet = BQ27441_readWord(BQ27441_COMMAND_SOC_UNFL);
            break;
    }
    return socRet;
}
uint16_t BQ27441_voltage(void) {
    return BQ27441_readWord(BQ27441_COMMAND_VOLTAGE);
}
int16_t BQ27441_power(void) {
    return (int16_t) BQ27441_readWord(BQ27441_COMMAND_AVG_POWER);
}
uint16_t BQ27441_deviceType(void) {
    uint8_t command[2] = {BQ27441_CONTROL_DEVICE_TYPE & 0xFF, BQ27441_CONTROL_DEVICE_TYPE >> 8};
    uint8_t data[2];
    uint16_t deviceType = 0;

    BQ27441_i2cWriteBytes(BQ27441_COMMAND_CONTROL, command, sizeof(command)); //write command to slave

    HAL_Delay(10); //delay to ensure device is ready

    if (BQ27441_i2cReadBytes(BQ27441_COMMAND_CONTROL, data, sizeof(data)) == HAL_OK) { //read device type from control register
        deviceType = (data[1] << 8) | data[0];
    }
    return deviceType;
}

bool BQ27441_init(BQ27441_ctx_t *dev) {
    if (dev == NULL)
        return false;

    ctx.read_reg = dev->read_reg;
    ctx.write_reg = dev->write_reg;
    ctx.BQ27441_i2c_address = dev->BQ27441_i2c_address;

    if (BQ27441_deviceType() == BQ27441_DEVICE_ID) {
        return true;
    } else
        return false;
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
  //BQ27441_init(&BQ27441);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//  printf("Attempting to read device type...\n\r");
//  uint16_t deviceType = BQ27441_deviceType();
//  printf("Device Type: 0x%04X\n\r", deviceType);
//  printf("before init\n\r");
//  BQ27441_init(&BQ27441);
//  printf("after init\n\r");
//  printf("Scanning I2C bus...\r\n");
//  fflush(stdout);
//  HAL_StatusTypeDef res;
//  uint8_t receiveBuffer[1];
//  for(uint16_t i = 0; i < 128; i++) {
//      res = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)i << 1, receiveBuffer, sizeof(receiveBuffer), HAL_MAX_DELAY);
//      if(res == HAL_OK) {
//          printf("Device found at address 0x%02X\r\n", i);
//          fflush(stdout);
//      } else {
//          printf("HAL status: %d at 0x%02X\r\n", res, i);
//          fflush(stdout);
//      }
//  }
	printf("Starting I2C device scan...\r\n");
	HAL_StatusTypeDef res;
	uint8_t receiveBuffer[1];
	for(uint16_t i = 0; i < 128; i++) {
		res = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)i << 1, receiveBuffer, sizeof(receiveBuffer), HAL_MAX_DELAY);
		if(res == HAL_OK) {
			printf("Device found at address 0x%02X\r\n", i);
			fflush(stdout);
		} else {
			printf(" - ");
			fflush(stdout);
		}
	}
  printf("Attempting to read BQ27441 device type...\r\n");
  uint16_t deviceType = BQ27441_deviceType();
  if (deviceType != 0) {
	  printf("BQ27441 Device Type: 0x%04X\r\n", deviceType);
  } else {
	  printf("Failed to read BQ27441 device type.\r\n");
  }
  if (BQ27441_init(&BQ27441)) {
	  printf("BQ27441 initialization successful.\r\n");
  } else {
	  printf("BQ27441 initialization failed.\r\n");
  }

  uint16_t socValue = BQ27441_soc(FILTERED);
  printf("State of Charge: %u%%\n", socValue);
  uint16_t voltage = BQ27441_voltage();
  printf("Voltage: %umV\r\n", voltage);
  uint16_t power = BQ27441_power();
  printf("Power: %umAh\r\n", power);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00401E26;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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
