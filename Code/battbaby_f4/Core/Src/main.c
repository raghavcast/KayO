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
#include "BQ27441_Definitions.h"
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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BQ27441_ctx_t BQ27441 = {
            .BQ27441_i2c_address = BQ27441_I2C_ADDRESS,
            .read_reg = BQ27441_i2cReadBytes,
            .write_reg = BQ27441_i2cWriteBytes,
};

volatile int bat_charging = 0;
int16_t charge = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
/*
 	function writes to slave.
 	returns HAL_OK if write successful
 	uint16_t memAddress: memory address within the BQ27441 where the data should start being written.
	uint8_t *pData: pointer to the array of data bytes that are to be written to the device.
	uint16_t Size:  number of bytes to write.
*/
HAL_StatusTypeDef BQ27441_i2cWriteBytes(uint16_t memAddress, uint8_t *pData, uint16_t Size) {
    return HAL_I2C_Mem_Write(&hi2c3, BQ27441_I2C_ADDRESS << 1, memAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY);
}
/*
 	function read from slave.
 	returns HAL_OK if read successful
	uint16_t memAddress: memory address within the BQ27441 from which the data should start being read.
	uint8_t *pData: pointer to the buffer where the read data will be stored.
	uint16_t Size: number of bytes to read.
*/
HAL_StatusTypeDef BQ27441_i2cReadBytes(uint16_t memAddress, uint8_t *pData, uint16_t Size) {
    return HAL_I2C_Mem_Read(&hi2c3, BQ27441_I2C_ADDRESS << 1, memAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY);
}
/*
	function uses BQ27441_i2cReadBytes to read strictly two bytes of data
	returns a combined 16 bit word.
	uint16_t subAddress: The sub-address or register address within the BQ27441 device from which the 16-bit word will be read.
*/
uint16_t BQ27441_readWord(uint16_t subAddress) {
    uint8_t data[2];
    BQ27441_i2cReadBytes(subAddress, data, 2);
    return ((uint16_t) data[1] << 8) | data[0];
}
/*
	function gets FILTERED/UNFILTERED SoC using BQ27441_readWord
	returns 16 bit SoC value.
	soc_measure type: enumeration value that specifies filtered or unfiltered SoC value should be read. (filtered is basically smoothed out value<-datasheet said so)
*/
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
/*
	function gets Voltage using BQ27441_readWord
	returns 16 bit voltage value.
*/
uint16_t BQ27441_voltage(void) {
    return BQ27441_readWord(BQ27441_COMMAND_VOLTAGE);
}
/*
	function gets power using BQ27441_readWord
	returns 16 bit power value.
*/
int16_t BQ27441_power(void) {
    return (int16_t) BQ27441_readWord(BQ27441_COMMAND_AVG_POWER);
}

int16_t BQ27441_opconfig(void) {
    return (int16_t) BQ27441_readWord(BQ27441_EXTENDED_OPCONFIG);
}

uint16_t BQ27441_deviceType(void) {
	// command sent to slave to tell it master wants to know devicetype
	// 0xFF to get right 8 bits of BQ27441_CONTROL_DEVICE_TYPE
	// >>8 to get left 8 bits, discarding right 8 bits of BQ27441_CONTROL_DEVICE_TYPE
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
// function to initialize battery, check if device type matches
bool BQ27441_init(BQ27441_ctx_t *dev) {
    if (dev == NULL)
        return false;
//    ctx.read_reg = dev->read_reg;
//    ctx.write_reg = dev->write_reg;
//    ctx.BQ27441_i2c_address = dev->BQ27441_i2c_address;

    if (BQ27441_deviceType() == BQ27441_DEVICE_ID) {
        return true;
    } else
        return false;
}

void Bat_init(BQ27441_ctx_t * dev) {
	BQ27441_init(dev);
	check_charging();
	charge = BQ27441_soc(FILTERED);
	HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
	if (bat_charging && charge < 99) {
		HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
	}
}

void check_charging(void) {
	uint16_t pow = BQ27441_power();
//	printf("pow: %d\r\n", pow);
	if (pow > 12000) {
		bat_charging = 0;
	}
	else {
		bat_charging = 1;
	}
}

void updateBat(void) {
	charge = BQ27441_soc(FILTERED);
	printf("lcd charge: %d\r\n", charge);
	if (bat_charging){
		if (charge >= 99) {
			HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);
		}
	}

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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Making sure charging is disabled.
  HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_SET);

//  printf("Starting I2C device scan...\r\n");
//  	HAL_StatusTypeDef res;
//  	uint8_t receiveBuffer[1];
//  	for(uint16_t i = 0; i < 128; i++) {
//  		res = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)i << 1, receiveBuffer, sizeof(receiveBuffer), HAL_MAX_DELAY);
//  		if(res == HAL_OK) {
//  			printf("Device found at address 0x%02X\r\n", i);
//  			fflush(stdout);
//  			break;
//  		} else {
//  			printf(" - ");
//  			fflush(stdout);
//  		}
//  	}

  	printf("\r\n~*BATTBABY*~\r\n");
  	// Checking for connection
  	HAL_StatusTypeDef res;
  	uint8_t receiveBuffer[1];
  	while (1) {
  		if (HAL_I2C_Master_Receive(&hi2c3, (uint16_t) BQ27441_I2C_ADDRESS << 1, receiveBuffer, sizeof(receiveBuffer), HAL_MAX_DELAY) == HAL_OK) {
  			printf("Device found\r\n");
  			break;
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
    	HAL_Delay(100);
  	  printf("BQ27441 initialization successful.\r\n");
  	  uint16_t socValue = BQ27441_soc(FILTERED);
  	    printf("State of Charge: %u%%\r\n", BQ27441_soc(UNFILTERED));
  	    printf("Voltage: %dmV\r\n", BQ27441_voltage());
  	    uint16_t power = BQ27441_power();
  	    printf("Power: %umAh\r\n", power);
    } else {
  	  printf("BQ27441 initialization failed.\r\n");
    }
    check_charging();
    printf("Charging: %d\r\n", bat_charging);
    printf("%d\r\n", BQ27441_opconfig() & BQ27441_OPCONFIG_BATLOWEN);




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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_CE_Pin */
  GPIO_InitStruct.Pin = BAT_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_GPOUT_Pin */
  GPIO_InitStruct.Pin = BAT_GPOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BAT_GPOUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	 if ((GPIO_Pin == B1_Pin || BAT_GPOUT_Pin)) {
		 updateBat();
	 }
	 else {
		 __NOP();
	 }
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