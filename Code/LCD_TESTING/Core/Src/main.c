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
#include <string.h>
#include <stdarg.h>
#include "ili9341.h"
#include "fonts.h"
#include "testimg.h"
int16_t x, y;
char buff[64];
uint16_t z=25;

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
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//FIL file; // File
//FILINFO fno;
//FRESULT fresult;  // result
//UINT br, bw;  // File read/write count
//
//typedef struct BmpHeader
//{
//uint8_t B;
//uint8_t M;
//uint32_t fsize;
//uint16_t res1;
//uint16_t res2;
//uint32_t offset;
//uint32_t hsize;
//uint32_t w;
//uint32_t h;
//uint16_t planes;
//uint16_t bpp;
//uint32_t ctype;
//uint32_t dsize;
//uint32_t hppm;
//uint32_t vppm;
//uint32_t colorsused;
//uint32_t colorreq;
//}BmpHeader;
//
//#define SPIbuffSize 264
//#define BUFFER_SIZE 1032
//#define BITMAP_HEADER_SIZE sizeof(BmpHeader)
//#define MIN(a, b) (((a)<(b))?(a): (b))
//
//char buffer[100];
//
//int bufsize (char *buf)
//{
//   int i=0;
//   while (*buf++ != '\0') i++;
//   return i;
//}
//
//int bufclear (void)
//{
//    for (int i=0; i<1024;i++)
//    {
//    	buffer[i]= '\0';
//    }
//}
//
//int displayImage(const char* fname) {
////    UART_Printf("Openning %s...\r\n", fname);
//    FIL file;
//    FRESULT res = f_open(&file, fname, FA_READ);
//    if(res != FR_OK) {
//        ILI9341_WriteString(0, 10, "f_open() failed, res = %d\r\n", Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        return -1;
//    }
//
//    ILI9341_WriteString(0, 20, "File opened, reading...\r\n", Font_7x10, ILI9341_RED, ILI9341_WHITE);
//
//    unsigned int bytesRead;
//    uint8_t header[34];
//    res = f_read(&file, header, sizeof(header), &bytesRead);
//    if(res != FR_OK) {
//        ILI9341_WriteString(0, 30, "f_read() failed, res = %d\r\n", Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        f_close(&file);
//        return -2;
//    }
//
//    if((header[0] != 0x42) || (header[1] != 0x4D)) {
//        ILI9341_WriteString(0, 40, "Wrong BMP signature: 0x%02X 0x%02X\r\n", Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        f_close(&file);
//        return -3;
//    }
//
//    uint32_t imageOffset = header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
//    uint32_t imageWidth = header[18] | (header[19] << 8) | (header[20] << 16) | (header[21] << 24);
//    uint32_t imageHeight = header[22] | (header[23] << 8) | (header[24] << 16) | (header[25] << 24);
//    uint16_t imagePlanes = header[26] | (header[27] << 8);
//    uint16_t imageBitsPerPixel = header[28] | (header[29] << 8);
//    uint32_t imageCompression = header[30] | (header[31] << 8) | (header[32] << 16) | (header[33] << 24);
//
//    snprintf(buff, sizeof(buff), "Pixels offset: %lu\r\n ", imageOffset);
//	ILI9341_WriteString(0, 50, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//    snprintf(buff, sizeof(buff), "WxH: %lux%lu\r\n ", imageWidth, imageHeight);
//	ILI9341_WriteString(0, 60, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//    snprintf(buff, sizeof(buff), "Planes: %d\r\n ", imagePlanes);
//	ILI9341_WriteString(0, 70, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//    snprintf(buff, sizeof(buff), "Bits per pixel: %d\r\n ", imageBitsPerPixel);
//	ILI9341_WriteString(0, 80, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//    snprintf(buff, sizeof(buff), "Compression: %d\r\n ", imageCompression);
//	ILI9341_WriteString(0, 90, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//
//    if((imageWidth != ILI9341_WIDTH) || (imageHeight != ILI9341_HEIGHT)) {
////        UART_Printf("Wrong BMP size, %dx%d expected\r\n", ST7735_WIDTH, ST7735_HEIGHT);
//        ILI9341_WriteString(0, 100, "Wrong BMP size, %dx%d expected\r\n", Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        f_close(&file);
//        return -4;
//    }
//
//    if((imagePlanes != 1) || (imageBitsPerPixel != 24) || (imageCompression != 0)) {
//        ILI9341_WriteString(0, 110, "Unsupported image format\r\n", Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        f_close(&file);
//        return -5;
//    }
//
//    res = f_lseek(&file, imageOffset);
//    if(res != FR_OK) {
//
//        snprintf(buff, sizeof(buff), "f_lseek() failed, res = %d\r\n ", res);
//    	ILI9341_WriteString(0, 120, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        f_close(&file);
//        return -6;
//    }
//
//    // row size is aligned to 4 bytes
//    uint8_t imageRow[(ILI9341_WIDTH * 3 + 3) & ~3];
//    for(uint32_t y = 0; y < imageHeight; y++) {
//        uint32_t rowIdx = 0;
//        res = f_read(&file, imageRow, sizeof(imageRow), &bytesRead);
//        if(res != FR_OK) {
//            snprintf(buff, sizeof(buff), "f_read() failed, res = %d\r\n", res);
//        	ILI9341_WriteString(0, 130, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//            f_close(&file);
//            return -7;
//        }
//
//        for(uint32_t x = 0; x < imageWidth; x++) {
//            uint8_t b = imageRow[rowIdx++];
//            uint8_t g = imageRow[rowIdx++];
//            uint8_t r = imageRow[rowIdx++];
//            uint16_t color565 = ILI9341_COLOR565(r, g, b);
//            ILI9341_DrawPixel(x, imageHeight - y - 1, color565);
//        }
//    }
//
//    res = f_close(&file);
//    if(res != FR_OK) {
////        UART_Printf("f_close() failed, res = %d\r\n", res);
//        snprintf(buff, sizeof(buff), "f_close() failed, res = %d\r\n", res);
//    	ILI9341_WriteString(0, 140, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
//        return -8;
//    }
//
//    return 0;
//}
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
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Unselect();
  ILI9341_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  displayImage("bluetooth.bmp");
//	  HAL_Delay(1000);
		ILI9341_FillScreen(ILI9341_BLACK);
		for(int x = 0; x < ILI9341_WIDTH; x++) {
		    ILI9341_DrawPixel(x, 0, ILI9341_RED);
		    ILI9341_DrawPixel(x, ILI9341_HEIGHT-1, ILI9341_RED);
		}

		for(int y = 0; y < ILI9341_HEIGHT; y++) {
		    ILI9341_DrawPixel(0, y, ILI9341_RED);
		    ILI9341_DrawPixel(ILI9341_WIDTH-1, y, ILI9341_RED);
		}
		HAL_Delay(1000);

		// Check font
		ILI9341_FillScreen(ILI9341_BLACK);
		ILI9341_WriteString(0, 0,"Font_7x10, HELLO", Font_7x10, ILI9341_RED,ILI9341_BLACK);
		ILI9341_WriteString(0,3*10,"Font_11x18,HELLO",Font_11x18,ILI9341_GREEN,ILI9341_BLACK);
		ILI9341_WriteString(0,3*20,"Font_16x26,HELLO",Font_16x26,ILI9341_BLUE,ILI9341_BLACK);

		HAL_Delay(1000);
		ILI9341_InvertColors(true);
		HAL_Delay(1000);
		ILI9341_InvertColors(false);

		HAL_Delay(5000);

		// Check colors
		ILI9341_FillScreen(ILI9341_WHITE);
		ILI9341_WriteString(0, 0, "WHITE", Font_11x18, ILI9341_BLACK, ILI9341_WHITE);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_BLUE);
		ILI9341_WriteString(0, 0, "BLUE", Font_11x18, ILI9341_BLACK, ILI9341_BLUE);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_RED);
		ILI9341_WriteString(0, 0, "RED", Font_11x18, ILI9341_BLACK, ILI9341_RED);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_GREEN);
		ILI9341_WriteString(0, 0, "GREEN", Font_11x18, ILI9341_BLACK, ILI9341_GREEN);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_CYAN);
		ILI9341_WriteString(0, 0, "CYAN", Font_11x18, ILI9341_BLACK, ILI9341_CYAN);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_MAGENTA);
		ILI9341_WriteString(0, 0, "MAGENTA", Font_11x18, ILI9341_BLACK, ILI9341_MAGENTA);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_YELLOW);
		ILI9341_WriteString(0, 0, "YELLOW", Font_11x18, ILI9341_BLACK, ILI9341_YELLOW);
		HAL_Delay(500);

		ILI9341_FillScreen(ILI9341_BLACK);
		ILI9341_WriteString(0, 0, "BLACK", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
		HAL_Delay(500);

		ILI9341_DrawImage((ILI9341_WIDTH - 240)/2,(ILI9341_HEIGHT-240)/2,240,240,
		(const uint16_t*)test_img_240x240);
		HAL_Delay(3000);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_Reset_GPIO_Port, LCD_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_Reset_Pin */
  GPIO_InitStruct.Pin = LCD_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_Reset_GPIO_Port, &GPIO_InitStruct);

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