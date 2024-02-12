/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @fil           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE fil
  * in the root directory of this software component.
  * If no LICENSE fil comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "ili9341.h"
#include <stdio.h>
#include <string.h>
#include "fonts.h"
#include "integer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Image defines
#define SPIbuffSize 264
#define BUFFER_SIZE 1032
#define BITMAP_HEADER_SIZE sizeof(BmpHeader)

// Image position defines
#define LOGO_X 10
#define LOGO_Y 60
#define STATUS_X 200
#define STATUS_Y 120
#define BATTERY_X 200
#define BATTERY_Y 60

// image filename defines
#define BLUETOOTH "blue.bmp"
#define USB "usb.bmp"
#define BATTERY_FULL "full.bmp"
#define BATTERY_HALF "half.bmp"
#define BATTERY_LOW "low.bmp"
#define BATTERY_CHARGE "charg.bmp"
#define LOGO "logo.bmp"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a, b) (((a)<(b))?(a): (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//0x05, 0x01, // USAGE_PAGE (Generic Desktop)
//0x09, 0x05, // USAGE (Game Pad)
//0xA1, 0x01, // COLLECTION (Application)
//	0xA1, 0x00, // COLLECTION (Physical)
//		0x05, 0x09, // USAGE_PAGE (Button)
//		0x19, 0x01, // USAGE_MINIMUM (Button 1)
//		0x29, 0x15, // USAGE_MAXIMUM (Button 21)
//		0x15, 0x00, // LOGICAL_MINIMUM (0)
//		0x25, 0x01, // LOGICAL_MAXIMUM (1)
//		0x95, 0x15, // REPORT_COUNT (21)
//		0x75, 0x01, // REPORT_SIZE (1)
//		0x81, 0x02, // INPUT (Data, Var, Abs)
//		0x95, 0x01, // REPORT_COUNT (1)
//		0x75, 0x03, // REPORT_SIZE (3)
//		0x81, 0x07, // INPUT (Cnst, Var, Rel)
//	0xC0, // END_COLLECTION
//0xC0 // END_COLLECTION

// ILI9341 Variables
int16_t x, y;
char buff[64];
uint16_t z=15;

// SD Card variables
FATFS fs;
FIL fil;
FRESULT fres;

// Image variables
typedef struct BmpHeader
{
uint8_t B;
uint8_t M;
uint32_t fsize;
uint16_t res1;
uint16_t res2;
uint32_t offset;
uint32_t hsize;
uint32_t w;
uint32_t h;
uint16_t planes;
uint16_t bpp;
uint32_t ctype;
uint32_t dsize;
uint32_t hppm;
uint32_t vppm;
uint32_t colorsused;
uint32_t colorreq;
}BmpHeader;

// Battery monitor variables
volatile int bat_percent = 10;  // May not need to be volatile
int bat_charging = 0;

// USB variables
extern USBD_HandleTypeDef hUsbDeviceFS;
volatile int col = 0;
uint8_t buffer[] = {0, 0, 0};
uint8_t cleaned_buffer[] = {0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void dispBat(void);
void buttons_init(void);
void drive_column(void);
uint8_t read_rows(void);
void clean_buffer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Serial debug
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// Image display
int displayImage(const char*fname, uint32_t startx, uint32_t starty) {
	    FRESULT res = f_open(&fil, fname, FA_READ);
	    if(res != FR_OK) {
	    	printf("f_open() failed, res = %d\r\n", res);
	        return -1;
	    }

	    printf("File opened, reading...\r\n");

	    unsigned int bytesRead;
	    uint8_t header[34];
	    res = f_read(&fil, header, sizeof(header), &bytesRead);
	    if(res != FR_OK) {
	    	printf("f_read() failed: %i\r\n", res);
	        f_close(&fil);
	        return -2;
	    }

	    if((header[0] != 0x42) || (header[1] != 0x4D)) {
	    	printf("Wrong BMP signature\r\n");
	        f_close(&fil);
	        return -3;
	    }

	    uint32_t imageOffset = header[10] | (header[11] << 8) | (header[12] << 16) | (header[13] << 24);
	    uint32_t imageWidth = header[18] | (header[19] << 8) | (header[20] << 16) | (header[21] << 24);
	    uint32_t imageHeight = header[22] | (header[23] << 8) | (header[24] << 16) | (header[25] << 24);
	    uint16_t imagePlanes = header[26] | (header[27] << 8);
	    uint16_t imageBitsPerPixel = header[28] | (header[29] << 8);
	    uint32_t imageCompression = header[30] | (header[31] << 8) | (header[32] << 16) | (header[33] << 24);

	    if((imagePlanes != 1) || (imageBitsPerPixel != 24) || (imageCompression != 0)) {
	        f_close(&fil);
	        return -5;
	    }

	    res = f_lseek(&fil, imageOffset);
	    if(res != FR_OK) {
	    	printf("f_lseek() failed, %i", res);
	        snprintf(buff, sizeof(buff), "f_lseek() failed, res = %d\r\n ", res);
	    	ILI9341_WriteString(0, 120, buff, Font_7x10, ILI9341_RED, ILI9341_WHITE);
	        f_close(&fil);
	        return -6;
	    }

	    // row size is aligned to 4 bytes
	    uint8_t imageRow[(imageWidth * 3 + 3) & ~3];
	    for(uint32_t y = starty; y < starty + imageHeight; y++) {
	        uint32_t rowIdx = 0;
	        res = f_read(&fil, imageRow, sizeof(imageRow), &bytesRead);
	        if(res != FR_OK) {
	        	printf("f_read() failed, %i", res);
	            f_close(&fil);
	            return -7;
	        }

	        for(uint32_t x = startx; x < startx + imageWidth; x++) {
	            uint8_t b = imageRow[rowIdx++];
	            uint8_t g = imageRow[rowIdx++];
	            uint8_t r = imageRow[rowIdx++];
	            uint16_t color565 = ILI9341_COLOR565(r, g, b);
	            ILI9341_DrawPixel(x, (2*starty) + imageHeight - y - 1, color565);
	        }
	    }

	    res = f_close(&fil);
	    if(res != FR_OK) {
	    	printf("f_close() failed %i", res);
	        return -8;
	    }

	    return 0;
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
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the screen
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);

  // Let SD card settle
  HAL_Delay(1000);

  // Check if SD card is connected
  if (HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) == GPIO_PIN_RESET) {
	  printf("Please check SD card connection");
	  ILI9341_WriteString(65, 100, "Please check", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
	  ILI9341_WriteString(100,  130,  "SD card", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
	  while (1) {
		  if(HAL_GPIO_ReadPin(SD_Detect_GPIO_Port, SD_Detect_Pin) == GPIO_PIN_SET) {
			  ILI9341_FillScreen(ILI9341_BLACK);
			  ILI9341_WriteString(90, 110, "SD card", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
	          ILI9341_WriteString(75, 130, "connected", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
			  HAL_Delay(1000);
			  break;
		  }
          HAL_Delay(500);
	  }
  }
  ILI9341_FillScreen(ILI9341_BLACK);

  // Mount SD Card
  fres = f_mount(&fs, "", 1);
  if (fres != FR_OK) {
	printf("f_mount error (%i)\r\n", fres);
	while(1);
  }
  displayImage(LOGO, LOGO_X, LOGO_Y);
  // Check USB connection
  if (HAL_GPIO_ReadPin(USB_Detect_GPIO_Port, USB_Detect_Pin) == GPIO_PIN_SET) {
	  // If USB
	  // get battery percentage and status

	  // show USB symbol, battery percentage
	  displayImage(USB, STATUS_X, STATUS_Y);
	  dispBat();

	  // Enable USB timer interrupt
	  HAL_TIM_Base_Start_IT(&htim3);
  }
  else {
	  // If not USB
	  // get battery percentage and status

	  // show bluetooth symbol, battery percentage
	  displayImage(BLUETOOTH, STATUS_X, STATUS_Y);
	  dispBat();

	  // Enable Bluetooth
  }

//  ILI9341_WriteString(80, 120, "LOGO", Font_16x26, ILI9341_WHITE, ILI9341_BLACK);
  // Enable buttons interrupt
  HAL_TIM_Base_Start_IT(&htim2);

  // Enable battery baby timer interrupt

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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Col0_Pin|Col1_Pin|Col2_Pin|SD_CS_Pin
                          |Col3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : USB_Detect_Pin */
  GPIO_InitStruct.Pin = USB_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USB_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Col0_Pin Col1_Pin Col2_Pin Col3_Pin */
  GPIO_InitStruct.Pin = Col0_Pin|Col1_Pin|Col2_Pin|Col3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_Detect_Pin Row0_Pin Row1_Pin Row2_Pin
                           Row3_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin|Row0_Pin|Row1_Pin|Row2_Pin
                          |Row3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : Button17_Pin Button18_Pin */
  GPIO_InitStruct.Pin = Button17_Pin|Button18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void dispBat(void) {
	snprintf(buff, sizeof(buff), "%d", bat_percent);
	if (bat_charging){
		displayImage(BATTERY_CHARGE, BATTERY_X, BATTERY_Y);
		ILI9341_WriteString(BATTERY_X + 15, BATTERY_Y + 3, buff, Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
	}
	else {
		if (bat_percent >= 25) {
			displayImage(BATTERY_FULL, BATTERY_X, BATTERY_Y);
			ILI9341_WriteString(BATTERY_X + 15, BATTERY_Y + 3, buff, Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
		}
		else {
			displayImage(BATTERY_LOW, BATTERY_X, BATTERY_Y);
			ILI9341_WriteString(BATTERY_X + 15, BATTERY_Y + 3, buff, Font_11x18, ILI9341_RED, ILI9341_BLACK);
		}
	}
}

void buttons_init(void) {
	  GPIOB->BSRR = 0xf;
	  GPIOB->BSRR = 0x1 << 16;
}

void drive_column(void) {
	col = (col + 1) & 0x3;
	GPIOB->BSRR = 0xf;
	GPIOB->BSRR = 0x1 << (col + 16);
}

uint8_t read_rows(void) {
	return ~((GPIOB->IDR & 0xf0) >> 4) & 0xf;
}

void clean_buffer(void) {
	cleaned_buffer[0] = buffer[0];
	cleaned_buffer[1] = buffer[1];
	cleaned_buffer[2] = buffer[2] & 0x01;
	if ((cleaned_buffer[0] & 0x20) && (cleaned_buffer[1] & 0x20)) {
		cleaned_buffer[1] &= ~0x20;
	}
	if ((cleaned_buffer[1] & 0x10 && cleaned_buffer[1] & 0x40)) {
		cleaned_buffer[1] &= ~0x50;
	}
	if (buffer[2] & 0x02) {
//		printf("cleaning %X, ", cleaned_buffer[2]);
//		printf("[0], %X, [1], %X, ", cleaned_buffer[0] & 0x20, cleaned_buffer[1] & 0x70);
		cleaned_buffer[2] |= ((cleaned_buffer[0] & 0x20) >> 4) | ((cleaned_buffer[1] & 0x70) >> 2);
//		printf("cleaned %X\r\n", cleaned_buffer[2]);
		cleaned_buffer[0] &= ~0x20;
		cleaned_buffer[1] &= ~0x70;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		uint8_t rows = read_rows();
		int idx = col / 2;
		buffer[idx] &= ~(0xf << ((col % 2) * 4));
		buffer[idx] |= rows << ((col % 2) * 4);
		drive_column();
		buffer[2] = HAL_GPIO_ReadPin(Button17_GPIO_Port, Button17_Pin) | HAL_GPIO_ReadPin(Button18_GPIO_Port, Button18_Pin) << 1;
	}

	if (htim == &htim3) {
		clean_buffer();
		USBD_HID_SendReport(&hUsbDeviceFS, cleaned_buffer, sizeof(cleaned_buffer));
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
  * @brief  Reports the name of the source fil and the source line number
  *         where the assert_param error has occurred.
  * @param  fil: pointer to the source fil name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *fil, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the fil name and line number,
     ex: printf("Wrong parameters value: fil %s on line %d\r\n", fil, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
