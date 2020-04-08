/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static const uint8_t ASCII[96][5] = { { 0x00, 0x00, 0x00, 0x00, 0x00 } // 20
		, { 0x00, 0x00, 0x5f, 0x00, 0x00 } // 21 !
		, { 0x00, 0x07, 0x00, 0x07, 0x00 } // 22 "
		, { 0x14, 0x7f, 0x14, 0x7f, 0x14 } // 23 #
		, { 0x24, 0x2a, 0x7f, 0x2a, 0x12 } // 24 $
		, { 0x23, 0x13, 0x08, 0x64, 0x62 } // 25 %
		, { 0x36, 0x49, 0x55, 0x22, 0x50 } // 26 &
		, { 0x00, 0x05, 0x03, 0x00, 0x00 } // 27 '
		, { 0x00, 0x1c, 0x22, 0x41, 0x00 } // 28 (
		, { 0x00, 0x41, 0x22, 0x1c, 0x00 } // 29 )
		, { 0x14, 0x08, 0x3e, 0x08, 0x14 } // 2a *
		, { 0x08, 0x08, 0x3e, 0x08, 0x08 } // 2b +
		, { 0x00, 0x50, 0x30, 0x00, 0x00 } // 2c ,
		, { 0x08, 0x08, 0x08, 0x08, 0x08 } // 2d -
		, { 0x00, 0x60, 0x60, 0x00, 0x00 } // 2e .
		, { 0x20, 0x10, 0x08, 0x04, 0x02 } // 2f /
		, { 0x3e, 0x51, 0x49, 0x45, 0x3e } // 30 0
		, { 0x00, 0x42, 0x7f, 0x40, 0x00 } // 31 1
		, { 0x42, 0x61, 0x51, 0x49, 0x46 } // 32 2
		, { 0x21, 0x41, 0x45, 0x4b, 0x31 } // 33 3
		, { 0x18, 0x14, 0x12, 0x7f, 0x10 } // 34 4
		, { 0x27, 0x45, 0x45, 0x45, 0x39 } // 35 5
		, { 0x3c, 0x4a, 0x49, 0x49, 0x30 } // 36 6
		, { 0x01, 0x71, 0x09, 0x05, 0x03 } // 37 7
		, { 0x36, 0x49, 0x49, 0x49, 0x36 } // 38 8
		, { 0x06, 0x49, 0x49, 0x29, 0x1e } // 39 9
		, { 0x00, 0x36, 0x36, 0x00, 0x00 } // 3a :
		, { 0x00, 0x56, 0x36, 0x00, 0x00 } // 3b ;
		, { 0x08, 0x14, 0x22, 0x41, 0x00 } // 3c <
		, { 0x14, 0x14, 0x14, 0x14, 0x14 } // 3d =
		, { 0x00, 0x41, 0x22, 0x14, 0x08 } // 3e >
		, { 0x02, 0x01, 0x51, 0x09, 0x06 } // 3f ?
		, { 0x32, 0x49, 0x79, 0x41, 0x3e } // 40 @
		, { 0x7e, 0x11, 0x11, 0x11, 0x7e } // 41 A
		, { 0x7f, 0x49, 0x49, 0x49, 0x36 } // 42 B
		, { 0x3e, 0x41, 0x41, 0x41, 0x22 } // 43 C
		, { 0x7f, 0x41, 0x41, 0x22, 0x1c } // 44 D
		, { 0x7f, 0x49, 0x49, 0x49, 0x41 } // 45 E
		, { 0x7f, 0x09, 0x09, 0x09, 0x01 } // 46 F
		, { 0x3e, 0x41, 0x49, 0x49, 0x7a } // 47 G
		, { 0x7f, 0x08, 0x08, 0x08, 0x7f } // 48 H
		, { 0x00, 0x41, 0x7f, 0x41, 0x00 } // 49 I
		, { 0x20, 0x40, 0x41, 0x3f, 0x01 } // 4a J
		, { 0x7f, 0x08, 0x14, 0x22, 0x41 } // 4b K
		, { 0x7f, 0x40, 0x40, 0x40, 0x40 } // 4c L
		, { 0x7f, 0x02, 0x0c, 0x02, 0x7f } // 4d M
		, { 0x7f, 0x04, 0x08, 0x10, 0x7f } // 4e N
		, { 0x3e, 0x41, 0x41, 0x41, 0x3e } // 4f O
		, { 0x7f, 0x09, 0x09, 0x09, 0x06 } // 50 P
		, { 0x3e, 0x41, 0x51, 0x21, 0x5e } // 51 Q
		, { 0x7f, 0x09, 0x19, 0x29, 0x46 } // 52 R
		, { 0x46, 0x49, 0x49, 0x49, 0x31 } // 53 S
		, { 0x01, 0x01, 0x7f, 0x01, 0x01 } // 54 T
		, { 0x3f, 0x40, 0x40, 0x40, 0x3f } // 55 U
		, { 0x1f, 0x20, 0x40, 0x20, 0x1f } // 56 V
		, { 0x3f, 0x40, 0x38, 0x40, 0x3f } // 57 W
		, { 0x63, 0x14, 0x08, 0x14, 0x63 } // 58 X
		, { 0x07, 0x08, 0x70, 0x08, 0x07 } // 59 Y
		, { 0x61, 0x51, 0x49, 0x45, 0x43 } // 5a Z
		, { 0x00, 0x7f, 0x41, 0x41, 0x00 } // 5b [
		, { 0x02, 0x04, 0x08, 0x10, 0x20 } // 5c ¥
		, { 0x00, 0x41, 0x41, 0x7f, 0x00 } // 5d ]
		, { 0x04, 0x02, 0x01, 0x02, 0x04 } // 5e ^
		, { 0x40, 0x40, 0x40, 0x40, 0x40 } // 5f _
		, { 0x00, 0x01, 0x02, 0x04, 0x00 } // 60 `
		, { 0x20, 0x54, 0x54, 0x54, 0x78 } // 61 a
		, { 0x7f, 0x48, 0x44, 0x44, 0x38 } // 62 b
		, { 0x38, 0x44, 0x44, 0x44, 0x20 } // 63 c
		, { 0x38, 0x44, 0x44, 0x48, 0x7f } // 64 d
		, { 0x38, 0x54, 0x54, 0x54, 0x18 } // 65 e
		, { 0x08, 0x7e, 0x09, 0x01, 0x02 } // 66 f
		, { 0x0c, 0x52, 0x52, 0x52, 0x3e } // 67 g
		, { 0x7f, 0x08, 0x04, 0x04, 0x78 } // 68 h
		, { 0x00, 0x44, 0x7d, 0x40, 0x00 } // 69 i
		, { 0x20, 0x40, 0x44, 0x3d, 0x00 } // 6a j
		, { 0x7f, 0x10, 0x28, 0x44, 0x00 } // 6b k
		, { 0x00, 0x41, 0x7f, 0x40, 0x00 } // 6c l
		, { 0x7c, 0x04, 0x18, 0x04, 0x78 } // 6d m
		, { 0x7c, 0x08, 0x04, 0x04, 0x78 } // 6e n
		, { 0x38, 0x44, 0x44, 0x44, 0x38 } // 6f o
		, { 0x7c, 0x14, 0x14, 0x14, 0x08 } // 70 p
		, { 0x08, 0x14, 0x14, 0x18, 0x7c } // 71 q
		, { 0x7c, 0x08, 0x04, 0x04, 0x08 } // 72 r
		, { 0x48, 0x54, 0x54, 0x54, 0x20 } // 73 s
		, { 0x04, 0x3f, 0x44, 0x40, 0x20 } // 74 t
		, { 0x3c, 0x40, 0x40, 0x20, 0x7c } // 75 u
		, { 0x1c, 0x20, 0x40, 0x20, 0x1c } // 76 v
		, { 0x3c, 0x40, 0x30, 0x40, 0x3c } // 77 w
		, { 0x44, 0x28, 0x10, 0x28, 0x44 } // 78 x
		, { 0x0c, 0x50, 0x50, 0x50, 0x3c } // 79 y
		, { 0x44, 0x64, 0x54, 0x4c, 0x44 } // 7a z
		, { 0x00, 0x08, 0x36, 0x41, 0x00 } // 7b {
		, { 0x00, 0x00, 0x7f, 0x00, 0x00 } // 7c |
		, { 0x00, 0x41, 0x36, 0x08, 0x00 } // 7d }
		, { 0x10, 0x08, 0x08, 0x10, 0x08 } // 7e �?
		, { 0x78, 0x46, 0x41, 0x46, 0x78 } // 7f →
};

uint8_t Nokia_map[6][84];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void nokia_senddata(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t len) {
	uint16_t i = 0;
	uint8_t dt;
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 1);
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 0);
	while (i < len) {
		dt = data[i] & 0b11111111;
		HAL_SPI_Transmit(hspi, &dt, 1, HAL_MAX_DELAY);
		i++;
	}
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 1);
}

void nokia_sendcmd(SPI_HandleTypeDef *hspi, uint8_t *data, uint16_t len) {
	uint16_t i = 0;
	uint8_t dt;
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 0);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 0);
	while (i < len) {
		dt = data[i];
		HAL_SPI_Transmit(hspi, &dt, 1, HAL_MAX_DELAY);
		i++;
	}
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 1);
}

void nokia_char(uint8_t str, uint8_t *x, uint8_t *y) {
	uint8_t i = 0;
	uint8_t lx = x[0];
	uint8_t ly = y[0];
	while (i < 5) {
		Nokia_map[ly][lx] = ASCII[(str - 0x20)][i];
		lx++;
		i++;
		if (lx >= 84) {
			ly++;
			lx = 0;
			if (ly > 5) {
				ly = 0;
			}
		}
	}
	Nokia_map[ly][lx] = 0x00;
	lx++;
	if (lx >= 84) {
		ly++;
		lx = 0;
		if (ly > 5) {
			ly = 0;
		}
	}
	x[0] = lx;
	y[0] = ly;
}

void nokia_str(uint8_t *str, uint8_t *x, uint8_t *y) {
	uint8_t stri = 0;
//	uint8_t data[2];
	while (str[stri]) {
		nokia_char(str[stri], x, y);
		stri++;
	}
}

void nokia_xy(SPI_HandleTypeDef *hspi, uint8_t x, uint8_t y) {
	uint8_t data[2];
	data[0] = 0b01000000 | y;
	data[1] = 0b10000000 | x;
	nokia_sendcmd(hspi, data, 2);
}

void nokia_init(SPI_HandleTypeDef *hspi) {
	uint16_t i = 0;
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 1);
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 0);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);
	Nokia_map[0][0] = 0x21;
	Nokia_map[0][1] = 0xb9;
	Nokia_map[0][2] = 0x04;
	Nokia_map[0][3] = 0x14;
	Nokia_map[0][4] = 0x20;
	Nokia_map[0][5] = 0x0c;
	nokia_sendcmd(hspi, Nokia_map[0], 6);
	i = 0;
	while (i < 504) {
		Nokia_map[0][i] = 0x00;
		i++;
	}
	nokia_senddata(hspi, Nokia_map[0], 504);
}

void nokia_refresh_map(SPI_HandleTypeDef *hspi) {
	nokia_xy(hspi, 0, 0);
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 1);
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 0);
	HAL_SPI_Transmit_DMA(hspi, Nokia_map[0], 504);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		nokia_refresh_map(&hspi1);
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
	uint8_t x;
	uint8_t y;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	nokia_init(&hspi1);
	HAL_TIM_Base_Start_IT(&htim1);
//	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(100);
	x = 0;
	y = 0;
	nokia_str("Salim", &x, &y);
	x = 0;
	y = 1;
	nokia_str("F4IFB", &x, &y);
	x = 0;
	y = 4;
	//nokia_str("Nokia 5110", &x, &y);

	Nokia_map[3][0] 	= 0b00011111;
	Nokia_map[3][1] 	= 0b00000101;
	Nokia_map[3][2] 	= 0b00000001;
	Nokia_map[3][3] 	= 0b00000000;
	Nokia_map[3][4] 	= 0b00000111;
	Nokia_map[3][5] 	= 0b00000100;
	Nokia_map[3][6] 	= 0b00011111;
	Nokia_map[3][7] 	= 0b00000000;
	Nokia_map[3][8] 	= 0b00010001;
	Nokia_map[3][9] 	= 0b00011111;
	Nokia_map[3][10] 	= 0b00010001;
	Nokia_map[3][11] 	= 0b00000000;
	Nokia_map[3][12] 	= 0b00011111;
	Nokia_map[3][13] 	= 0b00000101;
	Nokia_map[3][14] 	= 0b00000001;
	Nokia_map[3][15] 	= 0b00000000;
	Nokia_map[3][16] 	= 0b00011111;
	Nokia_map[3][17] 	= 0b00010101;
	Nokia_map[3][18] 	= 0b00001010;
	Nokia_map[3][19] 	= 0b00000000;

    Nokia_map[2][0]		= 0b01111111;
	Nokia_map[2][1]		= 0b00101010;
	Nokia_map[2][2]		= 0b00011100;
	Nokia_map[2][3]		= 0b00001000;
	Nokia_map[2][4]		= 0b00001000;
	Nokia_map[2][5]		= 0b00001000;
	Nokia_map[2][6]		= 0b00011000;
	Nokia_map[2][7]		= 0b01100000;
	Nokia_map[2][8]		= 0b10001110;
	Nokia_map[2][9]		= 0b01110001;
	Nokia_map[2][10]	= 0b01001110;
	Nokia_map[2][11]	= 0b10000000;
	Nokia_map[2][12]	= 0b01001110;
	Nokia_map[2][13]	= 0b00110001;
	Nokia_map[2][14]	= 0b01001110;
	Nokia_map[2][15]	= 0b10000000;
	Nokia_map[2][16]	= 0b01001110;
	Nokia_map[2][17]	= 0b00110001;
	Nokia_map[2][18]	= 0b01001110;
	Nokia_map[2][19]	= 0b10000000;
	Nokia_map[2][20]	= 0b01001110;
	Nokia_map[2][21]	= 0b00110001;
	Nokia_map[2][22]	= 0b01001110;
	Nokia_map[2][23]	= 0b10000000;
	Nokia_map[2][24]	= 0b01100000;
	Nokia_map[2][25]	= 0b00011000;
	Nokia_map[2][26]	= 0b00001000;
	Nokia_map[2][27]	= 0b00001000;
	Nokia_map[2][28]	= 0b01111111;
	Nokia_map[2][29]	= 0b00000000;
	Nokia_map[2][30]	= 0b00111110;
	Nokia_map[2][31]	= 0b00000000;
	Nokia_map[2][32]	= 0b00011100;
	Nokia_map[2][33]	= 0b00000000;
	Nokia_map[2][34]	= 0b00001000;
	Nokia_map[2][38]	= 0b00000000;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|DC_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_Pin DC_Pin CE_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
