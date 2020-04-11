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
uint16_t ADC_BUFF = 4200;
uint16_t BUFF_ADC1[4200];
uint16_t BUFF_ADC3[4200];
uint16_t trig = 2048;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC3_Init(void);
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
	Nokia_map[0][1] = 0xb5;
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
	Scope_buff_to_Disp(2048, 0, 1, 0);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		nokia_refresh_map(&hspi1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_TogglePin(ADCint_GPIO_Port, ADCint_Pin);
	if (hadc->Instance == ADC3) {
		Scope_buff_to_Disp(2048, 0, 0, 1);
	}
	HAL_GPIO_TogglePin(ADCint_GPIO_Port, ADCint_Pin);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_TogglePin(ADCint_GPIO_Port, ADCint_Pin);
	if (hadc->Instance == ADC3) {
		Scope_buff_to_Disp(2048, 0, 0, 0);
	}
	HAL_GPIO_TogglePin(ADCint_GPIO_Port, ADCint_Pin);
}

void Scope_buff_to_Disp(uint16_t trig, uint8_t trig_type, uint8_t del,
		uint8_t half) {
	static uint16_t i, x;
	static uint16_t j, y;
	static uint16_t val;
	static uint16_t val2;
	static uint16_t trigger;
	static uint32_t means, meanstrigger;
	static uint32_t samples;
	static uint16_t min, max;
	static uint16_t bfstart, bfstop;
	static uint8_t str[20];
	meanstrigger = 0;
	max = 0;
	min = 65535;
	if (!del) {
		if (half) {
			bfstart = ADC_BUFF / 2;
			bfstop = ADC_BUFF;
		} else {
			bfstart = 0;
			bfstop = ADC_BUFF / 2;
		}

		if (trig_type == 1) {
			j = 0;
			while (j < 128) {
				meanstrigger += BUFF_ADC1[j];
//				if (max < BUFF_ADC1[j]) {
//					max = BUFF_ADC1[j];
//				}
//				if (min > BUFF_ADC1[j]) {
//					min = BUFF_ADC1[j];
//				}
				j++;
			}
			meanstrigger = meanstrigger >> 7;
		}
		switch (trig_type) {
		case 0:
			trigger = trig;
			break;
		case 1:
			trigger = meanstrigger / 10;
			break;
		case 2:
//		Trigger by multiple sample
			break;
		default:
			trigger = trig;
			break;
		}

		j = bfstart + 84;
		samples = 0;
		while (BUFF_ADC1[j] > trigger && (j < (bfstop - 168))) {
			j++;
		}
		while (BUFF_ADC1[j] < trigger && (j < (bfstop - 84))) {
			j++;
		}

		if (j >= bfstop - 84) {
			j = 84;
		}

		j = j - 84;
	}

	i = 0;
	while (i < 84) {
		val = BUFF_ADC1[i + j];
		val2 = BUFF_ADC3[i + j];
		if (del) {
			if ((i % 10) - 2 == 0) {
				Nokia_map[1][i] = 1; //1
			} else {
				Nokia_map[1][i] = 0;
			}
			if ((i % 10) - 2 == 0) {
				Nokia_map[2][i] = 0;
			} else {
				Nokia_map[2][i] = 0;
			}
			if ((i % 10) - 2 == 0) {
				Nokia_map[3][i] = 1;
			} else {
				Nokia_map[3][i] = 0;
			}
			if ((i % 10) - 2 == 0) {
				Nokia_map[4][i] = 128; //128
			} else {
				Nokia_map[4][i] = 0;
			}
		} else {
			Nokia_map[4 - (val >> 10)][i] = Nokia_map[4 - (val >> 10)][i]
					| (1 << (7 - ((val >> 7) & 0b00000111)));
			Nokia_map[4 - (val2 >> 10)][i] = Nokia_map[4 - (val2 >> 10)][i]
					| (1 << (7 - ((val2 >> 7) & 0b00000111)));
		}
		i++;
		means += val;
	}
	means /= 84;
	x = 0;
	y = 5;
	sprintf(str, "means :%dmV", means);
	nokia_str(str, &x, &y);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	nokia_init(&hspi1);
	x = 0;
	y = 0;
	nokia_str("STM32SCOPE", &x, &y);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) BUFF_ADC1, ADC_BUFF);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) BUFF_ADC3, ADC_BUFF);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_Base_Start(&htim8);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable or disable the remapping of ADC1_ETRGREG:
  * ADC1 External Event regular conversion is connected to TIM8 TRG0 
  */
  __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  htim1.Init.Prescaler = 5599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2250;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 42;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADCint_GPIO_Port, ADCint_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|DC_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADCint_Pin */
  GPIO_InitStruct.Pin = ADCint_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADCint_GPIO_Port, &GPIO_InitStruct);

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
