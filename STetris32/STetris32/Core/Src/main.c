/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* NOTAS
 * 1. Tratar como matriz grande de 16x8?
 * 2. Enviar matriz lower y matriz upper aprovechando dirección 0?
 *
 *
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint8_t row_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOFNOTES 	73
#define NOTE_C1     0
#define NOTE_CS1    1
#define NOTE_D1     2
#define NOTE_DS1    3
#define NOTE_E1     4
#define NOTE_F1     5
#define NOTE_FS1    6
#define NOTE_G1     7
#define NOTE_GS1    8
#define NOTE_A1     9
#define NOTE_AS1    10
#define NOTE_B1     11
#define NOTE_C2     12
#define NOTE_CS2    13
#define NOTE_D2     14
#define NOTE_DS2    15
#define NOTE_E2     16
#define NOTE_F2     17
#define NOTE_FS2    18
#define NOTE_G2     19
#define NOTE_GS2    20
#define NOTE_A2     21
#define NOTE_AS2    22
#define NOTE_B2     23
#define NOTE_C3     24
#define NOTE_CS3    25
#define NOTE_D3     26
#define NOTE_DS3    27
#define NOTE_E3     28
#define NOTE_F3     29
#define NOTE_FS3    30
#define NOTE_G3     31
#define NOTE_GS3    32
#define NOTE_A3     33
#define NOTE_AS3    34
#define NOTE_B3     35
#define NOTE_C4     36
#define NOTE_CS4    37
#define NOTE_D4     38
#define NOTE_DS4    39
#define NOTE_E4     40
#define NOTE_F4     41
#define NOTE_FS4    42
#define NOTE_G4     43
#define NOTE_GS4    44
#define NOTE_A4     45
#define NOTE_AS4    46
#define NOTE_B4     47
#define NOTE_C5     48
#define NOTE_CS5    49
#define NOTE_D5     50
#define NOTE_DS5    51
#define NOTE_E5     52
#define NOTE_F5     53
#define NOTE_FS5    54
#define NOTE_G5     55
#define NOTE_GS5    56
#define NOTE_A5     57
#define NOTE_AS5    58
#define NOTE_B5     59
#define NOTE_C6     60
#define NOTE_CS6    61
#define NOTE_D6     62
#define NOTE_DS6    63
#define NOTE_E6     64
#define NOTE_F6     65
#define NOTE_FS6    66
#define NOTE_G6     67
#define NOTE_GS6    68
#define NOTE_A6     69
#define NOTE_AS6    70
#define NOTE_B6     71
#define NOTE_C7     72
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

// Display Module
row_t test_array[][8] = {
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00011100,0b00001000},
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000},
};
row_t figures_array[][8] = {
{0b00010001,0b10101010,0b01000100,0b00000000,0b00000000,0b00100010,0b01010101,0b10001000},
{0b00010001,0b10101010,0b01000100,0b00000000,0b00000000,0b00100010,0b01010101,0b10001000},
};
row_t matrix_buffer[8][8] = {
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000},		// Upper
{0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000}		// Lower
};

uint32_t high_score = 9990;
char high_score_user[15];
char high_score_str[20];

// Sound Module
uint16_t psc_array[] = {424, 403, 291, 211, 266, 232, 285, 289, 212, 195, 197, 137, 149, 136, 114, 193, 108, 113, 100, 84, 101, 81, 112, 137, 65, 68,
57, 81, 53, 64, 44, 42, 113, 81, 56, 34, 119, 34, 52, 77, 53, 32, 22, 21, 26, 21, 28, 17, 41, 17, 26, 77, 14, 16, 11, 14, 13, 21, 14, 9, 11, 9, 13, 10, 7, 8, 6, 7, 7, 6, 7, 5, 4};
uint16_t arr_array[] = {577, 573, 749, 975, 730, 790, 607, 565, 727, 746, 697, 946, 821, 849, 956, 533, 899, 811, 865, 972, 763, 898, 613, 473, 941, 849, 956, 635, 916, 716, 983, 972, 341, 449, 613, 953, 257, 849, 524, 334, 458, 716, 983, 972, 741, 866, 613, 953, 373, 849, 524, 167, 867, 716, 983, 729, 741, 433, 613, 900, 695, 802, 524, 643, 867, 716, 901, 729, 688, 758, 613, 810, 956};
uint8_t array_index = 0;

//notes in the melody:
int melody[] = {
  NOTE_E5, NOTE_E3, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_A3, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_D5, NOTE_E3, NOTE_E5,
  NOTE_E3, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_B2,
  NOTE_C3, NOTE_D3, NOTE_D5, NOTE_F5, NOTE_A5, NOTE_C5, NOTE_C5, NOTE_G5,
  NOTE_F5, NOTE_E5, NOTE_C3, 0, NOTE_C5, NOTE_E5, NOTE_A4, NOTE_G4, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_G4, NOTE_E5,
  NOTE_G4, NOTE_C5, NOTE_E4, NOTE_A4, NOTE_E3, NOTE_A4, 0,
  NOTE_E5, NOTE_E3, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_A3, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_D5, NOTE_E3, NOTE_E5,
  NOTE_E3, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_B2,
  NOTE_C3, NOTE_D3, NOTE_D5, NOTE_F5, NOTE_A5, NOTE_C5, NOTE_C5, NOTE_G5,
  NOTE_F5, NOTE_E5, NOTE_C3, 0, NOTE_C5, NOTE_E5, NOTE_A4, NOTE_G4, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_G4, NOTE_E5,
  NOTE_G4, NOTE_C5, NOTE_E4, NOTE_A4, NOTE_E3, NOTE_A4, 0,
  NOTE_E4, NOTE_E3, NOTE_A2, NOTE_E3, NOTE_C4, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_D4, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_B3, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_C4, NOTE_E3, NOTE_A2, NOTE_E3, NOTE_A3, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_GS3, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_B3, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_E4, NOTE_E3, NOTE_A2, NOTE_E3, NOTE_C4, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_D4, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_B3, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_C4, NOTE_E3, NOTE_E4, NOTE_E3, NOTE_A4, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_GS4, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_E5, NOTE_E3, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_A3, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_D5, NOTE_E3, NOTE_E5,
  NOTE_E3, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_B2,
  NOTE_C3, NOTE_D3, NOTE_D5, NOTE_F5, NOTE_A5, NOTE_C5, NOTE_C5, NOTE_G5,
  NOTE_F5, NOTE_E5, NOTE_C3, 0, NOTE_C5, NOTE_E5, NOTE_A4, NOTE_G4, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_G4, NOTE_E5,
  NOTE_G4, NOTE_C5, NOTE_E4, NOTE_A4, NOTE_E3, NOTE_A4, 0,
  NOTE_E5, NOTE_E3, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_A3, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_C5, NOTE_D5, NOTE_E3, NOTE_E5,
  NOTE_E3, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_A4, NOTE_A3, NOTE_B2,
  NOTE_C3, NOTE_D3, NOTE_D5, NOTE_F5, NOTE_A5, NOTE_C5, NOTE_C5, NOTE_G5,
  NOTE_F5, NOTE_E5, NOTE_C3, 0, NOTE_C5, NOTE_E5, NOTE_A4, NOTE_G4, NOTE_D5,
  NOTE_C5, NOTE_B4, NOTE_E4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_G4, NOTE_E5,
  NOTE_G4, NOTE_C5, NOTE_E4, NOTE_A4, NOTE_E3, NOTE_A4, 0,
  NOTE_E4, NOTE_E3, NOTE_A2, NOTE_E3, NOTE_C4, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_D4, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_B3, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_C4, NOTE_E3, NOTE_A2, NOTE_E3, NOTE_A3, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_GS3, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_B3, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_E4, NOTE_E3, NOTE_A2, NOTE_E3, NOTE_C4, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_D4, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_B3, NOTE_E3, NOTE_GS2, NOTE_E3,
  NOTE_C4, NOTE_E3, NOTE_E4, NOTE_E3, NOTE_A4, NOTE_E3, NOTE_A2, NOTE_E3,
  NOTE_GS4, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_GS2, NOTE_E3, NOTE_GS2, NOTE_E3,
};
int noteDurations[] = {
  8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,4,8,8,16,16,8,8,8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,4,4,
  8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,4,8,8,16,16,8,8,8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,4,4,
  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,4,8,8,16,16,8,8,8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,4,4,
  8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,4,8,8,16,16,8,8,8,8,8,8,8,16,16,8,8,8,8,8,8,8,8,8,8,8,8,8,8,4,4,
  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
};
// General Purpose
bool center_flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void max_transfer_command(uint8_t address, uint8_t data);
void max_transfer_data(uint8_t address, uint8_t data, uint8_t data_2);
void max_Init(void);

void shift_matrix_content(uint8_t direction);

void update_player_score(int points);

void custom_tone(uint32_t channel, uint8_t tone, uint8_t duration);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum matrix_selection{Upper = 0, Lower = 1};
enum direction{up = 0, down = 1, left = 2, right = 3, center = 4, none = 5};
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Matrices
  max_Init();
  max_Init();
  for(int i = 1 ; i < 9; i++){
	matrix_buffer[Upper][i-1] = test_array[0][i-1];
	matrix_buffer[Lower][i-1] = test_array[1][i-1];
	max_transfer_data(i, matrix_buffer[Upper][i-1], matrix_buffer[Lower][i-1]);
  }

  // LCD
  SSD1306_Init();
  SSD1306_Fill(0);

  SSD1306_GotoXY(20, 0);
  SSD1306_Puts("Welcome!", &Font_11x18, 1);
  SSD1306_GotoXY(30, 20);
  SSD1306_Puts("High Score", &Font_7x10, 1);

  SSD1306_GotoXY(0, 35);
  sprintf(high_score_user, "HugoCRISTO");
  sprintf(high_score_str, "%s - %05lu", high_score_user, high_score);
  SSD1306_Puts(high_score_str, &Font_7x10, 1);

  SSD1306_DrawLine(0, 32, 140, 32, 1);
  SSD1306_DrawLine(0, 45, 140, 45, 1);

  SSD1306_GotoXY(20, 50);
  SSD1306_Puts("Press Center", &Font_7x10, 1);

  SSD1306_UpdateScreen();

  int i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  custom_tone(TIM_CHANNEL_1, melody[i], noteDurations[i]);
	  i = (i < sizeof(melody)/sizeof(int) ? i + 1 : 0);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 51-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 713-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40-1;
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
  htim3.Init.Prescaler = 1000-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_RST_Pin */
  GPIO_InitStruct.Pin = BTN_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_UP_Pin BTN_DOWN_Pin BTN_LEFT_Pin BTN_CENTER_Pin */
  GPIO_InitStruct.Pin = BTN_UP_Pin|BTN_DOWN_Pin|BTN_LEFT_Pin|BTN_CENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_RIGHT_Pin */
  GPIO_InitStruct.Pin = BTN_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void max_transfer_command(uint8_t address, uint8_t data){
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}
void max_transfer_data(uint8_t address, uint8_t data, uint8_t data_2){
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);							// Upper
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data_2, 1, 100);							// Lower
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}
void max_Init(void){
	max_transfer_command(0x09, 0x00);       									//  Decode Mode 	[NO]
	max_transfer_command(0x0A, 0x07);       									//  Intensity		[07]
	max_transfer_command(0x0B, 0x07);       									//  Scan Limit 		[07]
	max_transfer_command(0x0C, 0x01);       									//  Shutdown		[01] Normal
	max_transfer_command(0x0F, 0x00);      										//  No Test Display [00]
}
void shift_matrix_content(uint8_t direction){
	row_t tmp_row;
	switch(direction){
	case up:
		  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  // Pending rotations
			for(int i = 1 ; i < 9; i++){
				matrix_buffer[Upper][i-1] = test_array[Upper][i-1];
				matrix_buffer[Lower][i-1] = test_array[Lower][i-1];
				max_transfer_data(i, matrix_buffer[Upper][i-1], matrix_buffer[Lower][i-1]);
			}
		break;
	case down:
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

		tmp_row = matrix_buffer[Upper][0];

		for(int i = 1 ; i < 8; i++){
			matrix_buffer[Upper][i-1] = matrix_buffer[Upper][i];
			matrix_buffer[Lower][i-1] = matrix_buffer[Lower][i];
			max_transfer_data(i, matrix_buffer[Upper][i-1], matrix_buffer[Lower][i-1]);
		}
		matrix_buffer[Upper][7] = 0b00000000;
		matrix_buffer[Lower][7] = tmp_row;
		max_transfer_data(8, matrix_buffer[Upper][7], matrix_buffer[Lower][7]);
		break;
	case left:
		  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  for(int i = 1; i < 9; i++){
			matrix_buffer[Upper][i-1] = matrix_buffer[Upper][i-1] >> 1;
			matrix_buffer[Lower][i-1] = matrix_buffer[Lower][i-1] >> 1;
			max_transfer_data(i, matrix_buffer[Upper][i-1], matrix_buffer[Lower][i-1]);
		  }

		break;
	case right:
		  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  for(int i = 1; i < 9; i++){
			matrix_buffer[Upper][i-1] = matrix_buffer[Upper][i-1] << 1;
			matrix_buffer[Lower][i-1] = matrix_buffer[Lower][i-1] << 1;
			max_transfer_data(i, matrix_buffer[Upper][i-1], matrix_buffer[Lower][i-1]);
		  }
		break;
	}

}
void update_player_score(int points){
	high_score += points;
	SSD1306_GotoXY(0, 35);
	sprintf(high_score_str, "%s - %05lu", high_score_user, high_score);
	SSD1306_Puts(high_score_str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void custom_tone(uint32_t channel, uint8_t tone, uint8_t duration){
	  htim1.Init.Prescaler = psc_array[tone]-1;
	  htim1.Init.Period = arr_array[tone]-1;
	  HAL_TIM_Base_Init(&htim1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	  htim3.Init.Prescaler = (1000*1.3)-1;
	  htim3.Init.Period = (1000/duration)-1;
	  HAL_Delay(200);
	  HAL_TIM_Base_Start_IT(&htim3);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == BTN_UP_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == BTN_DOWN_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == BTN_LEFT_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == BTN_RIGHT_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == BTN_CENTER_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == BTN_RST_Pin) HAL_TIM_Base_Start_IT(&htim2);


}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  /* Prevent unused argument(s) compilation warning */
  if(htim == &htim2){

	  if(!HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin)){
		  update_player_score(1);
		  shift_matrix_content(up);
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin)){
		  update_player_score(-1);
		  shift_matrix_content(down);
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin)){
		  shift_matrix_content(left);
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin)){
		  shift_matrix_content(right);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(BTN_CENTER_GPIO_Port, BTN_CENTER_Pin)){
		  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  center_flag = true;

		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(BTN_RST_GPIO_Port, BTN_RST_Pin)){
		  HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		  HAL_TIM_Base_Stop(&htim2);
	  }

  }
  if(htim == &htim3){
	  	  HAL_TIM_Base_Stop(&htim3);
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
