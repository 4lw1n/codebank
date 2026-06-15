/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "network.h"
#include "network_data.h"
#include "network_config.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h> // REQUIRED for build
#include <errno.h>    // REQUIRED for build
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS7341_ADDR       (0x39 << 1)
#define AS7341_CFG_0      0xA9
#define AS7341_ENABLE     0x80
#define AS7341_ATIME      0x81
#define AS7341_ASTEP_L    0xCA
#define AS7341_ASTEP_H    0xCB
#define AS7341_CFG1       0xAA
#define AS7341_STATUS     0x93
#define AS7341_CH0_DATA_L 0x95
#define AS7341_CONFIG     0x70
#define AS7341_LED        0x74
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t  activation_buffer[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
static float    net_input[AI_NETWORK_IN_1_SIZE];
static float    net_output[AI_NETWORK_OUT_1_SIZE];
static ai_handle network = AI_HANDLE_NULL;


ai_buffer *ai_input;
ai_buffer *ai_output;


static const char *class_labels[] = { "SALT", "SUGAR" };

/* StandardScaler parameters — values extracted from training */

static const float SCALER_MEAN[8] = {3157.44270833f, 15739.39322917f,  7197.25260417f, 14883.94270833f,
 18000.f,         18000.f,         16314.1015625f,   7698.390625f};


static const float SCALER_STD[8] = {54.87995278f, 453.79810817f, 111.62269615f, 125.60730327f,   1.f,
		   1.f,         148.26606362f,  71.61058985f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//SENSOR FUNCTIONS
void AS7341_Init(void);
void AS7341_Read_8_Values(uint16_t *readings);
void WriteReg(uint8_t reg, uint8_t value);
uint8_t ReadReg(uint8_t reg);
void AS7341_Setup_SMUX(uint8_t mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
       HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
    return len;
}
int _read(int file, char *ptr, int len) { return 0; }
int _close(int file) { return -1; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _fstat(int file, struct stat *st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { return 1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { errno = EINVAL; return -1; }


void WriteReg(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, AS7341_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

uint8_t ReadReg(uint8_t reg) {
    uint8_t val;
    HAL_I2C_Mem_Read(&hi2c1, AS7341_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

void AS7341_Init(void) {
    HAL_Delay(500);
    uint8_t id = ReadReg(0x92);
    printf("Sensor ID: 0x%02X\r\n", id);

    WriteReg(AS7341_ENABLE, 0x01); // Power ON
    HAL_Delay(10);

    // --- LED Setup (Level 1) ---
    uint8_t cfg0 = ReadReg(AS7341_CFG_0);
    WriteReg(AS7341_CFG_0, cfg0 | 0x10); // Bank 1
    uint8_t config = ReadReg(AS7341_CONFIG);
    WriteReg(AS7341_CONFIG, config | 0x08); // LED Master
    WriteReg(AS7341_LED, 0x00); // Level 1 (Low)
    WriteReg(AS7341_CFG_0, cfg0 & 0xEF); // Bank 0

    // --- Sensitivity Setup ---
    WriteReg(AS7341_ATIME, 29);
    WriteReg(AS7341_ASTEP_L, 0x57);
    WriteReg(AS7341_ASTEP_H, 0x02);
    WriteReg(AS7341_CFG1, 4); // Gain 8x (Matches Arduino "4")
}

void AS7341_Setup_SMUX(uint8_t mode) {
    uint8_t smuxConfig_F1_F4[20] = {
        0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x50, 0x00,
        0x00, 0x00, 0x20, 0x04, 0x00, 0x30, 0x01, 0x50, 0x00, 0x06
    };
    uint8_t smuxConfig_F5_F8[20] = {
        0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03, 0x50, 0x10,
        0x03, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x50, 0x00, 0x06
    };

    uint8_t *cfg = (mode == 0) ? smuxConfig_F1_F4 : smuxConfig_F5_F8;

    uint8_t cfg0 = ReadReg(AS7341_CFG_0);
    WriteReg(AS7341_CFG_0, cfg0 & 0xEF);
    WriteReg(0xAF, 0x10);
    HAL_I2C_Mem_Write(&hi2c1, AS7341_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, cfg, 20, 100);
    WriteReg(AS7341_ENABLE, 0x11);

    int timeout = 1000;
    while ((ReadReg(AS7341_ENABLE) & 0x10) && timeout > 0) timeout--;
}

void AS7341_Read_8_Values(uint16_t *readings) {
    uint8_t buffer[12];

    AS7341_Setup_SMUX(0);
    WriteReg(AS7341_ENABLE, 0x03);
    while (!(ReadReg(AS7341_STATUS) & 0x08)) {};
    HAL_I2C_Mem_Read(&hi2c1, AS7341_ADDR, AS7341_CH0_DATA_L, I2C_MEMADD_SIZE_8BIT, buffer, 12, 100);
    readings[0] = buffer[0] | (buffer[1] << 8);
    readings[1] = buffer[2] | (buffer[3] << 8);
    readings[2] = buffer[4] | (buffer[5] << 8);
    readings[3] = buffer[6] | (buffer[7] << 8);
    WriteReg(AS7341_ENABLE, 0x01);

    AS7341_Setup_SMUX(1);
    WriteReg(AS7341_ENABLE, 0x03);
    while (!(ReadReg(AS7341_STATUS) & 0x08)) {};
    HAL_I2C_Mem_Read(&hi2c1, AS7341_ADDR, AS7341_CH0_DATA_L, I2C_MEMADD_SIZE_8BIT, buffer, 12, 100);
    readings[4] = buffer[0] | (buffer[1] << 8);
    readings[5] = buffer[2] | (buffer[3] << 8);
    readings[6] = buffer[4] | (buffer[5] << 8);
    readings[7] = buffer[6] | (buffer[7] << 8);
    WriteReg(AS7341_ENABLE, 0x01);
    
    printf("%d,%d,%d,%d,%d,%d,%d,%d\n\r",readings[0],readings[1],readings[2],readings[3],readings[4],readings[5],readings[6],readings[7]);
  
}
/* printf → UART2 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* StandardScaler: normalized = (raw - mean) / std */
static void standardscale(uint16_t *raw, float *out)
{
    for (int i = 0; i < 8; i++)
        out[i] = ((float)raw[i] - SCALER_MEAN[i]) / SCALER_STD[i];
}

/* Initialize AI network */
static void ai_init(void)
{
    ai_error err;

    err = ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE) {
        printf("[ERROR] create failed: type=%d code=%d\r\n", err.type, err.code);
        Error_Handler();
    }

    const ai_network_params params = {
        AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
        AI_NETWORK_DATA_ACTIVATIONS(activation_buffer)
    };

    if (!ai_network_init(network, &params)) {
        err = ai_network_get_error(network);
        printf("[ERROR] init failed: type=%d code=%d\r\n", err.type, err.code);
        Error_Handler();
    }

    printf("Network ready\r\n");
}

/* Run one inference */
static void ai_run(float *input, float *output)
{

    ai_input = ai_network_inputs_get(network, NULL);
    ai_output = ai_network_outputs_get(network, NULL);

    ai_input[0].data  = AI_HANDLE_PTR(input);
    ai_output[0].data = AI_HANDLE_PTR(output);

    if (ai_network_run(network, ai_input, ai_output) != 1) {
        ai_error err = ai_network_get_error(network);
        printf("[ERROR] run failed: type=%d code=%d\r\n", err.type, err.code);
    }
}

static void print_result(uint16_t *raw, float *output)
{
    /* Raw output value from model (single sigmoid neuron) */
    float raw_out = output[0];

    /* Determine class from threshold */
    const char *label;
    float confidence;

    if (raw_out >= 0.5f) {
        label      = "SUGAR";
        confidence = raw_out;
    } else {
        label      = "SALT";
        confidence = 1.0f - raw_out;
    }

    printf("------------------------------\r\n");
    printf("Raw inputs : %u %u %u %u %u %u %u %u\r\n",
           raw[0], raw[1], raw[2],
           raw[3], raw[4], raw[5],
           raw[6], raw[7]);
    printf("AI raw out : %.6f\r\n", raw_out);   /* ← exact value from model */
    printf("Prediction : %s\r\n",   label);
    printf("Confidence : %.1f%%\r\n", confidence * 100.0f);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  AS7341_Init();
  ai_init();
  printf("===Salt/Sugar Classifier ===============\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint16_t raw[8];
	 AS7341_Read_8_Values(raw);
	      standardscale(raw, net_input);
	      ai_run(net_input, net_output);
	      print_result(raw, net_output);

	      HAL_Delay(400);
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
