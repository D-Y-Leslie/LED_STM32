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
#include "u8g2_stm32.h"
#include <math.h>
#include <stdio.h>
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

// volatile 关键字是必须的，因为它会在中断和主循环中同时被访问
volatile int32_t encoder_counter = 0;
volatile uint8_t button_pressed = 0;
volatile int32_t data_offset = 0;

// 定义显示模式
typedef enum {
    DISPLAY_HEART_RATE,
    DISPLAY_TEMPERATURE
} DisplayMode;

DisplayMode current_mode = DISPLAY_HEART_RATE;
int zoom_level = 1; // 缩放级别，1为不缩放

//暂时没有真实数据，我们先创建一些模拟数据来绘制。
#define WAVEFORM_LENGTH 128
uint8_t mock_heart_rate_data[WAVEFORM_LENGTH];
uint8_t mock_temp_data[WAVEFORM_LENGTH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void draw_waveform(u8g2_t* u8g2, uint8_t* data, const char* title, int zoom); 
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
  MX_TIM16_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim16); // 启动TIM16的更新中断
  u8g2_Init(&u8g2); // 初始化u8g2

  // 生成模拟心率波形 (类似正弦波)
for(int i=0; i<WAVEFORM_LENGTH; i++) {
    mock_heart_rate_data[i] = 32 + 20 * sin(i * 3.14159 * 4 / WAVEFORM_LENGTH);
}
// 生成模拟温度曲线 (缓慢上升)
for(int i=0; i<WAVEFORM_LENGTH; i++) {
    mock_temp_data[i] = 20 + (i / 8);
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  // --- 处理输入 ---
  if (button_pressed) {
      button_pressed = 0; // 清除标志
      if (current_mode == DISPLAY_HEART_RATE) {
          current_mode = DISPLAY_TEMPERATURE;
      } else {
          current_mode = DISPLAY_HEART_RATE;
      }
  }

  // 假设我们用编码器来调整缩放
  // 为了避免值变化太快，我们保存上一次的值
  static int32_t last_encoder_counter = 0;
  if (encoder_counter != last_encoder_counter) {
      if (encoder_counter > last_encoder_counter) {
          zoom_level++; // 顺时针，放大
      } else {
          zoom_level--; // 逆时针，缩小
      }
      if (zoom_level < 1) zoom_level = 1; // 最小缩放为1
      if (zoom_level > 5) zoom_level = 5; // 最大缩放为5
      last_encoder_counter = encoder_counter;
  }


  // --- 根据状态更新显示 ---
  if (current_mode == DISPLAY_HEART_RATE) {
      draw_waveform(&u8g2, mock_heart_rate_data, "Heart Rate", zoom_level);
  } else {
      draw_waveform(&u8g2, mock_temp_data, "Temperature", zoom_level);
  }

  HAL_Delay(50); // 主循环不需要太快，给其他任务留出时间
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 63;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin|OLED_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RES_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void draw_waveform(u8g2_t* u8g2, uint8_t* data, const char* title, int zoom)
{
    u8g2_ClearBuffer(u8g2);
    // 在 u8g2_ClearBuffer 之后
    u8g2_DrawVLine(u8g2, 5, 15, 110); // 在x=5位置，画一条从y=15到y=125的线 
    char val_str[10];
    // 假设屏幕中央对应1.65V，zoom=1时满幅对应3.3V
    float max_val = 1.65 + 1.65 / zoom;
    float min_val = 1.65 - 1.65 / zoom;
    sprintf(val_str, "%.1fV", max_val);
    u8g2_DrawStr(u8g2, 7, 25, val_str); // 在轴顶部显示
    sprintf(val_str, "%.1fV", min_val);
    u8g2_DrawStr(u8g2, 7, 125, val_str); // 在轴底部显示

    u8g2_SetFont(u8g2, u8g2_font_ncenB08_tr);

    // 绘制标题
    u8g2_DrawStr(u8g2, 0, 10, title);

    // 绘制缩放级别
    char zoom_str[10];
    sprintf(zoom_str, "x%d", zoom); // 我们可以只显示 "x2", "x3" 更简洁

    // 计算字符串的像素宽度
    u8g2_uint_t str_width = u8g2_GetStrWidth(u8g2, zoom_str);

    // 从屏幕最右边(127)减去宽度，得到起始X坐标
    u8g2_DrawStr(u8g2, 127 - str_width, 10, zoom_str);

    // 如果你还想显示 "Zoom:" 标签，可以把它放在左边
    u8g2_DrawStr(u8g2, 80, 10, "Zoom:");

    // 在绘制波形前
    u8g2_SetDrawColor(u8g2, 1); // 设置为前景色
    // 绘制水平线
    for (int y = 32; y < 128; y += 16) {
        u8g2_DrawHLine(u8g2, 0, y, 128);
    }
    // 绘制垂直线
    for (int x = 0; x < 128; x += 16) {
        u8g2_DrawVLine(u8g2, x, 15, 113);
    }

    // 绘制波形
    for (int x = 0; x < WAVEFORM_LENGTH - 1; x++) {
        // 简单的缩放逻辑：以屏幕中心为基准进行缩放
        int y1 = 64 - (data[x + data_offset] - 32) * zoom;
        int y2 = 64 - (data[x + 1 + data_offset] - 32) * zoom;

        // 限制在屏幕范围内
        if(y1 < 12) y1 = 12;
        if(y1 > 127) y1 = 127;

        if(y2 < 12) y2 = 12;
        if(y2 > 127) y2 = 127;

        u8g2_DrawLine(u8g2, x, y1, x + 1, y2);
    }

    u8g2_SendBuffer(u8g2);
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
