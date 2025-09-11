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
// 定义绘图区域的边界
#define GRAPH_X_START 15  // Y轴标签占了左边一些空间
#define GRAPH_Y_START 15  // 状态栏占了顶部一些空间
#define GRAPH_WIDTH   110 // 图表宽度
#define GRAPH_HEIGHT  100 // 图表高度
#define GRAPH_Y_END   (GRAPH_Y_START + GRAPH_HEIGHT)
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
volatile uint32_t button_press_time = 0;

// 定义显示模式
typedef enum {
    DISPLAY_HEART_RATE,
    DISPLAY_TEMPERATURE
} DisplayMode;

DisplayMode current_mode = DISPLAY_HEART_RATE;
int zoom_level = 1; // 缩放级别，1为不缩放

// 定义编码器控制模式
typedef enum {
    MODE_ZOOM,
    MODE_PAN
} ControlMode;

volatile ControlMode control_mode = MODE_ZOOM; // 默认是缩放模式
volatile uint8_t long_press_detected = 0;

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
void draw_waveform(u8g2_t* u8g2, uint8_t* data, const char* title, int zoom, DisplayMode mode);
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
      // 等待一小段时间，看看按键是否被释放
      HAL_Delay(20); // 简单的延时防抖

      // 如果按键仍然被按着 (低电平)
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
          // 检查按下时间是否超过长按阈值 (例如 800ms)
          if (HAL_GetTick() - button_press_time > 800) {
              // --- 长按事件 ---
              if (control_mode == MODE_ZOOM) {
                  control_mode = MODE_PAN;
              } else {
                  control_mode = MODE_ZOOM;
              }
              // 等待按键释放
              while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
              button_pressed = 0; // 处理完毕，清除标志
          }
      } else { // 如果按键已经被释放，说明是短按
          // --- 短按事件 ---
          if (current_mode == DISPLAY_HEART_RATE) {
              current_mode = DISPLAY_TEMPERATURE;
          } else {
              current_mode = DISPLAY_HEART_RATE;
          }
          button_pressed = 0; // 处理完毕，清除标志
      }
  }

  // --- 处理编码器 ---
  static int32_t last_encoder_counter = 0;
  if (encoder_counter != last_encoder_counter) {
      int32_t delta = encoder_counter - last_encoder_counter;

      if (control_mode == MODE_ZOOM) {
          // --- 缩放模式 ---
          zoom_level += delta;
          if (zoom_level < 1) zoom_level = 1;
          if (zoom_level > 8) zoom_level = 8; // 增加最大缩放
        } else {
          // --- 平移模式 ---
          data_offset -= delta * 5; // 乘以一个系数让平移更快
          // *** 关键：更新边界检查 ***
          if (data_offset < 0) data_offset = 0;
          // 缓冲区总长 WAVEFORM_LENGTH，屏幕上显示 GRAPH_WIDTH 个点
          if (data_offset > (WAVEFORM_LENGTH - GRAPH_WIDTH)) data_offset = (WAVEFORM_LENGTH - GRAPH_WIDTH);
      }
      last_encoder_counter = encoder_counter;
  }


  // --- 根据状态更新显示 ---
  if (current_mode == DISPLAY_HEART_RATE) {
      draw_waveform(&u8g2, mock_heart_rate_data, "Heart Rate", zoom_level, current_mode);
  } else {
      draw_waveform(&u8g2, mock_temp_data, "Temperature", zoom_level, current_mode);
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

void draw_waveform(u8g2_t* u8g2, uint8_t* data, const char* title, int zoom, DisplayMode mode)
{
  char buffer[20]; // 一个通用的字符串缓冲区

  // 1. 清空缓冲区
  u8g2_ClearBuffer(u8g2);

  // 2. 绘制全新的顶部状态栏
  u8g2_SetFont(u8g2, u8g2_font_ncenB08_tr); // 设置状态栏字体

  // 【修改点】将标题和实时数值合并显示在左侧
  uint8_t current_raw_value = data[data_offset + GRAPH_WIDTH / 2];
  if (mode == DISPLAY_HEART_RATE) {
      int current_hr = 60 + (current_raw_value / 4);
      sprintf(buffer, "HR: %d bpm", current_hr);
  } else { // DISPLAY_TEMPERATURE
      float current_temp = 35.0f + (current_raw_value / 5.0f);
      sprintf(buffer, "Temp: %.1fC", current_temp); // 使用 %.1f 并开启了浮点支持
  }
  u8g2_DrawStr(u8g2, 2, 10, buffer); // 在左上角绘制

  // 【修改点】将Zoom/Pan状态显示在状态栏右侧
  if (control_mode == MODE_ZOOM) {
      sprintf(buffer, "Zoom:x%d", zoom);
  } else { // MODE_PAN
      sprintf(buffer, "Pan:%ld", data_offset);
  }
  u8g2_uint_t str_width = u8g2_GetStrWidth(u8g2, buffer);
  u8g2_DrawStr(u8g2, 127 - str_width, 10, buffer); // 在右上角绘制

  // 绘制状态栏下方的分割线
  u8g2_DrawHLine(u8g2, 0, 12, 128);


  // 3. 绘制坐标轴 (这部分代码保持不变)
  u8g2_DrawVLine(u8g2, GRAPH_X_START, GRAPH_Y_START, GRAPH_HEIGHT);
  u8g2_DrawHLine(u8g2, GRAPH_X_START, GRAPH_Y_END, GRAPH_WIDTH);


  // 4. 绘制坐标轴刻度与标签 (这部分代码保持不变)
  u8g2_SetFont(u8g2, u8g2_font_t0_11_tr);
  if (mode == DISPLAY_HEART_RATE) {
      u8g2_DrawStr(u8g2, 0, GRAPH_Y_START + 5, "120");
      u8g2_DrawStr(u8g2, 0, GRAPH_Y_START + GRAPH_HEIGHT / 2, "90");
      u8g2_DrawStr(u8g2, 0, GRAPH_Y_END, "60");
  } else {
      u8g2_DrawStr(u8g2, 0, GRAPH_Y_START + 5, "42");
      u8g2_DrawStr(u8g2, 0, GRAPH_Y_START + GRAPH_HEIGHT / 2, "38");
      u8g2_DrawStr(u8g2, 0, GRAPH_Y_END, "35");
  }
  u8g2_DrawStr(u8g2, GRAPH_X_START, 127, "0s");
  u8g2_DrawStr(u8g2, GRAPH_X_START + GRAPH_WIDTH / 2 - 5, 127, "5s");
  // 【修改点】精确计算 "10s" 标签的位置，使其右对齐
  u8g2_uint_t label_width = u8g2_GetStrWidth(u8g2, "10s");
  u8g2_DrawStr(u8g2, GRAPH_X_START + GRAPH_WIDTH - label_width, 127, "10s");


  // 5. 绘制波形 (这部分代码保持不变)
  for (int x = 0; x < GRAPH_WIDTH -1; x++) {
      uint8_t raw_y1 = data[x + data_offset];
      uint8_t raw_y2 = data[x + 1 + data_offset];
      int y1 = GRAPH_Y_END - (raw_y1 * GRAPH_HEIGHT / 64);
      int y2 = GRAPH_Y_END - (raw_y2 * GRAPH_HEIGHT / 64);
      int center_y = GRAPH_Y_END - (GRAPH_HEIGHT / 2) * (1.0/zoom);
      y1 = center_y - (center_y - y1) * zoom;
      y2 = center_y - (center_y - y2) * zoom;
      if(y1 < GRAPH_Y_START) y1 = GRAPH_Y_START;
      if(y1 > GRAPH_Y_END) y1 = GRAPH_Y_END;
      if(y2 < GRAPH_Y_START) y2 = GRAPH_Y_START;
      if(y2 > GRAPH_Y_END) y2 = GRAPH_Y_END;
      u8g2_DrawLine(u8g2, GRAPH_X_START + x, y1, GRAPH_X_START + x + 1, y2);
  }

  // 6. 将缓冲区内容发送到屏幕
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
