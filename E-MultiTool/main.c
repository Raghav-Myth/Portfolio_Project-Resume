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
#include <stdio.h>
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIS2DH12_I2C_ADDR   (0x18 << 1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;

volatile uint32_t last_time = 0;
volatile uint32_t count = 0;
volatile uint32_t count_func = 0;
volatile uint32_t period;
volatile uint32_t pulse;
volatile uint32_t prev_count = 20;
uint8_t button_pressed = 0;
int option = 0;
volatile uint32_t task_init = 0;

int16_t accel_values[3];

char convert[7];

int16_t calib_x;
int16_t calib_y;
int16_t calib_z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//This OP_STATE enum keeps track of the current state
typedef enum{
		MAIN_MENU,
		TASK_STATE,
		FREE_FALL
}OP_STATE;

OP_STATE current_state = MAIN_MENU;

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  //This section initializes all the peripherals
  ssd1306_Init();

  display_menu();

  accel_init();

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
    There are two if-else block in this while loop. The first if-else
    assigns the appropriate state based on button press and changes the settings for
    a specific option. The second if-else is used to actually execute the state
  */
  while (1)
  {

	  if (button_pressed){ //1st if-else

		  button_pressed = 0;

		  if (current_state == MAIN_MENU){

			  current_state = TASK_STATE;
			  task_init = 1; // this variable is used so that the display contents are only written once
		  }
		  else if (current_state == TASK_STATE){
			  current_state = MAIN_MENU;
			  display_menu();
			  prev_count = 20; //RANDOM VALUE THAT THE COUNTER CAN NEVER REACH
			  HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
		  }
		  else{
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			  current_state = MAIN_MENU;
			  display_menu();
			  __HAL_TIM_SET_COUNTER(&htim3, 0); //This ensures that after free fall you always start from option 1
			  draw_arrow(20);
		  }
	  }

	  if (current_state == MAIN_MENU){ //2nd if-else

		  count = __HAL_TIM_GET_COUNTER(&htim3); //used to track rotary encoder position. 1 turn = 4 counts

		  if (count != prev_count){ //This check is needed so the arrows don't flicker
			  clear_arrows();
			  switch (count){ // count values are 0,4,8 and 12
			  	  case 0: draw_arrow(20); break;

			  	  case 4: draw_arrow(30); break;

			  	  case 8: draw_arrow(40); break;

			  	  case 12: draw_arrow(50); break;
			  }
			  prev_count = count;
		  }

	  }
	  else if(current_state == TASK_STATE){
		  switch (count){
		  case 0: run_Angle_Measure(); break;
		  case 4: run_Level_Checker(); break;
		  case 8: run_Continuity_Test(); break;
		  case 12: run_PWM(); break;
		  }
	  }
	  else{ //Free fall state
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  ssd1306_Fill(Black);

		  ssd1306_SetCursor(2, 20);
		  ssd1306_WriteString("Free Fall Detected!", Font_6x8, White);

		  ssd1306_SetCursor(2, 30);
		  ssd1306_WriteString("Press button to see", Font_6x8, White);
		  ssd1306_SetCursor(2, 40);
		  ssd1306_WriteString("Options", Font_6x8, White);

		  ssd1306_UpdateScreen();
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
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
  hi2c1.Init.Timing = 0x00503D58;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 16000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 8000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IO1_Pin|IO2_Pin|IO3_Pin|IO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_INT_Pin */
  GPIO_InitStruct.Pin = SW_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IO1_Pin IO2_Pin IO3_Pin IO4_Pin */
  GPIO_InitStruct.Pin = IO1_Pin|IO2_Pin|IO3_Pin|IO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Connect_Pin INT2_Pin */
  GPIO_InitStruct.Pin = Connect_Pin|INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT1_Pin */
  GPIO_InitStruct.Pin = INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void accel_init(void){
	//Sets the sensor to sample at 50Hz and enables all 3 axis (X, Y and Z)
	uint8_t ctrl_reg1 = 0x47;
	HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_I2C_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, 100);

	//enables interrupt1 pin
	uint8_t ctrl_reg3 = 0x40;
	HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_I2C_ADDR, 0x22, I2C_MEMADD_SIZE_8BIT, &ctrl_reg3, 1, 100);

	//Enables Block data Update. This tells the sensor to freeze the value until it is read. Also the full scale range is 2g
	uint8_t ctrl_reg4 = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_I2C_ADDR, 0x23, I2C_MEMADD_SIZE_8BIT, &ctrl_reg4, 1, 100);

	//Configures interrupt1 pin for free fall detection. Int1 triggers if X,Y,Z fall below the threshold value
	uint8_t INT1_CFG = 0x95;
	HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_I2C_ADDR, 0x30, I2C_MEMADD_SIZE_8BIT, &INT1_CFG, 1, 100);

	//Since the threshold is 320mg and 1LSB = 16mg for FS = 2g (from data sheet), a value of 20 is written to the register
	uint8_t INT1_THS = 0x14;
	HAL_I2C_Mem_Write(&hi2c1, LIS2DH12_I2C_ADDR, 0x32, I2C_MEMADD_SIZE_8BIT, &INT1_THS, 1, 100);

}


void get_XYZ(int16_t *accel_values){

	uint8_t raw_buffer[6];

	uint8_t start_reg = 0x28 | 0x80; //The start reg address is 0x28. Bitwise OR is done to place 1 in the MSB to enable auto increment

	if (HAL_I2C_Mem_Read(&hi2c1, LIS2DH12_I2C_ADDR, start_reg, I2C_MEMADD_SIZE_8BIT, raw_buffer, 6, 100) == HAL_OK)
	{
		/*The output resolution is 10 bits. However the max value a register can store is 8bits.
		 * so the high bits are left shifted by 8 and the low bits are appended using OR operation.
		 * This data stitching results in 16 bits but the data is stored in 10 bits. So the data is then right shifted by 6
		 */
		accel_x = (int16_t)((raw_buffer[1] << 8) | raw_buffer[0]) >> 6;
		accel_y = (int16_t)((raw_buffer[3] << 8) | raw_buffer[2]) >> 6;
		accel_z = (int16_t)((raw_buffer[5] << 8) | raw_buffer[4]) >> 6;
	}

	accel_values[0] = accel_x;
	accel_values[1] = accel_y;
	accel_values[2] = accel_z;

}

void display_menu(void){
	ssd1306_Fill(Black);

	ssd1306_SetCursor(41, 0);
	ssd1306_WriteString("Options", Font_7x10, White);

	ssd1306_SetCursor(2, 20);
	ssd1306_WriteString("  Angle Measure", Font_7x10, White);

	ssd1306_SetCursor(2, 30);
	ssd1306_WriteString("  Level Checker", Font_7x10, White);

	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("  Continuity Test", Font_7x10, White);

	ssd1306_SetCursor(2, 50);
	ssd1306_WriteString("  PWM Generator", Font_7x10, White);

	ssd1306_UpdateScreen();
}

void draw_arrow(int y_coord){
	ssd1306_SetCursor(2, y_coord);
	ssd1306_WriteString(">", Font_7x10, White);
	ssd1306_UpdateScreen();
}

void run_Level_Checker(void){
	if (task_init == 1){
		task_init = 0;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(4, 0);
		ssd1306_WriteString("Level Checker", Font_7x10, White);
		ssd1306_UpdateScreen();
	}

	get_XYZ(accel_values);

	//Checks if any 2 of the 3 axes have a low value
	int flatness = (accel_values[0] >= -10 && accel_values[0] <= 10) + (accel_values[1] >= -10 && accel_values[1] <= 10) + (accel_values[2] >= -10 && accel_values[2] <= 10);

	if (flatness == 2){
		ssd1306_Fill(Black);
		ssd1306_SetCursor(4, 0);
		ssd1306_WriteString("Surface: Flat", Font_7x10, White);
		ssd1306_DrawBitmap(0,18,epd_bitmap_Happy_server,128,64,White);
		ssd1306_UpdateScreen();
	}

	else{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(4, 0);
		ssd1306_WriteString("Surface: Not-Flat", Font_7x10, White);
		ssd1306_DrawBitmap(0,18,epd_bitmap_Sad_server,128,64,White);
		ssd1306_UpdateScreen();
	}

}

void run_Angle_Measure(void){
	if (task_init == 1){
		task_init = 0;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(3, 0);
		ssd1306_WriteString("Angle Measure", Font_7x10, White);
		ssd1306_DrawCircle(100 , 30, 2, White);
		ssd1306_UpdateScreen();
		get_XYZ(accel_values);

		 //calib_x = accel_values[0];
		 //calib_y = accel_values[1];
		 //calib_z = accel_values[2];
	}

	get_XYZ(accel_values);

	//Tangent function is used to calculate the angle between X and Z axis
	float angle = atan2f((float)accel_values[0],(float)accel_values[2])*(180.0f / 3.1415f);
	snprintf(convert, sizeof(convert), "%6.1f", angle);

	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString("      ", Font_16x26, White);
	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString(convert, Font_16x26, White);
	ssd1306_UpdateScreen();

	HAL_Delay(500);
}

void run_Continuity_Test(void){
	if (task_init == 1){
		task_init = 0;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(2, 0);
		ssd1306_WriteString("Continuity Test", Font_7x10, White);
		ssd1306_UpdateScreen();
	}

	if (HAL_GPIO_ReadPin(Connect_GPIO_Port, Connect_Pin))
	{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(4, 0);
		ssd1306_WriteString("Disconnected", Font_7x10, White);
		ssd1306_DrawBitmap(0,18,epd_bitmap_disconnected,128,64,White);
		ssd1306_UpdateScreen();
	} else {
		ssd1306_Fill(Black);
		ssd1306_SetCursor(4, 0);
		ssd1306_WriteString("Connected", Font_7x10, White);
		ssd1306_DrawBitmap(0,18,epd_bitmap_connected,128,64,White);
		ssd1306_UpdateScreen();
	}

}

void run_PWM(void){
	if (task_init == 1){
		task_init = 0;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(40, 0);
		ssd1306_WriteString("PWM", Font_7x10, White);
		ssd1306_UpdateScreen();
		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

		ssd1306_SetCursor(2, 20);
		ssd1306_WriteString("  PWM 1kHz", Font_7x10, White);

		ssd1306_SetCursor(2, 30);
		ssd1306_WriteString("  PWM 100kHz", Font_7x10, White);

		ssd1306_SetCursor(2, 40);
		ssd1306_WriteString("  PWM 500kHz", Font_7x10, White);

		ssd1306_SetCursor(2, 50);
		ssd1306_WriteString("  PWM 1MHz", Font_7x10, White);

		ssd1306_UpdateScreen();

		__HAL_TIM_SET_COUNTER(&htim3, 0);

	}

	count_func = __HAL_TIM_GET_COUNTER(&htim3);

	clear_arrows();

	switch (count_func){

		case 0:
			draw_arrow(20);
			__HAL_TIM_SET_AUTORELOAD(&htim14, 15999); //Sets the period
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 7999); //Sets the pulse width
			break;

		case 4:
			draw_arrow(30);
			__HAL_TIM_SET_AUTORELOAD(&htim14, 159);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 79);
			break;

		case 8:
			draw_arrow(40);
			__HAL_TIM_SET_AUTORELOAD(&htim14, 31);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 15);
			break;

		case 12:
			draw_arrow(50);
			__HAL_TIM_SET_AUTORELOAD(&htim14, 15);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 7);
			break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  //Interrupt for button press event
  if (GPIO_Pin == SW_INT_Pin)
  {
	  if (HAL_GPIO_ReadPin(SW_INT_GPIO_Port, SW_INT_Pin) == GPIO_PIN_RESET){

		  uint32_t current_time = HAL_GetTick();

		  if ((current_time - last_time) > 200){
		  		  button_pressed = 1;
		  		  last_time = current_time;
		  }

	  }
  }

  //Interrupt for FREE FALL event
  if (GPIO_Pin == INT1_Pin){
	  current_state = FREE_FALL;
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
