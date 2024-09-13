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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t data_rx=0;
uint32_t rx_data = 0;
uint32_t array5[5];
uint8_t control=0;
uint8_t lenght=0;
uint32_t show1=0;
char name='R';
uint32_t gelen_data = 0;
uint32_t data_14 = 0;
uint32_t data_TXE=1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* 4- Enable UART Receive Data Register Not Empty */
   //SET_BIT(USART2->CR1, USART_CR1_RXNEIE);
   USART2->CR1 |= USART_CR1_RXNEIE;

   SET_BIT(USART2->CR1, USART_CR1_TXEIE);

   /* 5 - Enable UART Interrupt in NVIC */

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //USART2->DR='G';
	  //HAL_Delay(1000);

	 //data_rx = USART2->DR;

	  //HAL_GPIO_WritePin(LED15_GPIO_Port,LED15_Pin, GPIO_PIN_SET);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED14_Pin|LED15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED14_Pin LED15_Pin */
  GPIO_InitStruct.Pin = LED14_Pin|LED15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void USART2_IRQHandler(void) {
	uint32_t isrflags = USART2->SR;
	uint32_t control_reg1 = USART2->CR1;
	 //data_TXE = USART2->SR & USART_SR_TXE;
	 data_TXE = USART2->CR1 & USART_CR1_TCIE;
	 while(data_TXE !=0){
		 HAL_GPIO_WritePin(LED15_GPIO_Port,LED15_Pin,GPIO_PIN_SET);
	 }
	//data_TXE = USART2->SR & USART_SR_TC;
	 if(data_TXE ==1){
		 HAL_GPIO_WritePin(LED15_GPIO_Port,LED15_Pin,GPIO_PIN_SET);
	 }
	/* UART in mode Receiver */
	if (((isrflags & USART_SR_RXNE) != RESET) && ((control_reg1 & USART_CR1_RXNEIE) != RESET)) {
		HAL_GPIO_TogglePin(LED14_GPIO_Port,LED14_Pin);
		rx_data = (uint8_t) USART2->DR;
		if(control>=0 && (control<5)){
		array5[control]=rx_data;
		control++;
		} else {
			control = 0;
			array5[control] = rx_data;
		}

		if (rx_data == 'A') {
			lenght = sizeof(array5) / sizeof(array5[0]);
			HAL_GPIO_WritePin(LED15_GPIO_Port,LED15_Pin,GPIO_PIN_SET);
			//time1();
			//HAL_GPIO_WritePin(LED15_GPIO_Port,LED15_Pin,GPIO_PIN_RESET);
			while ((USART2->SR & USART_SR_TXE) == 0)
				;
			USART2->DR = 'K';
			while ((USART2->SR & USART_SR_TXE) == 0)
				;
			USART2->DR = 'T';
			while ((USART2->SR & USART_SR_TXE) == 0)
				;
			USART2->DR = 'E';
			while ((USART2->SR & USART_SR_TXE) == 0)
				;
			USART2->DR = 'P';
		}

		else if (rx_data == 'B') {
			HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_RESET);
		}
		return;
	}

	/* UART in mode Transmitter */
	if (((isrflags & USART_SR_TXE) != RESET) && ((control_reg1 & USART_CR1_TXEIE) != RESET)) {
		USART2->CR1 &=~(USART_CR1_TXEIE);

		HAL_GPIO_WritePin(LED14_GPIO_Port,LED14_Pin,GPIO_PIN_SET);
		gelen_data = USART2->DR;
		if(gelen_data == 51){
		HAL_GPIO_WritePin(LED15_GPIO_Port,LED15_Pin,GPIO_PIN_SET);

		//USART2->CR1 |= USART_CR1_RXNEIE;
		//USART2->CR1 &=~(USART_CR1_TXEIE);
		}
		return;
	}
}


void time1(void){
	for (int var = 0; var < 10000000; ++var) {

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
