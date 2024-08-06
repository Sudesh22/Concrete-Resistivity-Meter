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
#include "ADS1015_ADS1115.h"
#include "liquidcrystal_i2c.h"
#include "W25Qxx.h"
#include <math.h>
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
ADS1xx5_I2C i2c;

typedef int bool;
#define true 1
#define false 0

int16_t adc1,v;
uint8_t rxData, txData;
uint16_t rangeResistor[] = {10, 100, 500};
uint16_t Device_ID = 2241;
uint32_t address = 16;

int currRange = 1;
char str[20];
char *sendStr;

float range[] = {0.005,0.05,0.025};
float LastLocation, empty, voltage, vrms, I;
const float voltageConv = 6.114 / 32768.0;

bool done = 1;
bool Boot;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void getReadings() {
	for (int i = 0; i < 53; i++) {					// check with53 samples
		HD44780_Clear();
		HD44780_SetCursor(0, 0);
		HD44780_PrintStr("Processing...");
		adc1 = ADSreadADC_Differential_0_1_Continuous(&i2c);
		voltage = voltage - 0; // Calibrate at 100mV
		if (adc1 > v) {
			v = adc1;
		}
	}
	voltage = v * voltageConv;
	vrms = voltage / (sqrt(2));
}

char floatToCharArr(float text)
{
	uint8_t str[1000];
	sprintf(str, "%f",text);
	return str;
}

void logValue()
{

//	temperature = getTemp();
//	getValues();

	LastLocation = floatToCharArr(W25Q_Read_NUM(0));
	empty = floatToCharArr(W25Q_Read_NUM(8));

	if(LastLocation == empty)
	{
		  W25Q_Write_NUM(4, Device_ID);	  		// Device_ID at Memory Location 0
		  Boot = false;
	}

	if (Boot)
	{
		for (uint32_t addr = 16; addr<4194304; addr++)
		{
			if(floatToCharArr(W25Q_Read_NUM(addr)) == empty)
			{
				address = addr;
				break;
			}
		}
		Boot = false;
	}

//	W25Q_Write_NUM(address, temperature);
//	W25Q_Write_NUM(address, readArr);
//	W25Q_Write_NUM(address + 1 * sizeof(float), HH);
//	W25Q_Write_NUM(address + 2 * sizeof(float), MM);
//	W25Q_Write_NUM(address + 3 * sizeof(float), SS);
//	W25Q_Write_NUM(address + 4 * sizeof(float), dd);
//	W25Q_Write_NUM(address + 5 * sizeof(float), mm);
//	W25Q_Write_NUM(address + 6 * sizeof(float), yy);

	W25Q_Write_NUM(0, address);	  			// Arbitrary value at Memory Location 0
//	address += 7 * sizeof(float);
	HAL_Delay(1000);
}

char uint32_tToCharArr(uint32_t text)
{
	char str[1000];
	sprintf(str, "%lu",text);
	return str;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


//  W25Q_Reset();
  ADS1115(&i2c, &hi2c1, ADS_ADDR_GND); // Or ADS1015(&i2c, &hi2c1, ADS_ADDR_GND);
  ADSsetGain(&i2c, GAIN_TWOTHIRDS);
  HD44780_Init(2);
  HD44780_SetCursor(3,0);
  HD44780_PrintStr("VEDANTRIK");
  HD44780_SetCursor(2,1);
  HD44780_PrintStr("TECHNOLOGIES");
  HAL_Delay(2000);
  HD44780_Clear();

  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
  HD44780_SetCursor(1,0);
  HD44780_PrintStr("Press to start");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){
	  if(!done){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		  HAL_Delay(10);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
//		  v = 0;
//			   HD44780_Clear();
//		  for (int i = 0; i < 3; i++){						// check range
//			  getReadings();
//			  currRange = 0;
//			  if (vrms<range[0]){
//			    //switch from 10 ohm to 100ohm
//				  HD44780_SetCursor(0,0);
//				  HD44780_PrintStr("range low...");
//			  }else if((vrms>range[i]) && (i!=0)){
//				  currRange=1;
//				  HAL_Delay(10);
//				  break;
//			  }
//
//				  break;
//		  }
//				  HD44780_SetCursor(0,0);
//				  HD44780_PrintStr("range correct...");
//				  I = vrms/rangeResistor[1];
		  currRange = 0;

		  for(int i = 0; i < 3; i++){
			  switch(currRange){
			  case 0:
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
				  HAL_Delay(1000);
				  //hal gpio
				  break;
			  case 1:
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HD44780_Clear();
				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("Changing range...");
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
				  HAL_Delay(1000);
				  //hal gpio
				  break;
			  case 2:
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HD44780_Clear();
				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("Changing range...");
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
				  HAL_Delay(1000);
				  //hal gpio
				  break;
			  }
			  v = 0;
			  getReadings();
			  if (vrms<range[0] && currRange != 2){
	//			  switch from 10 ohm to 100ohm
//				  HD44780_SetCursor(0,0);
//				  HD44780_PrintStr("range low...");
				  currRange += 1;
			  }else{
//				  HD44780_SetCursor(0,0);
//				  HD44780_PrintStr("range correct...");
				  I = vrms/rangeResistor[currRange];

				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("Vpeak: ");

				  HD44780_SetCursor(7,0);
				  sprintf(str, "%.6fV", voltage);
				  HD44780_PrintStr(str);

				  HD44780_Clear();
				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("Va:");

				  HD44780_SetCursor(3,0);
				  sprintf(str, "%.1fmV", vrms*1000);
				  HD44780_PrintStr(str);
				  HD44780_SetCursor(8,0);
				  HD44780_PrintStr("I:");

				  HD44780_SetCursor(10,0);
				  sprintf(str, "%.1fuA", I*1000000);
				  HD44780_PrintStr(str);

				  HD44780_SetCursor(0,1);
				  if (I<range[currRange]){
					  sprintf(str, "R:10-50");
				  }else if (I<range[1]){
					  sprintf(str, "R:50-500");
				  }else{
					  sprintf(str, "R:500-50000");
				  }
				  HD44780_PrintStr(str);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(200);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  break;
			  }

			  if ((currRange == 3) && (vrms <0.001)){
				  HD44780_Clear();
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(200);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HAL_Delay(10);
				  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
				  HD44780_SetCursor(3,0);
				  HD44780_PrintStr("No sample");
				  HD44780_SetCursor(3,1);
				  HD44780_PrintStr("detected!");
			  }
		  }

//		  HD44780_SetCursor(0,0);
//		  HD44780_PrintStr("Vpeak: ");
//
//		  HD44780_SetCursor(7,0);
//		  sprintf(str, "%.6fV", voltage);
//		  HD44780_PrintStr(str);

//		  HD44780_Clear();
//		  HD44780_SetCursor(0,0);
//		  HD44780_PrintStr("Va:");
//
//		  HD44780_SetCursor(3,0);
//		  sprintf(str, "%.1fmV", vrms);
//		  HD44780_PrintStr(str);
//		  HD44780_SetCursor(8,0);
//		  HD44780_PrintStr("I:");
//
//		  HD44780_SetCursor(10,0);
//		  sprintf(str, "%.1fuA", I);
//		  HD44780_PrintStr(str);

//		  HD44780_SetCursor(0,1);
//		  if (I<range[1]){
//			  sprintf(str, "R:10-50");
//		  }else if (I<range[2]){
//			  sprintf(str, "R:50-500");
//		  }else{
//			  sprintf(str, "R:500-50000");
//		  }
//		  HD44780_PrintStr(str);
		  done = 1;
	  }
//	 done = 0;
//	 }else{
//	 v = 0;
//	 voltage = 0;

//	 HD44780_Clear();
//	 for(int i = 0; i < 860; i++){
//	   HD44780_SetCursor(0,0);
//	   HD44780_PrintStr("Processing...");
//	   adc1 = ADSreadADC_Differential_0_1_Continuous(&i2c);
//	   voltage = voltage - 0; // Calibrate at 100mV
//	   if (adc1>v){
//		 v = adc1;
//	   }
//	  }


//	   HD44780_Clear();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    }
//	   if (count == 860){
//		   voltage = v * voltageConv;
//		   vrms = voltage/(sqrt(2));
//		   HD44780_SetCursor(0,0);
//		   HD44780_PrintStr("Vpeak: ");
//
//		   HD44780_SetCursor(7,0);
//		   sprintf(str, "%.6fV", v);
//		   HD44780_PrintStr(str);
//
//		   HD44780_SetCursor(0,1);
//		   HD44780_PrintStr("Vads: ");
//
//		   HD44780_SetCursor(6,1);
//		   sprintf(str, "%.6fV", vrms);
//		   HD44780_PrintStr(str);
//	   }
//	 }
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	while(GPIO_Pin == GPIO_PIN_1 && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1)){
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
//	}
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	if(GPIO_Pin == GPIO_PIN_1 && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1)){
		done = 0;

	}

		//	   HD44780_SetCursor(0,0);
		//	   HD44780_PrintStr("Processing...");
//			   adc1 = ADSreadADC_Differential_0_1_Continuous(&i2c);

//			   		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		//	   voltage = voltage - 0; // Calibrate at 100mV
//			   if (adc1>v){
//				 v = adc1;
//			   }
//			   count += 1;
//			  }

		//	   HD44780_Clear();


//		HD44780_Clear();
//		HD44780_SetCursor(0,0);
//		HD44780_PrintStr("Processing...");
//		execute();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
    if(rxData==78) // Ascii value of 'N' is 78 (N for NO)
    {
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    	sendStr ="abc\na";
    	HAL_UART_Transmit(&huart1, (uint8_t*)sendStr, sizeof(sendStr), HAL_MAX_DELAY);
    }
    else if (rxData==89) // Ascii value of 'Y' is 89 (Y for YES)
    {
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
    }
    HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive again
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
