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
	* Acquire data with trigger, no pretrigger data acquired.
	*
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t buffer[SIZE+1];
volatile uint16_t buffer_index;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	/* Setup ADC */
	
	
	//Select 1 conversions for each sequence
	ADC1->SQR1 = 0;
	//Select channel 5 (PA5)
	ADC1->SQR3 = 0x0;
	ADC1->SQR3 |= ADC_SQR3_SQ1_0;
	ADC1->SQR3 |= ADC_SQR3_SQ1_2;
	//Set EOC flag at the end of each conversion
	ADC1->CR2 |= ADC_CR2_EOCS;
	//Enable ADC overrun interrupt
	ADC1->CR1 &= ~ADC_CR1_OVRIE;
	
	ADC1->CR1 &= ~ADC_CR1_EOCIE;
	
	//Turn on scan mode
	ADC1->CR1 |= ADC_CR1_SCAN;
	//Turn on DMA mode!!
	ADC1->CR2 |= ADC_CR2_DMA;

	
	/* Setup UART */
	
	//Enable UART and RX interrupt
	USART3->CR1 |= USART_CR1_UE;
	USART3->CR1 &= ~USART_CR1_TCIE;
	USART3->CR1 |= USART_CR1_RXNEIE;
	//Turn on DMA on transmission
	USART3->CR3 |= USART_CR3_DMAT;
	
	
	/* setup DMA2 stream 0 - ADC */
	//set number of elements
	DMA2_Stream0->NDTR = SIZE;
	//set source peripheral address
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
	//set destination memory address
	DMA2_Stream0->M0AR = (uint32_t) buffer;
	//set half word memory data size
	DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;
	//set half word peripheral size
	DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;
	//enable transfer complete interrupt
	//DMA2_Stream0->CR |= DMA_SxCR_TCIE;
	//disable transfer complete interrupt
	DMA2_Stream0->CR &= ~DMA_SxCR_TCIE;
	//enable circ mode
	DMA2_Stream0->CR |= DMA_SxCR_CIRC;
	
  /* Setup DMA1 UART */
	//Disable DMA
	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	//set number of elements (multiply by 2 because we send bytes)
	DMA1_Stream3->NDTR = SIZE*2 + 2;
	//set source memory address
	DMA1_Stream3->M0AR = (uint32_t) buffer;
	//set destination peripheral address
	DMA1_Stream3->PAR = (uint32_t) &USART3->DR;
	//set byte memory data size
	DMA1_Stream3->CR &= ~DMA_SxCR_MSIZE_0;
	DMA1_Stream3->CR &= ~DMA_SxCR_MSIZE_1;
	//set byte peripheral size
	DMA1_Stream3->CR &= ~DMA_SxCR_PSIZE_0;
	DMA1_Stream3->CR &= ~DMA_SxCR_PSIZE_1;
	//enable transfer complete interrupt
	DMA1_Stream3->CR |= DMA_SxCR_TCIE;

	
	//Turn on ADC
	ADC1->CR2 |= ADC_CR2_ADON;
	
	//reset ADC SR
	ADC1->SR = 0x0;
	//Set number of elements
	DMA2_Stream0->NDTR = SIZE;
	//Enable DMA2 
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	//Enable ADC DMA bit
	ADC1->CR2 |= ADC_CR2_DMA;
	ADC1->CR2 |= ADC_CR2_DDS;
	//Enable TTM2 (start ADC conversion)
	TIM2->CR1 |= TIM_CR1_CEN;

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
