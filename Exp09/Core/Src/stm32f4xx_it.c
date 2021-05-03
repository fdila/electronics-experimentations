/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE 2000
#define PRETRIGGER 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t data_ok = 0;
volatile uint8_t trigger_ok = 0;
volatile uint16_t elements_left = 0;

volatile uint16_t trigger_buffer[5];
volatile uint8_t trigger_index = 0;

volatile uint16_t trigger_buffer_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern volatile uint16_t buffer[SIZE+1];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	//Disable DMA1 (UART)
	DMA1_Stream3->CR &= ~DMA_SxCR_EN;
	
	//Reset DMA1 SR
	DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
	DMA1->LIFCR |= DMA_LIFCR_CHTIF3;
	DMA1->LIFCR |= DMA_LIFCR_CTEIF3;
	DMA1->LIFCR |= DMA_LIFCR_CFEIF3;
	
	//Reset DMA1 data size
	DMA1_Stream3->NDTR = SIZE*2 + 2;
	
	//re-enable ADC DMA
	//reset ADC SR
	ADC1->SR = 0x0;
	//Set number of elements
	DMA2_Stream0->NDTR = SIZE;
	DMA2_Stream0->M0AR = (uint32_t) buffer;
	//Enable DMA2 
	DMA2_Stream0->CR |= DMA_SxCR_EN;	
	//Enable ADC DMA bit
	ADC1->CR2 |= ADC_CR2_DMA;
	ADC1->CR2 |= ADC_CR2_DDS;
	//Enable TTM2 (start ADC conversion)
	TIM2->CR1 |= TIM_CR1_CEN;
	
	
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	if(ADC1->DR > 800){
		
		//disable tim2
		TIM2->CR1 &= ~TIM_CR1_CEN;
		//reset timer
		TIM2->CNT = 0x0;
		TIM2->SR = 0x0;
		//disable interrupt adc
		ADC1->CR1 &= ~ADC_CR1_EOCIE;	
		//disable ADC DMA bit
		ADC1->CR2 &= ~ADC_CR2_DMA;
		ADC1->CR2 &= ~ADC_CR2_DDS;
		//disable DMA2
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		
		uint16_t new_ndtr;
		
		trigger_buffer_index = SIZE - DMA2_Stream0->NDTR;
		
		if (trigger_buffer_index > PRETRIGGER){
			//continua ad acquisire fino alla fine
			// e acquisisci altri (SIZE - trigger_buffer_index - PRETRIGGER) campioni
			
			elements_left = (SIZE - PRETRIGGER - trigger_buffer_index);
			new_ndtr = DMA2_Stream0->NDTR;
			
			//indice da mandare a matlab per inizio dei dati da plottare
			buffer[SIZE] = trigger_buffer_index - PRETRIGGER;
			trigger_ok = 1;
			
		} else{
			DMA2_Stream0->M0AR = (uint32_t) buffer[trigger_buffer_index];
			new_ndtr = SIZE - PRETRIGGER;			
			buffer[SIZE] = SIZE - (PRETRIGGER - trigger_buffer_index);
			data_ok = 1;
		}
		
		//Set number of elements
		DMA2_Stream0->NDTR = new_ndtr;
		//disable transfer complete interrupt
		DMA2_Stream0->CR &= ~DMA_SxCR_TCIE;
		//Reset DMA2 SR
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		
		while(DMA2_Stream0->CR & DMA_SxCR_EN){};
		
		DMA2_Stream0->CR |= DMA_SxCR_TCIE;
		//enable DMA2
		DMA2_Stream0->CR |= DMA_SxCR_EN;
		
		//turn on adc dma bit
		ADC1->CR2 |= ADC_CR2_DMA;
		ADC1->CR2 |= ADC_CR2_DDS;
		
		//reset ADC SR
		ADC1->SR = 0x0;
		
		//turn on tim2
		TIM2->CR1 |= TIM_CR1_CEN;
	}

  /* USER CODE END ADC_IRQn 0 */
  //HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uint8_t comando;	
	if (USART3->SR & USART_SR_RXNE){
		comando = USART3->DR;
		if(comando == 10) {
			//enable adc interrupt
			ADC1->CR1 |= ADC_CR1_EOCIE;
		}
	}

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
	
	//disable tim2
	TIM2->CR1 &= ~TIM_CR1_CEN;
	//reset timer
	TIM2->CNT = 0x0;
	TIM2->SR = 0x0;
	
	//disable ADC DMA bit
	ADC1->CR2 &= ~ADC_CR2_DDS;
	ADC1->CR2 &= ~ADC_CR2_DMA;
	
	if(trigger_ok){
		
		//disable DMA2
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		
		//disable transfer complete interrupt
		DMA2_Stream0->CR &= ~DMA_SxCR_TCIE;
		
		//Reset DMA2 SR
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		
		//get the elements left
		DMA2_Stream0->NDTR = elements_left;
		
		//disable transfer complete interrupt
		DMA2_Stream0->CR &= ~DMA_SxCR_TCIE;
		//Reset DMA2 SR
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		
		DMA2_Stream0->CR |= DMA_SxCR_TCIE;
		//enable DMA2
		DMA2_Stream0->CR |= DMA_SxCR_EN;
		
		//turn on adc dma bit
		ADC1->CR2 |= ADC_CR2_DMA;
		ADC1->CR2 |= ADC_CR2_DDS;
		
		//reset ADC SR
		ADC1->SR = 0x0;
		
		//turn on tim2
		TIM2->CR1 |= TIM_CR1_CEN;
		
		data_ok = 1;
		trigger_ok = 0;
			
	} else if(data_ok){
		
		data_ok = 0;
		trigger_ok = 0;
		//disable DMA2
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		
		//Reset DMA2 SR
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		
		DMA1_Stream3->M0AR = (uint32_t) buffer;
		
		//Clear USART TC bit
		USART3->SR &= ~USART_SR_TC;
	
		//enable DMA1 (UART)
		DMA1_Stream3->CR |= DMA_SxCR_EN;
	} else {

		//Reset DMA2 SR
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
		DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
		
		DMA2_Stream0->CR |= DMA_SxCR_TCIE;
		//enable DMA2
		DMA2_Stream0->CR |= DMA_SxCR_EN;
		
		//turn on adc dma bit
		ADC1->CR2 |= ADC_CR2_DMA;
		ADC1->CR2 |= ADC_CR2_DDS;
		
		//reset ADC SR
		ADC1->SR = 0x0;
		
		//turn on tim2
		TIM2->CR1 |= TIM_CR1_CEN;
	}
	

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  //HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
