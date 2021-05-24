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

volatile uint8_t trigger_count = 0;

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
	//disable transfer complete interrupt
	DMA2_Stream0->CR &= ~DMA_SxCR_TCIE;
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
uint16_t trigger_val = 0;
uint8_t quale_if = 7;
uint16_t valore1 = 0;
uint16_t valore2 = 0;
uint16_t valore3 = 0;

uint16_t stop_ndtr;

void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	trigger_val = ADC1->DR;
	if(trigger_val > 1200 && data_ok == 0){
		trigger_buffer_index = SIZE - DMA2_Stream0->NDTR + 1;			
		if (trigger_buffer_index > PRETRIGGER){
			//continua ad acquisire fino alla fine
			// e acquisisci altri (SIZE - trigger_buffer_index - PRETRIGGER) campioni
			
			stop_ndtr = SIZE - (trigger_buffer_index - PRETRIGGER);
			//indice da mandare a matlab per inizio dei dati da plottare
			buffer[SIZE] = trigger_buffer_index - PRETRIGGER;
			
			quale_if = 0;		
			
		} else{
			stop_ndtr = SIZE - (SIZE - (PRETRIGGER-trigger_buffer_index));
			buffer[SIZE] =  SIZE - (PRETRIGGER-trigger_buffer_index);
			quale_if = 1;
		}
		data_ok = 1;
	} else if (data_ok == 1){
		if (DMA2_Stream0->NDTR == stop_ndtr){
			//stop ADC and DMA
			
			//disable adc interrupt
			ADC1->CR1 &= ~ADC_CR1_EOCIE;
			
			//disable tim2
			TIM2->CR1 &= ~TIM_CR1_CEN;
			//reset timer
			TIM2->CNT = 0x0;
			TIM2->SR = 0x0;
	
			//disable ADC DMA bit
			ADC1->CR2 &= ~ADC_CR2_DDS;
			ADC1->CR2 &= ~ADC_CR2_DMA;
			//disable DMA2
			DMA2_Stream0->CR &= ~DMA_SxCR_EN;
			//Reset DMA2 SR
			DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
			DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
			DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
			
			//enable Transmission
			DMA1_Stream3->M0AR = (uint32_t) buffer;
			//Clear USART TC bit
			USART3->SR &= ~USART_SR_TC;
			//enable DMA1 (UART)
			DMA1_Stream3->CR |= DMA_SxCR_EN;
			
			stop_ndtr = 0;
			data_ok = 0;
			trigger_val = 0;
			
		}
	}
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
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

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
