/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern int ts_data[50];
extern int codici[50];
extern volatile int indice;
int s = 0;
int ind = 0;

int trasm = 0;
int gc = 0; 				//DIVENTA 1 QUANDO ABBIAMO COMPLETATO LA GENERAZIONE
//extern uint8_t* puntatore_temp;
//extern uint8_t* puntatore_mvolt;
extern uint8_t* puntatore_c1;
extern uint8_t* puntatore_c2;

extern int n_mis;
extern int soglia;
int solouno = 0;
extern uint16_t salvataggio;
volatile int	k=0;
volatile uint16_t pendenza = 0;
volatile 	int32_t tampone;
extern uint16_t posizione;
extern volatile uint16_t soglia_sup;
//posizione = 0;
extern uint8_t *puntatore;
extern volatile uint16_t DMA_ADC_vett[];
volatile uint8_t confronto;
//uint16_t adc = 0;
//puntatore_temp = (uint8_t*) temperatura_mC;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern volatile uint16_t pre_trigger;
extern volatile  uint16_t post_trigger;
volatile uint8_t spara_trigger=0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	spara_trigger=1;

  /* USER CODE END EXTI2_IRQn 0 */
//	EXTI->PR1 = GPIO_PIN_2 ;
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	USART3->CR3 &= ~USART_CR3_DMAT;
	DMA1_Stream3->CR &= ~DMA_SxCR_EN ;
	DMA1_Stream3->CR &= ~DMA_SxCR_TCIE;
	DMA1->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
	DMA1->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR	
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);

	
												k = 0;
							solouno = 0;
							posizione = 0;
							soglia_sup = 0;
							pendenza = 0;
							salvataggio = 0;
							trasm = 0;
					for( uint16_t ii=0; ii<n_mis;ii++){
					DMA_ADC_vett[ii]= 0;
				}
							DMA2_Stream0->NDTR = n_mis;									//DICIAMO AL DMA IL NUMERO DI MISURE CHE VENGONO FATTE DALL'ADC
							DMA2->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
							DMA2->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR				
							DMA2_Stream0->CR |= DMA_SxCR_EN; 						//ACCENDE IL DMA
				while(( DMA2_Stream0->CR & DMA_SxCR_EN )==0){}
							if (DMA2->LISR & DMA_LISR_FEIF0){
								DMA2_Stream0->CR &= ~DMA_SxCR_EN;
								while(( DMA2_Stream0->CR & DMA_SxCR_EN )){}
							DMA2->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
							DMA2->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR										
								DMA2_Stream0->CR |= DMA_SxCR_EN;
							}
//							USART3->CR1 &= ~USART_CR1_UE;
							ADC3->IER |= ADC_IER_EOCIE;
							TIM6->ARR = 1;
							ADC3->ISR |= ADC_ISR_OVR;
							if ((ADC3->CR & ADC_CR_ADSTART)==0){
										ADC3->CR |= ADC_CR_ADSTART;			//PARTE LA MISURA
							}

							while(GPIOC->IDR & GPIO_IDR_ID2){}
							confronto = 0;
							spara_trigger=0;
							TIM6->CR1 |= TIM_CR1_CEN;			  //ACCENDIAMO IL TIMER
	
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

		if ((USART3->ISR & USART_ISR_RXNE_RXFNE) == (USART_ISR_RXNE_RXFNE)){								//SE E' ATTIVATO IL FLAG RXNE (CIOE' SE HA RICEVUTO QUALCOSA)   
			/*if (USART3->RDR == 'p'){
			
			}	*/	
			if (USART3->RDR == 'p'){
							USART3->CR1 |= USART_CR1_TCIE;				//RIATTIVA L'INTERRUPT DI TRASMISSIONE
							USART3->ICR |= USART_ICR_TCCF;
							ind = 1;
							USART3->TDR = puntatore[0];						//SCRIVE IL PRIMO CARATTERE DA TRASMETTERE NEL DR
							trasm = 1;
			}
			else if (USART3->RDR == 'b'){
					//fermo la misura
					USART3->CR3 &= ~USART_CR3_DMAT;
					DMA1_Stream3->CR &= ~DMA_SxCR_TCIE;		
					DMA1_Stream3->CR &= ~DMA_SxCR_EN ;				
					TIM6->CR1 &= ~TIM_CR1_CEN;							// SPEGNIMENTO TIMER 6				
					DMA2_Stream0->CR &= ~DMA_SxCR_EN;				// SPEGNIMENTO DMA ADC	
					USART3->CR1 |= USART_CR1_RXNEIE;
					solouno = 2;
			}
			else if (USART3->RDR == 'v'){
							//AZZERO TUTTI GLI INDICI
							TIM6->CR1 &= ~TIM_CR1_CEN;							// SPEGNIMENTO TIMER 6
							USART3->CR3 &= ~USART_CR3_DMAT;
							DMA1_Stream3->CR &= ~DMA_SxCR_TCIE;					
							DMA2_Stream0->CR &= ~DMA_SxCR_EN;				// SPEGNIMENTO DMA ADC				
							k = 0;
							solouno = 0;
							posizione = 0;
							soglia_sup = 0;
							pendenza = 0;
							salvataggio = 0;
							trasm = 0;			
				
							//ACCENDE TIMER, ADC E TUTTO CIO CHE SERVE PER FARE LE MISURE
				for( uint16_t ii=0; ii<n_mis;ii++){
					DMA_ADC_vett[ii]= 0;
				}
							DMA2_Stream0->NDTR = n_mis;									//DICIAMO AL DMA IL NUMERO DI MISURE CHE VENGONO FATTE DALL'ADC
							DMA2->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
							DMA2->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR				
							DMA2_Stream0->CR |= DMA_SxCR_EN; 						//ACCENDE IL DMA
				while(( DMA2_Stream0->CR & DMA_SxCR_EN )==0){}
							if (DMA2->LISR & DMA_LISR_FEIF0){
								DMA2_Stream0->CR &= ~DMA_SxCR_EN;
								while(( DMA2_Stream0->CR & DMA_SxCR_EN )){}
							DMA2->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
							DMA2->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR										
								DMA2_Stream0->CR |= DMA_SxCR_EN;
							}
//							USART3->CR1 &= ~USART_CR1_UE;
							ADC3->IER |= ADC_IER_EOCIE;
							TIM6->ARR = 1;
							ADC3->ISR |= ADC_ISR_OVR;
							if ((ADC3->CR & ADC_CR_ADSTART)==0){
										ADC3->CR |= ADC_CR_ADSTART;			//PARTE LA MISURA
							}
							USART3->CR1 |= USART_CR1_RXNEIE;
							while(GPIOC->IDR & GPIO_IDR_ID2){}
							confronto = 0;
							spara_trigger=0;
							TIM6->CR1 |= TIM_CR1_CEN;			  //ACCENDIAMO IL TIMER
//								while( spara_trigger ==0){}
			}
		}
		
		if(USART3->ISR & USART_ISR_TC){																	//SE E' ATTIVO IL FLAG DELLA TRASMISSIONE
				if (trasm == 1){
							USART3->TDR = puntatore[ind];
							trasm = 2;
				}
		}
/*	if ((USART3->ISR & USART_ISR_RXNE_RXFNE) == (USART_ISR_RXNE_RXFNE)){								//SE E' ATTIVATO IL FLAG RXNE (CIOE' SE HA RICEVUTO QUALCOSA)
			if(trasm == 0){   
					if (USART3->RDR == 'c'){
							trasm = 1;
							USART3->CR1 |= USART_CR1_TCIE;				//RIATTIVA L'INTERRUPT DI TRASMISSIONE
							ind = 1;
							USART3->TDR = puntatore_c1[0];						//SCRIVE IL PRIMO CARATTERE DA TRASMETTERE NEL DR
					}
					else if (USART3->RDR == 'd'){
							trasm = 2;
							USART3->CR1 |= USART_CR1_TCIE;				//RIATTIVA L'INTERRUPT DI TRASMISSIONE
							ind = 1;
							USART3->TDR = puntatore_c2[0];						//SCRIVE IL PRIMO CARATTERE DA TRASMETTERE NEL DR
					}
			}	
	}
																			
	if(USART3->ISR & USART_ISR_TC){																	//SE E' ATTIVO IL FLAG DELLA TRASMISSIONE
				if(trasm == 1){
						USART3->TDR = puntatore_c1[1];
						trasm = 0;
						ind = 0;
				}
				if(trasm == 2){
						USART3->TDR = puntatore_c2[2];
						ind++;
						trasm = 0;
						ind = 0;
				}
			
	}
*/	

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	USART3->ICR |= USART_ICR_TCCF;														//RIPRISTINA LO STATUS REGISTER
	USART3->RQR |= USART_RQR_RXFRQ;
  /* USER CODE END USART3_IRQn 1 */
	USART3->CR1 |= USART_CR1_RXNEIE;
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
	
//		TIM6->CR1 &= ~TIM_CR1_CEN;							// SPEGNIMENTO TIMER 6
//		DMA2_Stream0->CR &= ~DMA_SxCR_EN;				// SPEGNIMENTO DMA ADC
//		DMA2->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
//		DMA2->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR 
//	  DMA1_Stream3->CR |= DMA_SxCR_EN;					// ACCENSIONE DMA UART
//		USART3->CR3 |= USART_CR3_DMAT;					// PARTENZA DELL'USART
//	 	
	
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles ADC3 global interrupt.
  */
void ADC3_IRQHandler(void)
{
  /* USER CODE BEGIN ADC3_IRQn 0 */

	//if ((ADC3->ISR & ADC_ISR_EOC)==ADC_ISR_EOC){			  //SE HA RICEVUTO QUALCOSA DALL'ADC
		/*if (adc >= salvataggio){
				pendenza++;
		}
		else pendenza = 0;*/
		if (solouno == 0){
//					if ( (ADC3->DR > soglia) /*& (pendenza > 5)*/ ){					// SE HA RICEVUTO UN SEGNALE MAGGIORE DI UNA CERTA SOGLIA		
					if ( spara_trigger == 0/*& (pendenza > 5)*/ ){					// SE HA RICEVUTO UN SEGNALE MAGGIORE DI UNA CERTA SOGLIA		
						
					}else {
//					if( confronto==0){
//						confronto= GPIO_IDR_ID2;
						if(k==3){
						//TIM6->CR1 &= ~TIM_CR1_CEN;	
						soglia_sup = ((uint16_t)n_mis) - (DMA2_Stream0->NDTR);	
						
						if(DMA2_Stream0->NDTR <= (int16_t) post_trigger){
							tampone = DMA2_Stream0->NDTR + (int16_t) pre_trigger;
						}else{
							tampone = DMA2_Stream0->NDTR - (int16_t) post_trigger;
						}
						if (tampone==0) tampone++;
						if (tampone>100) tampone=99;
							solouno = 1;
//							k = 0;
					}else{
						k++;
						spara_trigger=0;
					}
//		}else{
//			confronto=0;
//		}
	}
}
		if(solouno == 1){
				if((tampone == DMA2_Stream0->NDTR)){
//				if((k<900)){
//					k++;
//				}else{
					TIM6->CR1 &= ~TIM_CR1_CEN;							// SPEGNIMENTO TIMER 6				
					DMA2_Stream0->CR &= ~DMA_SxCR_EN;				// SPEGNIMENTO DMA ADC								

		
					posizione = ((uint16_t)n_mis) - (DMA2_Stream0->NDTR);
					DMA_ADC_vett[n_mis]= posizione;

					DMA2->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
					DMA2->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR
					DMA1->LIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO LISR 
					DMA1->HIFCR = 0xffffffff;								// AZZERIAMO IL REGISTRO HISR					
					DMA1_Stream3->NDTR = n_mis * 2 +2;
					USART3->CR1 |= USART_CR1_UE;
					DMA1_Stream3->CR |= DMA_SxCR_TCIE;
					DMA1_Stream3->CR |= DMA_SxCR_EN;					// ACCENSIONE DMA UART
					USART3->CR3 |= USART_CR3_DMAT;					// PARTENZA DELL'USART
					ADC3->ISR |= ADC_ISR_OVR;
					solouno = 2;
				}
		}
		
		//salvataggio = adc;
	//}
  /* USER CODE END ADC3_IRQn 0 */
//  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC3_IRQn 1 */
		ADC3->ISR |= ADC_ISR_EOC;
  /* USER CODE END ADC3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
