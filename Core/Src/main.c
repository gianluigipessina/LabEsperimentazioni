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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*int ts_data[50];															// VETTORE CHE MEMORIZZA I CODICI MISURATI DAL MICRO
#define TCAL1 *(uint16_t*)(0x1FF1E820)				// COSTANTE MISURATA IN FABBRICA A T=30°C
#define TCAL2 *(uint16_t*)(0x1FF1E840)				// COSTANTE MISURATA IN FABBRICA A T=110°C
int temperatura_mC[50];												// VETTORE CHE CONTIENE I DATI CONVERTITI IN TEMPERATURE IN GRADI °C

#define VREFINT_C *(uint16_t*)(0x1FF1E860)		// CODICE PRECISIO MISURATO IN FABBRICA A T=30°C, VDDA=3.3V 
#define VREF_PRECISA 3300											// COSTANTE = 3300mV
int v_ref_cal[50];														// VETTORE CHE CONTIENE LE ALIMENTAZIONI REALE CHE IL MICRO RICEVE ~3,3V
int codici[50];																// VETTORE CHE CONTIENE I CODICI MISURATI DAL MICRO; NUMERO PROPORZIONALE ALLA TENSIONE 
int tensione_milliV[50];											// VETTORE CHE CONTIENE LA TENSIONE INTERNA IN MILLIVOLT (RISULTATO DELLA CONVERSIONE DEI CODICI)
*/
int  n_mis = 100;																// NNUMERO DI VOLTE CHE RIPETIAMO LE MISURE
volatile int indice = 0;											// INIZIALIZZAZIONE DELL'INDICE
int soglia =20000;
uint16_t salvataggio;
volatile uint16_t DMA_ADC_vett[101];								// DICHIARIAMO IL VETTORE IN CUI MEMORIZZIAMO I DATI PER LA DMA

//int c1 = 
uint16_t posizione = 0;
volatile uint16_t soglia_sup = 0;
volatile uint16_t pre_trigger=10;
volatile  uint16_t post_trigger=90;
uint8_t *puntatore;
//PER TRASMISSIONE A MATLAB
//uint8_t *puntatore_temp;
//uint8_t *puntatore_mvolt;
/*
#define TCAL1 *(uint16_t*)(0x1FF1E820)				// COSTANTE MISURATA IN FABBRICA A T=30°C
#define TCAL2 *(uint16_t*)(0x1FF1E840)				// COSTANTE MISURATA IN FABBRICA A T=110°C
uint16_t a = 0;
uint16_t b = 0;
*/
uint8_t* puntatore_c1;
uint8_t* puntatore_c2;


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
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//PER LA TRASMISSIONE A MATLAB  -  ATTIVAZIONE DI TRASMISSIOE E RICEZIONE DELLA USART3
	USART3->CR1 |= USART_CR1_TE;			//BISOGNA METTERE OR PERCHE' COSI' VIENE MODIFICATO SOLO IL PIN DI TE IN CR1
	USART3->CR1 |= USART_CR1_RE;			//ALTRIMENTI VENGONO MESSI TUTTI A ZERO E UNO SU TE
	USART3->CR1 |= USART_CR1_UE;
	USART3->CR1 |= USART_CR1_RXNEIE;
//	USART3->CR1 |= USART_CR1_TCIE;
	
	//puntatore_temp = (uint8_t*) temperatura_mC;					// ASSEGNIAMO AI PUNTATORI L'INDIRIZZO DEI VETTORI
	//puntatore_mvolt = (uint8_t*) tensione_milliV;
	
	//PER L'ADC
//TEMPERATURA TENSIONE
/*	ADC3->SQR1 |= (18 << ADC_SQR1_SQ1_Pos) | (19 << ADC_SQR1_SQ2_Pos);		//INDICHIAMO QUALE CANALE (19, 18) VOGLIAMO LEGGERE IN QUALE REGISTRO (SQ1, SQ2)
	ADC3->SQR1 |= (1 << ADC_SQR1_L_Pos);																	//INDICHIAMO, NEL CAMPO L DI SQR1, IL NUMERO DI NODI CHE IL MICROCONTROLLORE DEVE LEGGERE (2)
	ADC3->PCSEL |= ADC_PCSEL_PCSEL_19 | ADC_PCSEL_PCSEL_18;								//INDICHIAMO QUALI CANALI STIAMO LEGGENDO NEL REGISTRO PCSEL (19 E 18)
	ADC3_COMMON->CCR |= 8<<ADC_CCR_PRESC_Pos;
*/
//OSCILLOSCOPIO
	ADC3->SQR1 = 0;
	ADC3->SQR1 |= (1 << ADC_SQR1_SQ1_Pos);
	ADC3->SQR1 |= (0 << ADC_SQR1_L_Pos);
	ADC3->PCSEL |= ADC_PCSEL_PCSEL_0;
	ADC3_COMMON->CCR |= 0<<ADC_CCR_PRESC_Pos;
	ADC3->CFGR |= (3<<ADC_CFGR_DMNGT_Pos);			//INIZIA L'ATTIVITA' DELL'ADC E PASSA I DATI AL DMA; 3 PER IL BUFFER CIRCOLARE, 1  PER MODALITA' NORMALE

	//CALIBRAZIONE E ATTIVAZIONE DELL'ADC
	ADC3->CR &= ~ADC_CR_ADCALDIF;			//ADCALDIF = 0 PER MISURA ENDED, ADCALDIF = 1 PER MISURA DIFFERENZIALE
	ADC3->CR |= ADC_CR_ADCALLIN;			//ADCALLIN = 1 PER AVERE CALIBRAZIONE LINEARE OLTRE CHE OFFSET, = 0 SE NON LA SI VUOLE. CONSIGLIATO 1
	ADC3->CR &= ~ADC_CR_ADEN;					//CON ADEN = 0 L'ADC NON E' ABILITATO
	ADC3->CR |= ADC_CR_ADCAL;					//ADCAL = 1 INIZIA LA CALIBRAZIONE, DIVENTA 0 QUANDO LA CALIBRAZIONE E' TERMINATA
	
	//ADC3_COMMON->CCR |= ADC_CCR_TSEN;  	  //ATTIVIAMO IL SENSORE DI TEMPERATURA 
	//ADC3_COMMON->CCR |= ADC_CCR_VREFEN;		//ATTIVIAMO IL SENSORE DELLA VEREF_IN

	while ((ADC3->CR & ADC_CR_ADCAL)){		//FINCHE' SI STA CALIBRANDO
			//ASPETTA
	} 
	
	ADC3->ISR |= ADC_ISR_ADRDY;				//AZZERIAMO IL BIT ADRDY SCRIVENDOGLI 1
	if ((ADC3->CR & ADC_CR_ADCAL) == 0){
				ADC3->CR |= ADC_CR_ADEN;				//ABILITA L'ADC
	}
	while ((ADC3->ISR & ADC_ISR_ADRDY) == 0){    //FINCHE' ADRDY == 0			ADC3->ISR & ADC_ISR_ADRDY = 1 SE IL BIT E' 1, = 0 SE IL BIT E' 0
			//ASPETTA
	}
	ADC3->ISR |= ADC_ISR_ADRDY;	
	
	// ATTIVAZIONE DELL'INTERRUPT
//	ADC3->IER = ADC_IER_EOCIE;				//ABILITIAMO L'INTERRUPT
	ADC3->IER = ADC_IER_EOCIE;
	
	
	//IMPOSTARE TEMPI DI ACQUISIZIONE LUNGHI IN SMPR1 E SMPR2
//	ADC3->SMPR1 |= ADC_SMPR1_SMP1_0;			//IMPOSTIAMO SMP1 = 111, CIOE' RALLENTIAMO LA MISURA DI X CICLI DI CLOCK DELL'ADC
//	ADC3->SMPR1 |= ADC_SMPR1_SMP1_1;
//	ADC3->SMPR1 |= ADC_SMPR1_SMP1_2;
	
	/*for(int i=0; i<n_mis; i++){						// INIZIALIZZIAMO A ZERO TUTTI GLI ELEMENTI DELL'ARRAY
				ts_data[i] = 0;
				codici[i] = 0;
		}
	*/
		
	///////// UNA VOLTA CHE L'ADC E' CALIBRATO, ACCESO E IMPOSTATO 
	//	IMPOSTO IL DMA DELL'ADC

	DMA2_Stream0->M0AR = (uint32_t) &DMA_ADC_vett;				// ASSEGNIAMO AL DMA DELL'ADC L'INDIRIZZO DELLA PRIMA CELLA DEL VETTORE IN CUI SONO MEMORIZZATI I DATI
	DMA2_Stream0->PAR = (uint32_t) (&ADC3->DR);						// ASSEGNIAMO AL DMA L'INDIRIZZO DI USART3->TDR, CIOE' QUELLO DOVE DEVE TRASMETTERE
	DMA2_Stream0->CR |= DMA_SxCR_CIRC;	
	//DMA2_Stream0->CR |= DMA_SxCR_TCIE;										// ABILITIAMO L'INTERRUPT
	
	uint16_t num_mis_adc = n_mis;															//NUMERO DI MISURE CHE VOGLIAMO FAR FARE ALL'ADC	
		
	//	IMPOSTO IL DMA DELL'UART
	DMA1_Stream3->M0AR = (uint32_t) &DMA_ADC_vett;				// ASSEGNIAMO AL DMA DELL'UART L'INDIRIZZO DELLA PRIMA CELLA DEL VETTORE IN CUI SONO MEMORIZZATI I DATI
	DMA1_Stream3->PAR = (uint32_t) &USART3->TDR;					// ASSEGNIAMO AL DMA L'INDIRIZZO DI ADC3->DR, CIOE' QUELLO DA DOVE RICEVERA' DATI
//  DMA1_Stream3->CR |= DMA_SxCR_TCIE;										// ABILITIAMO L'INTERRUPT
DMA1_Stream3->CR &= ~DMA_SxCR_EN ;
//	DMA1_Stream3->NDTR = num_mis_adc * 2;									// DICIAMO AL DMA IL NUMERO DI BYTE CHE DOVRA' TRASFERIRE ALL'USART

	//PARTE MISURA ADC
/*	DMA2_Stream0->CR |= DMA_SxCR_CIRC;
	DMA2_Stream0->NDTR = n_mis;									//DICIAMO AL DMA IL NUMERO DI MISURE CHE VENGONO FATTE DALL'ADC
	DMA2_Stream0->CR |= DMA_SxCR_EN; 						//ACCENDE IL DMA
	ADC3->CFGR |= (3<<ADC_CFGR_DMNGT_Pos);			//INIZIA L'ATTIVITA' DELL'ADC E PASSA I DATI AL DMA; 3 PER IL BUFFER CIRCOLARE, 1  PER MODALITA' NORMALE
		
	TIM6->ARR = 1;
	ADC3->CR |= ADC_CR_ADSTART;			//PARTE LA MISURA
	TIM6->CR1 |= TIM_CR1_CEN;			  //ACCENDIAMO IL TIMER
*/	
	//PER TRASMISSIONE A MATLAB
	//a = TCAL1;
	//b = TCAL2;
	//puntatore_c1 = (uint8_t*) 0x1FF1E820;
	//puntatore_c2 = (uint8_t*) 0x1FF1E840;
	
	puntatore = (uint8_t*) &posizione;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//if(indice==n_mis){
			
		//SPEGNIMENTO TIMER
		//TIM6->CR1	&= ~TIM_CR1_CEN;   //SPEGNIAMO IL TIMER
			
		// ELABORAZIONE TEMPERATURA
			/*	for (int i=0; i<n_mis; i++){
						if(ts_data[i] > TCAL1){
								temperatura_mC[i] = ( (80000 * (ts_data[i] - TCAL1)) / (TCAL2 - TCAL1) ) + 30000;
						}
						else {
								temperatura_mC[i] = 30000 - ( (80000 *(TCAL1 - ts_data[i])) / (TCAL2 - TCAL1));
						}
				}
		
				// ELABORAZIONE SEGNALE
				for(int i=0; i<n_mis; i++){
						v_ref_cal[i] = (VREFINT_C * VREF_PRECISA)/ codici[i];			//CALCOLO LA TENSIONE REALE FORNITA AL MICRO PER OGNI MISURA
						tensione_milliV[i] = (VREF_PRECISA * codici[i])/65536;		//CONVERTO IL CODICE LETTO DAL MICRO IN UNA TENSIONE IN MILLIVOLT
				}
				indice = 100;*/
		//}
	
		
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
