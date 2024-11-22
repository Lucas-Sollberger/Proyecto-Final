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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include <string.h> // Necesario para memcpy
#include "usbd_cdc_if.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void DMA_Transfer(uint16_t *source, uint16_t *destination, uint16_t size);
void convert_Uint16_to_Uint8(uint16_t *srcArray, size_t srcLength, uint8_t *destArray);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))

// Variables del ADC
#define CANALES_ADC 8									//Cantidad de canales que muestrea el ADC
#define BUFFER_CANALES_ADC 10							//Cantidad de mediciones de cada canal que se guardara antes de enviarlos
#define ADC_BUFFER_SIZE CANALES_ADC*BUFFER_CANALES_ADC  //Size del buffer donde el DMA guardara las mediciones del ADC
#define ADC_BUFFER_HALF_SIZE ADC_BUFFER_SIZE/2

uint16_t adc1_array[ADC_BUFFER_SIZE]; 					//Array donde el DMA guarda las conversiones del ADC1 [Io, LEM, i30, Vo, IL, Vext, Pote, Vin]
uint16_t tx_buffer1[ADC_BUFFER_HALF_SIZE];				//Array donde se guarda la primera mitad de los datos muestreados por el ADC (adc1_array)
uint16_t tx_buffer2[ADC_BUFFER_HALF_SIZE];				//Array donde se guarda la segunda mitad de los datos muestreados por el ADC (adc1_array)

//Array que se va a enviar por USB. Para evitar que el ADC sobreescriba los datos del array antes de que estos
//sean envaidos, se utiliza una tecnica que consiste en:
//	1. Almacenar los datos del ADC en un Array (adc1_array)
//  2. Cuando el DMA lleno la mitad del array (adc1_array) mueve estos datos al array tx_buffer1
//  3. Como el array tx_buffer1 es de 16 bits, se convierten cada uno de sus valores a datos de 8 bits que se almacenan en el array "tx_buffer_final"
//     Por eso, el tamaño del array "tx_buffer_final" es el doble del array "tx_buffer1". Una vez convertido, el array "tx_buffer_final" es enviado por USB.
//  4. Mientras el punto 3 se ejecuta, el DMA continua llenando la segunda mitad del array "adc1_array", una vez terminado, se repite el punto 3 pero con el array "tx_buffer2"

uint8_t tx_buffer_final[ADC_BUFFER_SIZE+2]; 			//Tamaño del encabezado (2 bytes) + tamaño de los datos convertidos (2 * número de elementos en srcArray)

uint16_t duty_pwm = 0;
uint16_t trigger = 0;

uint16_t cont = 0;

//--Variables para la banda muerta
uint16_t x_prev = 0;
float deadBand = 0.05;

float duty= 0;

float bandaMuerta(float x) {		//Banda muerta para el potenciometro

	if (fabs(x - x_prev) < deadBand) {
		x = x_prev;
	} else {
		x_prev = x;
	}

	if (x < 0.1){
		return 0.1;
	}
	else if (x > 0.9){
		return 0.9;
	}
	else{
		return x;
	}
}

void set_period(float duty){

	duty_pwm = duty * (htim4.Init.Period + 1);

	if (duty > 0.5){
		trigger = (duty*0.5) * (htim8.Init.Period + 1);
	}
	else{
		trigger = ((1-duty)*0.5 + duty) * (htim8.Init.Period + 1);
	}

	trigger = duty_pwm;

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, duty_pwm);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3, trigger);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	DBGMCU->APB1FZ |= (DBGMCU_APB1_FZ_DBG_TIM4_STOP);//Detiene el clock del APB1 en Debug Mode
	DBGMCU->APB2FZ |= (DBGMCU_APB2_FZ_DBG_TIM8_STOP);//Detiene el clock del APB2 en Debug Mode

	__HAL_TIM_SET_COUNTER(&htim4, 0);//Reseteo los contadores de ambos timers para que cuenten en paralelo
	__HAL_TIM_SET_COUNTER(&htim8, 0);

	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc1_array, (sizeof(adc1_array) / sizeof(adc1_array[0]))) != HAL_OK) {//Inicio el DMA para enviar los datos del ADC a la variable "adc1_array" -> Iniciarlo antes que los Timers!!!
		Error_Handler();
	}


	if (HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK) {//Inicio el CH1 del Timer 4. Salida del  PWM
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3) != HAL_OK) {//Inicio el CH3 del Timer 8. Salida del  PWM
		Error_Handler();
	}

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, 0.5*(htim4.Init.Period + 1));
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3, 0.5*(htim8.Init.Period + 1));

	HAL_GPIO_WritePin(Rele_1_GPIO_Port, Rele_1_Pin, 1);
	HAL_GPIO_WritePin(Rele_2_GPIO_Port, Rele_2_Pin, 1);
	HAL_GPIO_WritePin(Rele_3_GPIO_Port, Rele_3_Pin, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(500);

	  duty = bandaMuerta(adc1_array[6]/4096.0);



	  set_period(duty);

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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Rele_1_Pin|Rele_2_Pin|Rele_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_ROJO_Pin|LED_AZUL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Rele_1_Pin Rele_2_Pin Rele_3_Pin */
  GPIO_InitStruct.Pin = Rele_1_Pin|Rele_2_Pin|Rele_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_VERDE_Pin */
  GPIO_InitStruct.Pin = LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED_VERDE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ROJO_Pin LED_AZUL_Pin */
  GPIO_InitStruct.Pin = LED_ROJO_Pin|LED_AZUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void convert_Uint16_to_Uint8(uint16_t *srcArray, size_t srcLength, uint8_t *destArray){

    // Agregar encabezado
    destArray[0] = 0xAA; // Primer byte del encabezado
    destArray[1] = 0x55; // Segundo byte del encabezado

    // Convertir uint16_t a uint8_t y agregar al array de destino
    for (size_t i = 0; i < srcLength; i++) {
        destArray[2 + 2 * i] = srcArray[i] & 0xFF;       	  // Parte baja del uint16_t
        destArray[2 + 2 * i + 1] = (srcArray[i] >> 8) & 0xFF; // Parte alta del uint16_t

    }


    ITM_Port32(2) = 1;
    CDC_Transmit_FS((uint8_t *) tx_buffer_final, sizeof(tx_buffer_final));	//Envia por USB los datos
    ITM_Port32(2) = 0;
}


void DMA_Transfer(uint16_t *source, uint16_t *destination, uint16_t size){

    ITM_Port32(20) = 0;
    // Inicia la transferencia DMA
    if (HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)source, (uint32_t)destination, size) != HAL_OK) {
        Error_Handler();         // Error de inicio
    }

    HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1, HAL_DMA_FULL_TRANSFER, 1);
    ITM_Port32(20) = 1;

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {									  	//Interrupcion que indica el fin de la conversion

	if (hadc->DMA_Handle == &hdma_adc1) {
		HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
		DMA_Transfer(&adc1_array[ADC_BUFFER_HALF_SIZE], &tx_buffer2[0], ADC_BUFFER_HALF_SIZE); 	//Transfiere la segunda mitad de los datos al registro tx_buffer2
		convert_Uint16_to_Uint8(tx_buffer2, ADC_BUFFER_HALF_SIZE, tx_buffer_final);
	}

}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {									//Interrupcion cuando el DMA lleno la mitad del buffer

	if (hadc->DMA_Handle == &hdma_adc1) {
		HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
		DMA_Transfer(&adc1_array[0], &tx_buffer1[0], ADC_BUFFER_HALF_SIZE);						//Transfiere la primer mitad de los datos al registro tx_buffer1
		convert_Uint16_to_Uint8(tx_buffer1, ADC_BUFFER_HALF_SIZE, tx_buffer_final);
	}

}




//Funcion Callback llamada por las interrupciones de los timers. Deben estar habilitadas en la configuracion
// Ingresa con el flanco descendente del PWM
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM8) {			//TRGO
		ITM_Port32(30) = 1;
		HAL_GPIO_TogglePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin);	//PD15
//
		HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 0); //PD14
	}

	if (htim->Instance == TIM4) {			//PWM -
		ITM_Port32(30) = 2;
		HAL_GPIO_TogglePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin); //PD12

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
