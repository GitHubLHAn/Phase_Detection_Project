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

#include "LoRa.h"
#include <string.h>

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ON_LED_DEBUG( )	HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET)
#define OFF_LED_DEBUG( )	HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET)

#define ON_LED_PA( )	HAL_GPIO_WritePin(LED_PA_GPIO_Port, LED_PA_Pin, GPIO_PIN_SET);
#define OFF_LED_PA( )	HAL_GPIO_WritePin(LED_PA_GPIO_Port, LED_PA_Pin, GPIO_PIN_RESET);


#define ON_LED_PB( )	HAL_GPIO_WritePin(LED_PB_GPIO_Port, LED_PB_Pin, GPIO_PIN_SET);
#define OFF_LED_PB( )	HAL_GPIO_WritePin(LED_PB_GPIO_Port, LED_PB_Pin, GPIO_PIN_RESET);


#define ON_LED_PC( )	HAL_GPIO_WritePin(LED_PC_GPIO_Port, LED_PC_Pin, GPIO_PIN_SET);
#define OFF_LED_PC( )	HAL_GPIO_WritePin(LED_PC_GPIO_Port, LED_PC_Pin, GPIO_PIN_RESET);


#define MA_SIZE 5

#define ZERO_PA 1972
#define ZERO_PB 1973
#define ZERO_PC 1970

typedef struct
{
    uint16_t buf[MA_SIZE];
    uint32_t sum;
    uint8_t index;
} MA_Filter_t;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

uint32_t tick = 0;
volatile uint32_t cnt_timetick = 0;

LoRa vLoRa;

uint8_t TX_Lora_buff[12];
uint8_t RX_LoRa_buff[12];


uint8_t send_ok = 0;
uint16_t num_RX_irq_LoRa = 0;
uint8_t flag_Lora_Rx = 0;

uint16_t config_lora = 0xFA;

uint8_t cnt = 0;

uint16_t sendOK = 0;
uint16_t recOK = 0;

uint32_t cycle = 40000;
		
volatile uint32_t last_zcA = 0;
volatile uint32_t now_zcA = 0;
volatile uint32_t interval_zcA = 0;

volatile uint32_t last_zcB = 0;
volatile uint32_t now_zcB = 0;
volatile uint32_t interval_zcB = 0;

volatile uint32_t last_zcC = 0;
volatile uint32_t now_zcC = 0;
volatile uint32_t interval_zcC = 0;

uint16_t delta_pB = 0, delta_pC = 0;
		
		// uint8_t phase_detected = 0;

volatile uint8_t flag_cnt_50us = 0;
volatile uint8_t flag_enable_zc = 0;


// sender
volatile uint8_t trigger_send_lora = 0;

volatile uint16_t adc_raw[3];

volatile uint16_t adc_filtered_pA = 0;
  volatile uint16_t adc_filtered_pB = 0;
    volatile uint16_t adc_filtered_pC = 0;

volatile uint16_t pA_cur = 0;
volatile uint16_t pA_prev = 0;
  volatile uint16_t pB_cur = 0;
  volatile uint16_t pB_prev = 0;
    volatile uint16_t pC_cur = 0;
    volatile uint16_t pC_prev = 0;

volatile uint32_t ovf_tim3 = 0;

uint8_t flag_tx_log = 0;
uint32_t cnt_log = 0;

char uart_tx_log[100];

uint32_t stt = 0;

uint16_t log_buff[1000];
uint16_t log_index = 0;

volatile uint8_t flag_get_zero = 0;
uint16_t offset_ZpA = 0;
  uint16_t offset_ZpB = 0;
    uint16_t offset_ZpC = 0;

MA_Filter_t adcFilter_pA;
  MA_Filter_t adcFilter_pB;
    MA_Filter_t adcFilter_pC;
	
	uint16_t cnt_detect_pA = 1;
		uint16_t cnt_detect_pB = 1;
	    uint16_t cnt_detect_pC = 1;

uint32_t last_send = 0;
uint32_t now_send = 0;
uint32_t interval_send = 0;
uint32_t interval_send_mod = 0;
uint32_t interval_send_div = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t MA_Update(MA_Filter_t *f, uint16_t sample)
{
    f->sum -= f->buf[f->index];
    f->buf[f->index] = sample;
    f->sum += sample;
    f->index++;

    if(f->index >= MA_SIZE)
        f->index = 0;

    return f->sum / MA_SIZE;
}

static inline uint32_t GetTimeUs(){
  uint32_t ovf1, ovf2;
  uint32_t cnt;

  do {
    ovf1 = ovf_tim3;
    cnt = TIM3->CNT;
    ovf2 = ovf_tim3;
  } while (ovf1 != ovf2); // Guard against rollover during reading

  return (ovf1 << 16) + cnt;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) //50us
  {
    cnt_timetick++;
		flag_cnt_50us++;
		
		if(flag_cnt_50us >= 2)    //100us
    {
      flag_cnt_50us = 0;
      adc_filtered_pA = MA_Update(&adcFilter_pA, adc_raw[0]);
			adc_filtered_pB = MA_Update(&adcFilter_pB, adc_raw[1]);
      adc_filtered_pC = MA_Update(&adcFilter_pC, adc_raw[2]);

      // adc_filtered_pA = adc_raw[0];
			// adc_filtered_pB = adc_raw[1];
      // adc_filtered_pC = adc_raw[2];

      pA_cur = adc_filtered_pA;
			pB_cur = adc_filtered_pB;
			pC_cur = adc_filtered_pC;

      // catch ZC phase A
      if(pA_cur != 0 && pA_prev != 0 && pA_prev <= ZERO_PA && pA_cur > ZERO_PA)
      {
				now_zcA = GetTimeUs();
				cnt_detect_pA = 10000; // 1s
        delta_pB = (now_zcA - last_zcB);
				delta_pC = (now_zcA - last_zcC);

        while(delta_pB > 20000) delta_pB -= 20000;
        while(delta_pC > 20000) delta_pC -= 20000;
      
				if(flag_enable_zc == 1 && cnt_detect_pB > 0 && cnt_detect_pC > 0){
					trigger_send_lora = 1;
					flag_enable_zc = 0;
          now_send = GetTimeUs();
          interval_send = now_send - last_send;
          last_send = now_send;
				}
							
				 interval_zcA = now_zcA - last_zcA;
				 last_zcA = now_zcA;
      }

      // catch ZC phase B
			if(pB_cur != 0 && pB_prev != 0 && pB_prev <= ZERO_PB && pB_cur > ZERO_PB)
      {
				now_zcB = GetTimeUs();
        cnt_detect_pB = 10000; // 1s
        interval_zcB = now_zcB - last_zcB;
        last_zcB = now_zcB;
			}
			
      // catch ZC phase C
			if(pC_cur != 0 && pC_prev != 0 && pC_prev <= ZERO_PC && pC_cur > ZERO_PC)
      {
				now_zcC = GetTimeUs();
				cnt_detect_pC = 10000; // 1s
				interval_zcC = now_zcC - last_zcC;
        last_zcC = now_zcC;
			}
			
			pA_prev = pA_cur;
			pB_prev = pB_cur;
			pC_prev = pC_cur;

    }
  
    if(cnt_detect_pA > 0) cnt_detect_pA--;
    if(cnt_detect_pB > 0) cnt_detect_pB--;
    if(cnt_detect_pC > 0) cnt_detect_pC--;
  }

  if(htim->Instance == TIM3){
    ovf_tim3++;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		/* Prevent unused argument(s) compilation warning */
		UNUSED(GPIO_Pin);

//		if(GPIO_Pin == vLoRa.DIO0_pin)
//		{
//			num_RX_irq_LoRa++;
//			flag_Lora_Rx = 1;	 
//		}
	
	}

void MA_Init(MA_Filter_t *f, uint16_t init_value)
{
    f->sum = 0;
    f->index = 0;

    for(int i=0; i<MA_SIZE; i++)
    {
        f->buf[i] = init_value;
        f->sum += init_value;
    }
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);

  ON_LED_PA();
    ON_LED_PB();
      ON_LED_PC();
	
	Lora_Init(&vLoRa, &hspi1, 808);
	LoRa_reset(&vLoRa);
		
	config_lora = LoRa_Config(&vLoRa);
	
	while(config_lora!=0x00C8){
			HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin, GPIO_PIN_SET);	

	LoRa_startReceiving(&vLoRa);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw, 3);



  MA_Init(&adcFilter_pA, ZERO_PA);
	  MA_Init(&adcFilter_pB, ZERO_PB);
	    MA_Init(&adcFilter_pC, ZERO_PC);

	
	HAL_GPIO_WritePin(GenOut_GPIO_Port, GenOut_Pin, GPIO_PIN_RESET);
	
	OFF_LED_PA();
		OFF_LED_PB();
			OFF_LED_PC();

		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		tick++;
		
		if(cnt_timetick >= cycle)
    {
      cnt_timetick = 0;
			//HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
      flag_enable_zc = 1;
    }

		if(flag_get_zero){
			flag_get_zero = 0;
			
			uint32_t sumA = 0;
				uint32_t sumB = 0;
						uint32_t sumC= 0;
			for(uint16_t i=0; i<1000; i++){
				sumA+= adc_raw[0];
					sumB+= adc_raw[1];
						sumC+= adc_raw[2];
				HAL_Delay(1);
			}	
			offset_ZpA = sumA/1000;
				offset_ZpB = sumB/1000;
					offset_ZpC = sumC/1000;
		}

				// code mach sender
    if(trigger_send_lora == 1)
    {							
      HAL_GPIO_WritePin(GenOut_GPIO_Port, GenOut_Pin, GPIO_PIN_SET);

      TX_Lora_buff[0] = 0xAA;
      TX_Lora_buff[1] = 0;
      TX_Lora_buff[2] = delta_pB/100;
      TX_Lora_buff[3] = delta_pC/100;
      TX_Lora_buff[4] = TX_Lora_buff[0] + TX_Lora_buff[1] + TX_Lora_buff[2] + TX_Lora_buff[3];

			send_ok = LoRa_transmit(&vLoRa, (uint8_t*)TX_Lora_buff, 5, 10000000);
			HAL_GPIO_WritePin(GenOut_GPIO_Port, GenOut_Pin, GPIO_PIN_RESET);

      interval_send_mod = interval_send % 20000;
      interval_send_div = interval_send / 20000;
			
			if(send_ok == 1){
				 HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
					sendOK++;
			}
			trigger_send_lora = 0;
    }


    if(cnt_detect_pA > 0){
      ON_LED_PA();
    }
    else{
      OFF_LED_PA();
    }
    if(cnt_detect_pB > 0){
      ON_LED_PB();
    }
    else{
      OFF_LED_PB();
    }
    if(cnt_detect_pC > 0){
      ON_LED_PC();
    }
    else{
      OFF_LED_PC();
    }
		





    		
		
		
		
//		if(flag_tx_log == 1 && cnt_log < 20000){
//			cnt_log++;
//			flag_tx_log = 0;
//			//sprintf(uart_tx_log, "%u %u %u %d\n",++stt, adc_raw_pA, adc_filtered_pA, interval_zcA);
//			sprintf(uart_tx_log, "%u %u %u %u\n",++stt, adc_raw[0], adc_raw[1], adc_raw[2]);
//			//sprintf(uart_tx_log, "%u %u %u %d\n",++stt, adc_filtered_pA, adc_filtered_pB, adc_filtered_pC);


//			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_tx_log, strlen(uart_tx_log));
//		}
		
	
		
		
//			if(flag_Lora_Rx == 1){
//				flag_Lora_Rx = 0;
//				
//				LoRa_receive(&vLoRa, (uint8_t*)RX_LoRa, 12);
//				
//				if(memcmp(RX_LoRa, TX_toRB, 12) == 0){
//					HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
//					interval = TIM3->CNT - start;;
//					//interval = HAL_GetTick() - start;
//					recOK++;
//				}
//			}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  huart1.Init.BaudRate = 500000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_DEBUG_Pin|INT_GEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_RESET_Pin|GenOut_Pin|LED_PC_Pin|LED_PB_Pin
                          |LED_PA_Pin|Sender_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_DEBUG_Pin INT_GEN_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_Pin|INT_GEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_RESET_Pin GenOut_Pin LED_PC_Pin LED_PB_Pin
                           LED_PA_Pin Sender_Pin */
  GPIO_InitStruct.Pin = RF_RESET_Pin|GenOut_Pin|LED_PC_Pin|LED_PB_Pin
                          |LED_PA_Pin|Sender_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO0_Pin PB10 */
  GPIO_InitStruct.Pin = DIO0_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
