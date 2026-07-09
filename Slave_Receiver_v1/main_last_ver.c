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

#define ON_LED_DEBUG( )	HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_RESET)
#define OFF_LED_DEBUG( )	HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_SET)
#define TOGGLE_LED_DEBUG( )	HAL_GPIO_TogglePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin)

#define ON_LED_PA( )	HAL_GPIO_WritePin(LED_PA_GPIO_Port, LED_PA_Pin, GPIO_PIN_SET);
#define OFF_LED_PA( )	HAL_GPIO_WritePin(LED_PA_GPIO_Port, LED_PA_Pin, GPIO_PIN_RESET);

#define ON_LED_PB( )	HAL_GPIO_WritePin(LED_PB_GPIO_Port, LED_PB_Pin, GPIO_PIN_SET);
#define OFF_LED_PB( )	HAL_GPIO_WritePin(LED_PB_GPIO_Port, LED_PB_Pin, GPIO_PIN_RESET);

#define ON_LED_PC( )	HAL_GPIO_WritePin(LED_PC_GPIO_Port, LED_PC_Pin, GPIO_PIN_SET);
#define OFF_LED_PC( )	HAL_GPIO_WritePin(LED_PC_GPIO_Port, LED_PC_Pin, GPIO_PIN_RESET);

#define MA_SIZE 5

#define ZERO_PS 1971

#define CYCLE_GRID 20000

#define RANGE_GRID_L 19500
#define RANGE_GRID_H 20500

#define TIME_PHASE_DETECT 10000   // 1s

#define pA_L   6000
#define pA_H   8000

#define pB_L   1900
#define pB_H   867

#define pC_L   1367
#define pC_H   1567


typedef struct
{
    uint16_t buf[MA_SIZE];
    uint32_t sum;
    uint8_t index;
} MA_Filter_t;

typedef enum
{
  MODE_OFF = 0,
  MODE_STS_1 = 1,
  MODE_STS_2 = 2,

  MODE_SET_FLASH_1 = 3,
  MODE_FLASH_1 = 4,

  MODE_SET_FLASH_2 = 5,
  MODE_FLASH_2 = 6,
}MODE_LED_e;

typedef struct {
    uint16_t* adc_raw_ptr;       // Trỏ tới adc_raw[x]
    MA_Filter_t filter;         // Trỏ tới bộ lọc tương ứng
    uint16_t zero_val;     // Giá trị ZERO_PX
    uint16_t p_cur;
    uint16_t p_prev;
    uint16_t cnt_detect;
    uint32_t last_zc;
    uint32_t now_zc;
    uint32_t interval_zc;
    bool zc_ok;
    bool has_new_edge;          // Cờ báo hiệu có cạnh lên mới cho Main xử lý
} Phase_Data_t;




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint32_t tick_slave = 0;

uint32_t cycle = 20000;

volatile uint32_t ovf_tim3 = 0;
volatile uint8_t flag_cnt_50us = 0;

//LoRa variables
LoRa vLoRa;
uint16_t config_lora = 0xFA;

uint8_t TX_Lora_buff[12];
uint8_t RX_LoRa_buff[12];

uint8_t send_ok = 0;
uint16_t cnt_recOK = 0;
volatile uint16_t num_RX_irq_LoRa = 0;
volatile bool flag_Lora_Rx = false;

// Phase variables	
Phase_Data_t phaseS;


// For logging
uint8_t flag_tx_log = 0;
char uart_tx_log[100];
uint32_t stt = 0;



// For calib ADC at zeropoint
uint8_t flag_get_zero = 0;
uint16_t offset_pS = 0;

// Debug variables
uint16_t delta_pA = 0;
uint16_t delta_pB = 0;
uint16_t delta_pC = 0;

uint64_t tx_time = 0;
uint64_t rx_time = 0;
uint32_t deltaT = 0;
uint32_t LN = 0;

uint32_t latency = 0;
					
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
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void Get_Offset(void);
void OFF_LED_STATUS(void);
void ON_LED_STATUS_1(void);
void ON_LED_STATUS_2(void);
void MA_Init(MA_Filter_t *f, uint16_t init_value);
uint16_t MA_Update(MA_Filter_t *f, uint16_t sample);

uint32_t GetTimeUs(){
  uint32_t ovf1, ovf2;
  uint32_t cnt;

  do {
    ovf1 = ovf_tim3;
    cnt = TIM3->CNT;
    ovf2 = ovf_tim3;
  } while (ovf1 != ovf2); // Guard against rollover during reading

  return ((uint32_t)ovf1 << 16) + cnt;
}

void Phase_init(Phase_Data_t* p_data, uint16_t* adc_raw_ptr, uint16_t zero_val){
    p_data->adc_raw_ptr = adc_raw_ptr;
    p_data->zero_val = zero_val;
    p_data->p_cur = 0;
    p_data->p_prev = 0;
    p_data->cnt_detect = 0;
    p_data->last_zc = 0;
    p_data->now_zc = 0;
    p_data->interval_zc = 0;
    p_data->zc_ok = false;
    p_data->has_new_edge = false;

    MA_Init(&p_data->filter, 0);
}

static inline void Process_Phase_ZC(Phase_Data_t *phase, uint32_t now_time)
{
    phase->p_cur = MA_Update(&phase->filter, *(phase->adc_raw_ptr));

    uint32_t itv_lZC = now_time - phase->last_zc;
    

    /* Detect loss grid */
    if ((itv_lZC > 100000) && (phase->last_zc != 0))
    {
        phase->zc_ok       = false;
        phase->cnt_detect  = 0;
        phase->last_zc     = 0;
    }

    /* Rising Zero-Cross */
    if ((phase->p_prev <= phase->zero_val) &&(phase->p_cur  >  phase->zero_val))
    {
        while(itv_lZC > RANGE_GRID_H){
          itv_lZC -= CYCLE_GRID;
        }

        /* First edge after startup or loss-grid */
        if (phase->last_zc == 0U)
        {
            phase->last_zc = now_time;
        }
        else
        {
            if ((itv_lZC >= RANGE_GRID_L) && (itv_lZC <= RANGE_GRID_H))
            {
                phase->now_zc       = now_time;
                phase->interval_zc  = itv_lZC;
                phase->has_new_edge = true;

                phase->last_zc = now_time;

                if (phase->cnt_detect < TIME_PHASE_DETECT){
                  if (++phase->cnt_detect == TIME_PHASE_DETECT){
                    phase->zc_ok = true;
                  }
                }
            }
        }
    }
    phase->p_prev = phase->p_cur;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    cnt_timetick++;
		flag_cnt_50us++;
		 if(flag_cnt_50us >= 2)    // 50us or 100us
    {
      flag_cnt_50us = 0;
      //flag_tx_log = 1;
      adc_filtered_pS = MA_Update(&adcFilter_pS, adc_raw_pS);

      pS_cur = adc_filtered_pS;

      if(pS_cur != 0 && pS_prev != 0)
      {
          if(pS_prev <= ZERO_PS && pS_cur > ZERO_PS)
          {
              //HAL_GPIO_WritePin(GenOut_GPIO_Port, GenOut_Pin, GPIO_PIN_SET);
              now_zc = GetTimeUs();
              interval_zc = now_zc - last_zc;
              last_zc = now_zc;
							//flag_tx_log = 1;
          }
      }

      pS_prev = pS_cur;
    }
  }
  if(htim->Instance == TIM3)
  {
    ovf_tim3++;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		/* Prevent unused argument(s) compilation warning */
		UNUSED(GPIO_Pin);

		if(GPIO_Pin == vLoRa.DIO0_pin)
		{
			num_RX_irq_LoRa++;
			flag_Lora_Rx = 1;	 
		}
//		if(GPIO_Pin == GPIO_PIN_10)
//		{
//			tx_time = GetTimeUs(); 
//		}
	
	}






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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	 HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_RESET);
	 ON_LED_PA();
	 ON_LED_PB();
	 ON_LED_PC();

	
	Lora_Init(&vLoRa, &hspi1, 912);
	LoRa_reset(&vLoRa);
		
	config_lora = LoRa_Config(&vLoRa);
	
	while(config_lora!=0x00C8){
			HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_RESET);

	}
	HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_SET);	

	LoRa_startReceiving(&vLoRa);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw_pS, 1);
	
  MA_Init(&adcFilter_pS, ZERO_PS);
			
	
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
		
		tick_slave++;
		
		if(cnt_timetick >= cycle)
    {
      cnt_timetick = 0;
			//HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
    }


		
		// code mach receiver
    if(flag_Lora_Rx == 1)
    {				
      flag_Lora_Rx = 0;
			send_ok = LoRa_receive(&vLoRa, (uint8_t*)RX_LoRa_buff, 5);

      uint8_t checksum = RX_LoRa_buff[0] + RX_LoRa_buff[1] + RX_LoRa_buff[2] + RX_LoRa_buff[3];

      if(RX_LoRa_buff[0] == 0xAA && checksum == RX_LoRa_buff[4])
      {
          HAL_GPIO_TogglePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin);
          rx_time = GetTimeUs();
          deltaT = (rx_time - last_zc)%20000;
					LN = (rx_time - last_zc)/20000;
					latency = rx_time - tx_time;
			 	
					delta_pA = RX_LoRa_buff[1]*100;
					delta_pB = RX_LoRa_buff[2]*100;
					delta_pC = RX_LoRa_buff[3]*100;

          if((deltaT > 0 && deltaT < 6000) || (deltaT > 19500 && deltaT < 20000))
          {
            ON_LED_PA();
						OFF_LED_PB();
						OFF_LED_PB();
          }
          else if(deltaT > 8000 && deltaT < 12000)
          {
						if(delta_pC > delta_pB){
							OFF_LED_PA();
							ON_LED_PB();
							OFF_LED_PC();
						}
						else{
							OFF_LED_PA();
							OFF_LED_PB();
							ON_LED_PC();
						}
            
          }
          else if(deltaT > 14000 && deltaT < 18000)
          {
            if(delta_pC > delta_pB){
							OFF_LED_PA();
							OFF_LED_PB();
							ON_LED_PC();
						}
						else{
							OFF_LED_PA();
							ON_LED_PB();
							OFF_LED_PC();
						}
          }
					else{
						OFF_LED_PA();
							OFF_LED_PB();
							OFF_LED_PC ();
					}
					
					//flag_tx_log = 1;

      }
    }

    if(flag_tx_log == 1){
			flag_tx_log = 0;
			//sprintf(uart_tx_log, "%u %u %u\n",++stt, adc_raw_pS, adc_filtered_pS);
			
			//sprintf(uart_tx_log, "%u %u %u %u\n",++stt, adc_raw_pS, adc_filtered_pS, interval_zc);

			sprintf(uart_tx_log, "%u %u %u %u %u\n",++stt, deltaT, LN, delta_pB, delta_pC);

			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_tx_log, strlen(uart_tx_log));
		}
		
    Get_Offset();

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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_Pin|GPIO_Spare0_Pin|MCU_BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_RESET_Pin|GPIO_Spare1_Pin|LED_PA_Pin|LED_PB_Pin
                          |LED_PC_Pin|LED_STATUS_SLAVE_2_Pin|LED_STATUS_SLAVE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_DEBUG_ON_BOARD_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_ON_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_DEBUG_ON_BOARD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin GPIO_Spare0_Pin MCU_BUZZER_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|GPIO_Spare0_Pin|MCU_BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_RESET_Pin GPIO_Spare1_Pin LED_STATUS_SLAVE_2_Pin LED_STATUS_SLAVE_1_Pin */
  GPIO_InitStruct.Pin = RF_RESET_Pin|GPIO_Spare1_Pin|LED_STATUS_SLAVE_2_Pin|LED_STATUS_SLAVE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PA_Pin LED_PB_Pin LED_PC_Pin */
  GPIO_InitStruct.Pin = LED_PA_Pin|LED_PB_Pin|LED_PC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Get_Offset(void)
{  
  if(flag_get_zero == 0) return;
  flag_get_zero = 0;

  uint32_t sumS = 0;

  for(int i=0; i<1000; i++)
  {
    sumS += adc_raw_pS;
    HAL_Delay(1);
  }

  offset_pS = sumS/1000;
}

void OFF_LED_STATUS(void)
{
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_1_GPIO_Port, LED_STATUS_SLAVE_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_2_GPIO_Port, LED_STATUS_SLAVE_2_Pin, GPIO_PIN_SET);
}

void ON_LED_STATUS_1(void)
{
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_1_GPIO_Port, LED_STATUS_SLAVE_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_2_GPIO_Port, LED_STATUS_SLAVE_2_Pin, GPIO_PIN_SET);
}

void ON_LED_STATUS_2(void)
{
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_1_GPIO_Port, LED_STATUS_SLAVE_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_2_GPIO_Port, LED_STATUS_SLAVE_2_Pin, GPIO_PIN_RESET);
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
