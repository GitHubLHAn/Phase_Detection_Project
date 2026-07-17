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

#include "flash.h"

#include <string.h>

#include <stdio.h>

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t buf[MA_SIZE_MAX];
    uint32_t sum;
    uint8_t index;
    uint16_t size;
} MA_Filter_t;

typedef enum
{
  MODE_IDLE = 0xFF,
  MODE_OFF = 0,
  MODE_STS_G = 1,
  MODE_STS_R = 2,

  MODE_SET_FLASH_G = 3,
  MODE_FLASH_G = 4,

  MODE_SET_FLASH_R = 5,
  MODE_FLASH_R = 6,
}MODE_LED_e;

typedef enum
{
  BUZZER_OFF = 0,
  BUZZER_ON = 1,
  BUZZER_SET_TRIGGER_PIP = 2,
  BUZZER_TRIGGER_PIP = 3,
}MODE_BUZZER_e;

typedef enum
{
  NORMAL = 0,
  BAT_LOW = 1,
}SLAVE_STATUS_e;

typedef enum
{
  WAIT_NEG = 1,
  WAIT_ZC_UP = 2,
  WAIT_POS = 3,
  LOSS_GRID = 4,
}MODE_DETECT_e;

typedef struct {
    volatile uint16_t* adc_raw_ptr;       // Trỏ tới adc_raw[x]
    MA_Filter_t filter;         // Trỏ tới bộ lọc tương ứng
    uint16_t zero_val;     // Giá trị ZERO_PX
    volatile bool phase_detected;
    volatile uint16_t p_cur;
    volatile uint16_t p_prev;
    volatile uint16_t cnt_detect;
    volatile uint32_t last_zc;
    volatile uint32_t now_zc;
    volatile uint32_t interval_zc;
    volatile bool has_new_edge;          // Cờ báo hiệu có cạnh lên mới cho Main xử lý

    volatile MODE_DETECT_e mode_det;
    volatile uint16_t cnt_det_loss_grid;
  } Phase_Data_t;

typedef struct {
    volatile uint16_t* adc_raw_ptr;
    MA_Filter_t filter;
    float vBat_raw, vBat_filtered;
    uint16_t adc_filterd;

    uint16_t cnt_low_bat;
    uint16_t cnt_full_bat;

    bool flag_track_charge_full;

    uint32_t last_time;
    uint32_t interval_time;
    uint32_t now_time;
}Battery_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define DEBUG

#define ON_LED_DEBUG( )	HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_SET)
#define OFF_LED_DEBUG( )	HAL_GPIO_WritePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin, GPIO_PIN_RESET)
#define TOGGLE_LED_DEBUG( )	HAL_GPIO_TogglePin(LED_DEBUG_ON_BOARD_GPIO_Port, LED_DEBUG_ON_BOARD_Pin)

#define ON_LED_PA( )	HAL_GPIO_WritePin(LED_PA_GPIO_Port, LED_PA_Pin, GPIO_PIN_SET);
#define OFF_LED_PA( )	HAL_GPIO_WritePin(LED_PA_GPIO_Port, LED_PA_Pin, GPIO_PIN_RESET);

#define ON_LED_PB( )	HAL_GPIO_WritePin(LED_PB_GPIO_Port, LED_PB_Pin, GPIO_PIN_SET);
#define OFF_LED_PB( )	HAL_GPIO_WritePin(LED_PB_GPIO_Port, LED_PB_Pin, GPIO_PIN_RESET);

#define ON_LED_PC( )	HAL_GPIO_WritePin(LED_PC_GPIO_Port, LED_PC_Pin, GPIO_PIN_SET);
#define OFF_LED_PC( )	HAL_GPIO_WritePin(LED_PC_GPIO_Port, LED_PC_Pin, GPIO_PIN_RESET);

// Buzzer active
#define BUZZER_ON()  HAL_GPIO_WritePin(MCU_BUZZER_GPIO_Port, MCU_BUZZER_Pin, GPIO_PIN_SET);
#define BUZZER_OFF()  HAL_GPIO_WritePin(MCU_BUZZER_GPIO_Port, MCU_BUZZER_Pin, GPIO_PIN_RESET);

#define MA_SIZE_MAX     100
#define MA_SIZE_PHASE_ZC  5
#define MA_SIZE_BAT       8

#define MAX_SIZE_LORA 12

// #define ZERO_PS 1978

#define CYCLE_GRID 20000

#define RANGE_GRID_L 19000
#define RANGE_GRID_H 21000

#define TIME_DET_PHASE 50   //      50*20ms = 1s
#define LOSS_GRID_TIMEOUT 1000   //   100ms/0.1ms = 1000

#define pA_L   0
#define pA_H   6667

#define pB_L   13333
#define pB_H   20000

#define pC_L   6667
#define pC_H   13333

#define BATTERY_LOW   9.5
#define BATTERY_FULL  12.0



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint32_t tick_slave = 0;
uint32_t cnt_timetick = 0;
uint32_t cycle_timetick = 20000;

volatile uint32_t ovf_tim3 = 0;
volatile uint8_t flag_cnt_50us = 0;

// LED
volatile MODE_LED_e mode_led = MODE_IDLE;
volatile uint16_t cnt_handle_led = 0;

//BUZZER
volatile MODE_BUZZER_e mode_buzzer = BUZZER_OFF;
volatile uint16_t cnt_handle_buzzer = 0;

//LoRa variables
LoRa vLoRa;
uint16_t config_lora = 0xFA;

uint8_t TX_Lora_buff[MAX_SIZE_LORA];
uint8_t RX_LoRa_buff[MAX_SIZE_LORA];

uint8_t rec_ok = 0;
uint16_t cnt_recOK = 0;
volatile uint16_t num_RX_irq_LoRa = 0;
volatile bool flag_Lora_Rx = false;

SLAVE_STATUS_e slave_status = NORMAL;

// Phase variables	
Phase_Data_t phaseS;

// ADC measurement variables
volatile uint16_t adc_raw[2];

// Battery measurement
Battery_Data_t battery;

// For calib ADC at zeropoint
uint8_t flag_get_zero = 0;
uint16_t offset_pS = 0;

// For logging
uint8_t flag_tx_log = 0;
char uart_tx_log[100];
uint32_t stt = 0;

// Other variables
uint16_t delta_pA = 0;
uint16_t delta_pB = 0;
uint16_t delta_pC = 0;

uint32_t tx_time = 0, rx_time = 0;
uint32_t deltaT_mod = 0, deltaT_div = 0;
uint32_t latency = 0;

uint16_t point_pA = 0, point_pB = 0, point_pC = 0;

					
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
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void Get_Offset(void);
void Handle_LED(void);
void Handle_Buzzer(void);
void Battery_Handle(Battery_Data_t *pBat);
void Handle_LoRa_RX(void);
void OFF_LED_STATUS(void);
void ON_LED_STATUS_G(void);
void ON_LED_STATUS_R(void);
void MA_Init(MA_Filter_t *f, uint16_t init_value, uint16_t sizeMA);
uint16_t MA_Update(MA_Filter_t *f, uint16_t sample);

uint32_t GetTimeUs(){
  uint32_t ovf1, ovf2;
  uint32_t cnt;

  do {
    ovf1 = ovf_tim3;
    cnt = TIM3->CNT;
    ovf2 = ovf_tim3;
  } while (ovf1 != ovf2); // Guard against rollover during reading

  return (ovf1 << 16) + cnt;
}

void Phase_init(Phase_Data_t* p_data, volatile  uint16_t* adc_raw_ptr, uint16_t zero_val){
    p_data->adc_raw_ptr = adc_raw_ptr;
    p_data->zero_val = zero_val;
    p_data->p_cur = 0;
    p_data->p_prev = 0;
    p_data->cnt_detect = 0;
    p_data->last_zc = 0;
    p_data->now_zc = 0;
    p_data->interval_zc = 0;
    p_data->phase_detected = false;
    p_data->has_new_edge = false;
    MA_Init(&p_data->filter, zero_val, MA_SIZE_PHASE_ZC);

    p_data->mode_det = WAIT_NEG;
    p_data->cnt_det_loss_grid = 0;
}

static inline void Process_Phase_ZC(Phase_Data_t *phase, uint32_t now_time)
{
    phase->p_cur = MA_Update(&phase->filter, *(phase->adc_raw_ptr));

    if(phase->phase_detected){
      if(++phase->cnt_det_loss_grid >= LOSS_GRID_TIMEOUT){
        phase->cnt_det_loss_grid = 0;
        mode_det = LOSS_GRID;
      }
    } 

    switch(phase->mode_det)
    {
      case WAIT_NEG:
        if (phase->p_cur < phase->zero_val - 100)
        {
          if(phase->phase_detected == true){
            phase->mode_det = WAIT_ZC_UP;
          }else{
            phase->mode_det = WAIT_POS;
          }
        }
        break;
      case WAIT_POS:
        if (phase->p_cur > phase->zero_val + 100)
        {
          if(++phase->cnt_detect == TIME_DET_PHASE){
            phase->phase_detected = true;
          }
          phase->mode_det = WAIT_NEG;
        }
        break;
      case WAIT_ZC_UP: 
			{
        uint32_t itv_lZC = now_time - phase->last_zc;

        /* Rising Zero-Cross */
        if (phase->p_prev <= phase->zero_val && phase->p_cur>phase->zero_val)
        {
            /* First edge after startup or loss-grid */
            if (phase->last_zc == 0U)
            {
                phase->last_zc = now_time;
            }
            else
            {
                while(itv_lZC > RANGE_GRID_H){
                  itv_lZC -= CYCLE_GRID;
                }
                if ((itv_lZC >= RANGE_GRID_L) && (itv_lZC <= RANGE_GRID_H))
                {
                  phase->now_zc       = now_time;
                  phase->interval_zc  = itv_lZC;
                    
                  phase->has_new_edge = true;
                  phase->last_zc = now_time;
                  phase->cnt_det_loss_grid = 0;
                }			
            }
            phase->mode_det = WAIT_NEG;
        }
        phase->p_prev = phase->p_cur;
        break;
			}
      case LOSS_GRID:
        phase->phase_detected = false;
        phase->cnt_detect  = 0;
        phase->last_zc     = 0;
        phase->now_zc = 0;
        phase->mode_det = WAIT_NEG;
        break;
    }
    
  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    cnt_timetick++;
    cnt_handle_led++;
    cnt_handle_buzzer++;
    
    if(++flag_cnt_50us >= 2)    // 50us or 100us
    {
      uint32_t now_time = GetTimeUs();
      flag_cnt_50us = 0;

      Process_Phase_ZC(&phaseS, now_time);
    }
  }

  if(htim->Instance == TIM3)        // every 65535us
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
    flag_Lora_Rx = true;	 
  }
#ifdef DEBUG
  if(GPIO_Pin == GET_IRQ_Pin)
  {
    tx_time = GetTimeUs(); 
  }
#endif
}

void Battery_init(Battery_Data_t* pBat, volatile  uint16_t* adc_raw_ptr){
    pBat->adc_raw_ptr = adc_raw_ptr;
    pBat->vBat_raw = 0.0f;
    pBat->vBat_filtered = 0.0f;
    pBat->adc_filterd = 0;
   
    pBat->last_time = 0;
    pBat->interval_time = 0;
    pBat->now_time = 0;

    pBat->cnt_low_bat = 0;
    pBat->cnt_full_bat = 0;
    pBat->flag_track_charge_full = false;
    MA_Init(&pBat->filter, 0, MA_SIZE_BAT);

    p_data->mode_det = WAIT_NEG;
    p_data->cnt_det_loss_grid = 0;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	ON_LED_DEBUG();
  ON_LED_STATUS_G();

  ON_LED_PA();
  ON_LED_PB();
  ON_LED_PC();

	Lora_Init(&vLoRa, &hspi1, 912);
	LoRa_reset(&vLoRa);
	config_lora = LoRa_Config(&vLoRa);
	
	while(config_lora!=0x00C8){
		ON_LED_DEBUG(); 
    Lora_Init(&vLoRa, &hspi1, 912);
	  LoRa_reset(&vLoRa);
    config_lora = LoRa_Config(&vLoRa);
		OFF_LED_DEBUG();
		HAL_Delay(15);
	}
  LoRa_startReceiving(&vLoRa);
  // Config Lora ok
  OFF_LED_DEBUG();
  OFF_LED_STATUS();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw, 2);

  Phase_init(&phaseS, &adc_raw[0], ZERO_PS);

  Battery_init(&battery, &adc_raw[1]);

	BUZZER_ON(); HAL_Delay(15);
	BUZZER_OFF(); HAL_Delay(100);
  OFF_LED_PA();
	BUZZER_ON(); HAL_Delay(15);
	BUZZER_OFF(); HAL_Delay(100);
  OFF_LED_PB();
	BUZZER_ON(); HAL_Delay(15);
	BUZZER_OFF(); HAL_Delay(100);
  OFF_LED_PC();
	
	HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
	
	uint16_t cnt_get_zero = 0;
	while(HAL_GPIO_ReadPin(SET_MODE_GPIO_Port, SET_MODE_Pin) == GPIO_PIN_RESET){
		if(++cnt_get_zero == 3000){
			flag_get_zero = true;
			break;
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		tick_slave++;
		
		if(cnt_timetick >= cycle_timetick)
    {
      cnt_timetick = 0;
			TOGGLE_LED_DEBUG();
    }

		// Handle receive packet Lora
		Handle_LoRa_RX();
		
    // Handle LED status
    Handle_LED();

    // Handle Buzzer
    Handle_Buzzer();

    // Battery Voltage Meas
    Battery_Handle(&battery);
		
		// Get offset for ADC measurement
    Get_Offset();

#ifdef DEBUG
    // For debug
    if(flag_tx_log == 1){
			flag_tx_log = 0;
			//sprintf(uart_tx_log, "%u %u %u\n",++stt, adc_raw_pS, adc_filtered_pS);
			
			//sprintf(uart_tx_log, "%u %u %u %u\n",++stt, adc_raw_pS, adc_filtered_pS, interval_zc);

			//sprintf(uart_tx_log, "%u %u %u %u %u\n",++stt, deltaT, LN, delta_pB, delta_pC);

			//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uart_tx_log, strlen(uart_tx_log));
		}
#endif

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
  hadc1.Init.NbrOfConversion = 2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  HAL_GPIO_WritePin(GPIOA, NSS_Pin|MCU_BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_RESET_Pin|LED_PA_Pin|LED_PB_Pin|LED_PC_Pin
                          |LED_STATUS_SLAVE_2_Pin|LED_STATUS_SLAVE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_DEBUG_ON_BOARD_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_ON_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_DEBUG_ON_BOARD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin MCU_BUZZER_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|MCU_BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_RESET_Pin LED_STATUS_SLAVE_2_Pin LED_STATUS_SLAVE_1_Pin */
  GPIO_InitStruct.Pin = RF_RESET_Pin|LED_STATUS_SLAVE_2_Pin|LED_STATUS_SLAVE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SET_MODE_Pin */
  GPIO_InitStruct.Pin = SET_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SET_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GET_IRQ_Pin */
  GPIO_InitStruct.Pin = GET_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GET_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_PA_Pin LED_PB_Pin LED_PC_Pin */
  GPIO_InitStruct.Pin = LED_PA_Pin|LED_PB_Pin|LED_PC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
    sumS += adc_raw[0];
    HAL_Delay(1);
  }

  offset_pS = sumS/1000;

  vInfor_cache.offset_pS = offset_pS;
  if(Update_NEW_Infor() == UPDATE_SUCCESS){
    mode_buzzer = BUZZER_ON;
  }else{
    while(1){
      ON_LED_PA();ON_LED_PB();ON_LED_PC();BUZZER_ON();
      HAL_Delay(100);
      OFF_LED_PA();OFF_LED_PB();OFF_LED_PC();BUZZER_OFF();
      HAL_Delay(100);
    }
  }
}

void OFF_LED_STATUS(void)
{
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_1_GPIO_Port, LED_STATUS_SLAVE_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_2_GPIO_Port, LED_STATUS_SLAVE_2_Pin, GPIO_PIN_RESET);
}

void ON_LED_STATUS_G(void)
{
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_1_GPIO_Port, LED_STATUS_SLAVE_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_2_GPIO_Port, LED_STATUS_SLAVE_2_Pin, GPIO_PIN_RESET);
}                                                            
																														 
void ON_LED_STATUS_R(void)                                   
{                                                            
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_1_GPIO_Port, LED_STATUS_SLAVE_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_STATUS_SLAVE_2_GPIO_Port, LED_STATUS_SLAVE_2_Pin, GPIO_PIN_SET);
}

void Handle_LED(void)
{
  if(!phaseS.phase_detected){
    point_pA = 0; point_pB = 0; point_pC = 0;
    OFF_LED_PA(); OFF_LED_PB(); OFF_LED_PC();
  }

  switch(mode_led)
  {
    case MODE_IDLE:
      break;
    case MODE_OFF:
      OFF_LED_STATUS();
      mode_led = MODE_IDLE;
      break;
    case MODE_STS_G:
      ON_LED_STATUS_G();
      mode_led = MODE_IDLE;
      break;
    case MODE_STS_R:
      ON_LED_STATUS_R();
      mode_led = MODE_IDLE;
      break;
    case MODE_SET_FLASH_G:
      cnt_handle_led = 0;
      ON_LED_STATUS_G();
      mode_led = MODE_FLASH_G;
      break;
    case MODE_FLASH_G:
      if(cnt_handle_led >= 3600){			// 30ms
        cnt_handle_led = 0;
        mode_led = MODE_OFF;      
      }
      break;
    case MODE_SET_FLASH_R:
      cnt_handle_led = 0;
      ON_LED_STATUS_R();
      mode_led = MODE_FLASH_R;
      break;
    case MODE_FLASH_R:
      if(cnt_handle_led >= 3600){			// 30ms
        cnt_handle_led = 0;
        mode_led = MODE_OFF;      
      }
      break;
    default:
      break;     
  }
}

void Handle_Buzzer(void)
{
  switch(mode_buzzer)
  {
    case BUZZER_OFF:
      BUZZER_OFF();
      break;
    case BUZZER_ON:
			BUZZER_ON();
      break;
    case BUZZER_SET_TRIGGER_PIP:
      // Initialize pip mode
      cnt_handle_buzzer = 0;
			BUZZER_ON();
      mode_buzzer = BUZZER_TRIGGER_PIP;
      break;
    case BUZZER_TRIGGER_PIP:
      if(cnt_handle_buzzer >= 2000)					// 2000*0.00005
      {
        mode_buzzer = BUZZER_OFF;
      }
      break;
    default:
      break;
  }
}

void Handle_LoRa_RX(void)
{
	if(!flag_Lora_Rx)	return;	
	flag_Lora_Rx = false;
	
	rec_ok = LoRa_receive(&vLoRa, RX_LoRa_buff, 5);

	uint8_t checksum = RX_LoRa_buff[0] + RX_LoRa_buff[1] + RX_LoRa_buff[2] + RX_LoRa_buff[3];

	if(RX_LoRa_buff[0] == 0xAA && checksum == RX_LoRa_buff[4] && slave_status != BAT_LOW)
	{
			mode_led = MODE_SET_FLASH_G;
			rx_time = GetTimeUs();
			if(phaseS.phase_detected)
			{
				deltaT_mod = (rx_time - phaseS.last_zc)%20000;
				deltaT_div = (rx_time - phaseS.last_zc)/20000;

#ifdef DEBUG
				latency = rx_time - tx_time;
#endif
			
				delta_pA = RX_LoRa_buff[1]*100;
				delta_pB = RX_LoRa_buff[2]*100;
				delta_pC = RX_LoRa_buff[3]*100;

				if(deltaT_mod <= 6667 || deltaT_mod > 19000){
					point_pA++;
				}else if(deltaT_mod <= 13333){
					point_pC++;
				}else{
					point_pB++;
				}

				uint8_t phase_check = 0;
				if(point_pA > point_pB && point_pA > point_pC){
					phase_check = 1;
				}else if(point_pB > point_pA && point_pB > point_pC){
					phase_check = 2;
				}else if(point_pC > point_pA && point_pC > point_pB){
					phase_check = 3;
				}else{
					phase_check = 0;
				}

				if(phase_check == 1){
					ON_LED_PA();
					OFF_LED_PB();
					OFF_LED_PC();
					mode_buzzer = BUZZER_SET_TRIGGER_PIP;
				}else if(phase_check == 2){
					if(delta_pB > delta_pC){
						OFF_LED_PA();
						ON_LED_PB();
						OFF_LED_PC();
					}else{
						OFF_LED_PA();
						OFF_LED_PB();
						ON_LED_PC();
					}
					mode_buzzer = BUZZER_SET_TRIGGER_PIP;
				}else if(phase_check == 3){
					if(delta_pB > delta_pC){
						OFF_LED_PA();
						OFF_LED_PB();
						ON_LED_PC();
					}else{
						OFF_LED_PA();
						ON_LED_PB();
						OFF_LED_PC();
					}
					mode_buzzer = BUZZER_SET_TRIGGER_PIP;
				}else{
					OFF_LED_PA();
					OFF_LED_PB();
					OFF_LED_PC();
				}
			}
	}
	memset(RX_LoRa_buff, 0, MAX_SIZE_LORA);     
}

void Battery_Handle(Battery_Data_t *pBat)
{
  pBat->now_time = HAL_GetTick();
  pBat->interval_time = pBat->now_time - pBat->last_time;
  if(pBat->interval_time < 500) return;
  pBat->last_time = pBat->now_time;

  pBat->vBat_raw = ((float)*(pBat->adc_raw_ptr)/4095.0f)*3.3f*4.9f;   // tinh dien ap theo "*(pBat->adc_raw_ptr)"

  pBat->adc_filterd = MA_Update(&pBat->filter, *(pBat->adc_raw_ptr));

  pBat->vBat_filtered = (float)(pBat->adc_filterd/4095.0f)*3.3f*4.9f;     // tinh dien ap theo "pBat->adc_filterd"

  if(pBat->vBat_filtered < BATTERY_LOW)  
  {
    if(pBat->cnt_low_bat < 60){		// low bat in 30s
			pBat->cnt_low_bat++;
    }else{
			slave_status = BAT_LOW;
      mode_led = MODE_STS_R;
		}
  }else{
    pBat->cnt_low_bat = 0;
  }

  // Enable track charge process to full battery
  if(pBat->flag_track_charge_full == false && pBat->vBat_filtered < BATTERY_FULL-0.2f && pBat->vBat_filtered > 3.0f)
  {
    pBat->flag_track_charge_full = true;
  }

  if(pBat->vBat_filtered > BATTERY_FULL && pBat->flag_track_charge_full){
    if(pBat->cnt_full_bat < 120){
			pBat->cnt_full_bat++;
    }else{
			slave_status = NORMAL;
      mode_led = MODE_STS_G;
      pBat->flag_track_charge_full = false;
		}
  }else{
    pBat->cnt_full_bat = 0;
  }
}


void MA_Init(MA_Filter_t *f, uint16_t init_value, uint16_t sizeMA)
{
    f->sum = 0;
    f->index = 0;
    f->size = sizeMA;

    for(int i=0; i<f->size; i++)
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

    if(f->index >= f->size)
        f->index = 0;

    return f->sum / f->size;
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
