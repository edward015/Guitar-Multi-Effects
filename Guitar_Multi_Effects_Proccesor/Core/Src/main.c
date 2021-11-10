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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar (void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE * f)
#endif /* __GNUC__ */

void Test_Validation_Menu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })


int button_state = 0;
uint32_t* inBufPtr;
uint32_t* outBufPtr;
uint32_t* delayPtr;
uint32_t* delayPtrWrite;
uint32_t* delayPtrRead;
#define DataSize 512
#define BufferSize DataSize*2
#define PI 3.141592653589793238
volatile int ADCbuff_Half = 0;
volatile int ADCbuff_Full = 0;
volatile int DACbuff_Half = 0;
volatile int DACbuff_Full = 0;
volatile int DelayState = 0;

uint32_t adc_buff[BufferSize] = {0};
uint32_t dac_buff[BufferSize] = {0};
uint32_t delay_buff[BufferSize*3] = {0};

char name[5] = "START";

void effectfunc(int e1,int e2, int e3, int e4){

	// First Half Buffer
		if(ADCbuff_Half && DACbuff_Half){

			inBufPtr = &adc_buff[0];
			outBufPtr = &dac_buff[DataSize];

			if(DelayState == 0){
				delayPtrRead = &delay_buff[0];
				delayPtrWrite= &delay_buff[2*BufferSize];
			}
			else if (DelayState == 1){
				delayPtrRead = &delay_buff[BufferSize];
				delayPtrWrite= &delay_buff[0];
			}
			else if(DelayState == 2){
				delayPtrRead = &delay_buff[(2*BufferSize)];
				delayPtrWrite= &delay_buff[BufferSize];
			}

			ADCbuff_Half = 0;
			DACbuff_Half = 0;
		}
		// Second Half Buffer
		else if(ADCbuff_Full && DACbuff_Full){
			inBufPtr = &adc_buff[DataSize];
			outBufPtr = &dac_buff[0];

			if(DelayState == 0){
				delayPtrRead = &delay_buff[DataSize];
				delayPtrWrite= &delay_buff[2*BufferSize+DataSize];
			}
			else if (DelayState == 1){
				delayPtrRead = &delay_buff[BufferSize+DataSize];
				delayPtrWrite= &delay_buff[DataSize];
			}
			else if (DelayState == 2){
				delayPtrRead = &delay_buff[(2*BufferSize+DataSize)];
				delayPtrWrite= &delay_buff[BufferSize+DataSize];
			}


			ADCbuff_Full = 0;
			DACbuff_Full = 0;
		}

		for(int i = 0; i < DataSize; i++){
			// Clean
			outBufPtr[i] = inBufPtr[i]*1.05;

			// Distortion
			if(e4){
				//outBufPtr[i] = 1;
				//for(int idx = 0; idx < 20; idx++){
				//	outBufPtr[i] += ((pow((-sgn_func(outBufPtr[i])*0.1*outBufPtr[i]), idx))/factorial_func(idx));
				//}
				outBufPtr[i] = 0.1*(sgn_func(outBufPtr[i]) * (1 - exp(sgn_func(0.1*outBufPtr[i])*outBufPtr[i])));
			}

			// Tremolo
			if(e1){
				outBufPtr[i] = max(0,sin(2*PI*i/1024))*outBufPtr[i];
			}


			if( !e3 && ((e2 && e1)||(e2 && e4) || (e2)) ){
				delayPtrWrite[i] = outBufPtr[i];
			}

			// Delay
			if(e2 || e3){
				outBufPtr[i] = 0.75*outBufPtr[i] + 0.5*delayPtrRead[i];
			}

			if ((e2 && e3) || e3){
				delayPtrWrite[i] = outBufPtr[i];
			}


		}


}
void tremolo(){
	// First Half
	if(ADCbuff_Half && DACbuff_Half){
		//pointer math
		inBufPtr = &adc_buff[0];
		outBufPtr = &dac_buff[DataSize];
		for(int i = 0; i < DataSize; i++){
			outBufPtr[i] = max(0,sin(2*PI*i/512))*inBufPtr[i];
		}
		ADCbuff_Half = 0;
		DACbuff_Half = 0;
	}
	// Second Half
	else if(ADCbuff_Full && DACbuff_Full){
		// pointer math
		inBufPtr = &adc_buff[DataSize];
		outBufPtr = &dac_buff[0];

		for(int i = 0; i < DataSize; i++){
		outBufPtr[i] = max(0,sin(2*PI*i/512))*inBufPtr[i];
		}
		ADCbuff_Full = 0;
		DACbuff_Full = 0;
	}


}

void clean(){
	// First Half
	if(ADCbuff_Half && DACbuff_Half){
		//pointer math
		inBufPtr = &adc_buff[0];
		outBufPtr = &dac_buff[DataSize];
		for(int i = 0; i < DataSize; i++){
			outBufPtr[i] = inBufPtr[i];
		}
		ADCbuff_Half = 0;
		DACbuff_Half = 0;
	}
	// Second Half
	else if(ADCbuff_Full && DACbuff_Full){
		// pointer math
		inBufPtr = &adc_buff[DataSize];
		outBufPtr = &dac_buff[0];

		for(int i = 0; i < DataSize; i++){
		outBufPtr[i] = inBufPtr[i];
		}
		ADCbuff_Full = 0;
		DACbuff_Full = 0;

	}
}
void distortion_exp(){
	int G = 0.7;
	double low_gain = 1.05;
	double hi_gain = 1.35;
	double hithresh = 2200;
	double lowthresh  = 1850;



	if(ADCbuff_Half && DACbuff_Half){
			//pointer math
			inBufPtr = &adc_buff[0];
			outBufPtr = &dac_buff[DataSize];
			for(int i = 0; i < DataSize; i++)
				{
					if(adc_buff[i] < lowthresh)
						outBufPtr[i] = hi_gain*inBufPtr[i];
					else if(adc_buff[i] > hithresh)
						outBufPtr[i] = inBufPtr[i];
					else
						outBufPtr[i] = low_gain*inBufPtr[i];

				}
			ADCbuff_Half = 0;
			DACbuff_Half = 0;
		}
		// Second Half
		else if(ADCbuff_Full && DACbuff_Full){
			// pointer math
			inBufPtr = &adc_buff[DataSize];
			outBufPtr = &dac_buff[0];

			for(int i = 0; i < DataSize; i++)
				{
					if(adc_buff[i] < lowthresh)
						outBufPtr[i] = hi_gain*inBufPtr[i]*0.5;
					else if(adc_buff[i] > hithresh)
						outBufPtr[i] = inBufPtr[i];
					else
						outBufPtr[i] = low_gain*inBufPtr[i];

				}
			ADCbuff_Full = 0;
			DACbuff_Full = 0;

		}

}
void delay(){

	// First Half Buffer
	if(ADCbuff_Half && DACbuff_Half){

		inBufPtr = &adc_buff[0];
		outBufPtr = &dac_buff[DataSize];

		if(DelayState == 0){
			delayPtrRead = &delay_buff[0];
			delayPtrWrite= &delay_buff[2*BufferSize];
		}
		else if (DelayState == 1){
			delayPtrRead = &delay_buff[BufferSize];
			delayPtrWrite= &delay_buff[0];
		}
		else if(DelayState == 2){
			delayPtrRead = &delay_buff[(2*BufferSize)];
			delayPtrWrite= &delay_buff[BufferSize];
		}


		for(int i = 0; i < DataSize; i++){
			delayPtrWrite[i] = outBufPtr[i];
			outBufPtr[i] = 0.75*inBufPtr[i] + 0.5*delayPtrRead[i];//+ 0.6*delayPtr[i]+ 0.45*delayPtrHalf[i];
		}


		ADCbuff_Half = 0;
		DACbuff_Half = 0;
	}
	// Second Half Buffer
	else if(ADCbuff_Full && DACbuff_Full){
		inBufPtr = &adc_buff[DataSize];
		outBufPtr = &dac_buff[0];

		if(DelayState == 0){
			delayPtrRead = &delay_buff[DataSize];
			delayPtrWrite= &delay_buff[2*BufferSize+DataSize];
		}
		else if (DelayState == 1){
			delayPtrRead = &delay_buff[BufferSize+DataSize];
			delayPtrWrite= &delay_buff[DataSize];
		}
		else if (DelayState == 2){
			delayPtrRead = &delay_buff[(2*BufferSize+DataSize)];
			delayPtrWrite= &delay_buff[BufferSize+DataSize];
		}

		for(int i = 0; i < DataSize; i++){
			delayPtrWrite[i] = outBufPtr[i];
			outBufPtr[i] = 0.75*inBufPtr[i] + 0.5*delayPtrRead[i];
		}


		ADCbuff_Full = 0;
		DACbuff_Full = 0;
	}

}

int sgn_func(uint32_t val) {
    return (2550 < val) - (val < 2550);
}
int factorial_func(int n)
{
    if (n == 0)
        return 1;
    return n * factorial_func(n - 1);
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  fflush(stdout);
  printf("\r\n");
  printf("\r\n*********************************************\r\n");
  printf("\r\n** EE 113 Design: Guitar Effects Processor **\r\n");
  printf("\r\n*********************************************\r\n");

  printf("\r\n");

  printf("\r\n    ** Press s to Start!!! \r\n");
  while (name[0] != 's')
  	  {
        Serial_Scanf(name, 1);
        if (name[0] != 's')
          {
          printf("\r\n !!! Invalid Character, please press s to Start !!! \r\n");
          }
      }
  Menu();

  printf("\r\n");
  printf("\r\n********************************************");
  printf("\r\n** Thank you for using Guitar Effects Processor By Edward and Nick **");
  printf("\r\n********************************************\r\n");
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLQ = 20;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 1;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T1_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : StopButton_Pin */
  GPIO_InitStruct.Pin = StopButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(StopButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */


int Serial_Scanf(char *ptr, int len)
{

  int DataIdx = 0;
  uint8_t thechar;
  thechar= ' ';
  while(thechar!= '\n' && thechar != '\r' && DataIdx<len)
  {
#ifdef __GNUC__
    thechar = __io_getchar();

#else
    thechar = fgetc(NULL);
#endif
  if ( thechar  >= 0xFF)
  {
    printf("\n\r  !!! Please enter a valid ASCII character \n");
    return 0xFF;
  }
  *ptr++ =thechar;
  DataIdx+=1;
  }
  return DataIdx;
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  //HAL_Delay(1);
  return ch;
}



GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;
  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  HAL_Delay(20);

  HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  HAL_Delay(20);

  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  HAL_Delay(20);

  return ch;
}

void Menu(void)
{
  int menu_option = 0;
  uint8_t stop_test = 0;

  char menu_select[5];
  char menu1[] ="1";
  char menu2[] ="2";
  char menu3[] ="3";
  char menu4[] ="4";
  char menu5[] ="5";
  char menu6[] ="6";
  int e1 = 0;
  int e2 = 0;
  int e3 = 0;
  int e4 = 0;

  char menu_init[] ="0";

  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET,ADC_SINGLE_ENDED) != HAL_OK){
      	  Error_Handler();
    }
  HAL_TIM_Base_Start(&htim1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buff, BufferSize) ;
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*) dac_buff, BufferSize, DAC_ALIGN_12B_R);

  printf("\r\n\r\n   Menu Options:");
  printf("\r\n      1: Tremolo");
  printf("   2: Delay");
  printf("   3: Multi-Delay");
  printf("   4: Distortion");
  printf("   5: Play");
  printf("   6: Exit \r\n");

  do
  {
    strcpy(menu_select, menu_init);
    int menu_option=0;


    menu_option = Serial_Scanf(menu_select,1);

    printf("\r\n\r\n   Menu Options:");
    printf("\r\n      1: Tremolo");
    printf("   2: Single-Delay");
    printf("   3: Multi-Delay");
    printf("   4: Distortion");
    printf("   5: Play");
    printf("   6: Exit \r\n");
/* CASE '1' selected */
    if(strncmp(menu_select, menu1, menu_option) == 0)
    {
        printf("\r\n          ** Tremolo Toggled **\r\n");
        e1 ^= 1;

    }

/* CASE '2' selected */
    else if(strncmp(menu_select, menu2, menu_option) == 0)
    {
        printf("\r\n          ** Single-Delay Toggled **\r\n");
        e2 ^= 1;


    }

/* CASE '3' selected */
    else if(strncmp(menu_select, menu3, menu_option) == 0)
    {
    	printf("\r\n          ** Multi-Delay Toggled **\r\n");
    	e3 ^= 1;

    }
/* CASE '4' selected */
    else if(strncmp(menu_select, menu4, menu_option) == 0)
    {
    	printf("\r\n          ** Distortion Toggled **\r\n");
    	e4 ^= 1;

    }
/* CASE '5' selected */
    else if(strncmp(menu_select, menu5, menu_option) == 0)
    {


    }

/* CASE 'Exit' selected */
    else if(strncmp(menu_select, menu6, menu_option) == 0)
    {
        printf("\n\r          >> Effect processor shutdown. <<\n\r");
        stop_test=1;
    }

/* CASE wrong selection */
    else
    {
        printf("\r\n                !! INVALID CHOICE\r\n");
    }

    printf("\r\n");
    printf("\r\nSettings:      Tremolo: %d",e1);
    printf("   Single-Delay: %d",e2);
    printf("   Multi-Delay: %d",e3);
    printf("   Distortion: %d\r\n",e4);

if (button_state){
	printf("\r\n Playing: Click Button to Change Selections\r\n");
	while(button_state){
		effectfunc(e1,e2,e3,e4);
	}
	printf("\r\n\r\n   Menu Options:");
	printf("\r\n      1: Tremolo");
	printf("   2: Single-Delay");
	printf("   3: Multi-Delay");
	printf("   4: Distortion");
	printf("   5: Play");
	printf("   6: Exit \r\n");
}
else if (!button_state){
	printf("\r\n Click Button and then Select 5 to Start Playback\r\n");
}



  } while (stop_test != 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == StopButton_Pin){ //check interrupt for specific pin
		button_state ^= 1;
		memset(adc_buff,0,sizeof(adc_buff));
		memset(dac_buff,0,sizeof(dac_buff));
	}
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	DACbuff_Full = 1;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	DACbuff_Half = 1;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADCbuff_Full = 1;
	DelayState = (DelayState + 1) % 3;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	ADCbuff_Half = 1;

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
	 HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
