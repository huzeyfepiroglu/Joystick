/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "sdadc.h"
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
#define ASELSAN		 0
#define ASELSANKONYA 1

#define XAXIS	1
#define YAXIS	0
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
/*ADC Variables & Functions*/ //finger ADC
void averageAnalogInputs(uint32_t* ADC_BUFFERPtr, uint32_t* ADC_READPtr, uint32_t COUNT, uint32_t AVERAGE_WINDOW);	//OK

//DACO formatta olmayacak
//uint32_t FinADC_Values[ANALOG_COUNT];
//uint32_t FinADC_Read[ANALOG_COUNT];
bool sampleAnalogInputs = false;
GPIO_PinState errX;
GPIO_PinState errY;

int16_t xAxisData;
int16_t yAxisData;

/*AN_ADC Variables & Function Prototype*/

uint32_t AnADC_Read[ANALOG_COUNT];
uint32_t AnADC_Values[ANALOG_COUNT];

int32_t fittedAnAdc_Values[ANALOG_COUNT];

void SDADC_ScanConversion(void);
uint32_t SDADCErrorCounter;

/* Digital Input variables and Functions */
void initDigitalInputs(digitalInput *digitalInputPtr); 		//OK
void sampleDigitalInputs(digitalInput *digitalInputPtr); 	//OK
void readDigitalInputs(digitalInput *digitalInputPtr);		//OK
digitalInput* getDigitalInputs(void);						//OK
digitalInput tutamakDigitalInputs[BUTTON_COUNT];			//OK

/*rs485 variables & Function prototype*/
void sendUartFrame(UART_HandleTypeDef *huart,uint8_t *uartFrame, uint16_t Size, uint32_t Timeout, bool *sendFlag);		//OK
uint8_t rs422Frame[10];
uint8_t rsSendFormat[9];
uint8_t canSendFormat[8];
uint8_t rs422CheckSum = 0;
bool sendUartFlag = false;

//can
/* Alınan son mesajı tutmak için basit buffer (debug amaçlı) */
typedef struct {
  uint32_t StdId;
  uint8_t  DLC;
  uint8_t  Data[8];
} CAN_LastRx_t;

CAN_LastRx_t g_lastRx;

//volatile uint8_t can_rx_flag = 0;
//CAN_RxHeaderTypeDef rxh;
//uint8_t rxdata[8];
//receive
void sendAckUart(void);					//OK
void sendAckCan(void);
void checkCommand(uint8_t* rxBuffer); 	//OK
//UART variables
uint8_t tempRxDataIn;
uint8_t rxDataIn;
uint32_t rxBufferDataCounter = 0;
uint8_t rxBuffer[9];
uint8_t rxDataCheksum;
uint32_t errorCounter;


/* FLASH Programming */
void eraseFlashUserConf(void); 							//OK
void refreshFlashUserConf(ConfigData *configDataPtr); 	//OK
void readFlashUserConf(ConfigData *configDataPtr);		//OK
uint32_t PAGEError = 0;

void loadDefaultValues(ConfigData *configDataPtr);
//default config flash programming
/*default page write protect seklinde ayarlanacak*/

void writeDefaultConfigPage(void);	//will be called once per board

ConfigData tempTkkConfig;
ConfigData userTkkConfig;

volatile ConfigData defaultTkkConfig;

//Joystick Border Calculation
joystickBorder tkkJoystickBorder;

void calculateJostickBorders(ConfigData* tempConfigData, joystickBorder* tempJoystickBorder);
//axisData = 1: xAxis, axisData = 0: yAxis
void calculateAxisData(joystickBorder* tempJoystickBorder, uint32_t* tempAnADC_Value, int32_t* fittedAnAdc_Values, bool axisData);
//calibration and and remote mood selection// 0 : remote_mode, 1: calibration mode
uint8_t remoteMode = 1;

bool tutamakVersion = ASELSANKONYA;
/* Private variables ---------------------------------------------------------*/
/*
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

SDADC_HandleTypeDef hsdadc1;
//DMA_HandleTypeDef hdma_sdadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
*/

//debug variables
uint32_t counter = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
//static void MX_GPIO_Init(void);
//static void MX_DMA_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_CAN_Init(void);
//static void MX_SDADC1_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_USART1_UART_Init(void);

void systemInit(void);
void mainLoop(void);
void configurationSettings(void);

void bootloaderInit(void);
void (*SysMemBootJump)(void);

void bootloaderCommand(void);
void checkBootloader(void);

uint16_t bootValue = 0;
uint8_t flashReadValue = 0;
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
  MX_CAN_Init();
  MX_SDADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(1000);
	checkBootloader();

/*Baslangic için default config data atamasi*/
	writeDefaultConfigPage();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_DMA(&huart1, &rxDataIn, 1);

	CAN_ConfigFilter();       // Tüm ID'leri kabul et (mask filtre)
	CAN_StartIT();            // CAN'i başlat + RX interrupt aç

//	 CAN_Filter_All_To_FIFO0();                         // her şeyi kabul et
//	  HAL_CAN_Start(&hcan);                              // CAN’ı başlat
//	  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // FIFO0 RX kesmesi


	//fingerAnalogread start
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1);
	HAL_SDADC_PollForCalibEvent(&hsdadc1, 1000);
	configurationSettings();
	initDigitalInputs(getDigitalInputs());


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	    if (can_rx_flag)
//	    {
//	      can_rx_flag = 0;
//	      //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);    // bir şey alındı -> LED değiştir
//	      // rxh.StdId, rxh.DLC, rxdata[] burada kullanılabilir
//	    }
	  mainLoop();
//	  CAN_SendExample();
//	  HAL_Delay(1000);

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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV3;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC1
                              |RCC_PERIPHCLK_SDADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV4;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
}

/* USER CODE BEGIN 4 */
void configurationSettings(void)
{
	//check for DEFAULT/USER Configuration
	flashReadValue = (*(uint32_t*)CONFIG_DATA_DEFAULT_SELECT);
	flashReadValue = 0xff;
	if(flashReadValue == 0xFF)
	{
		loadDefaultValues(&userTkkConfig);
	}
	else
	{
		readFlashUserConf(&userTkkConfig);
	}

	calculateJostickBorders(&userTkkConfig, &tkkJoystickBorder);
}
void mainLoop(void)
{
	if(sampleAnalogInputs == true)
		{

			//averageAnalogInputs(&AnADC_Values[0], &AnADC_Read[0], ANALOG_COUNT, AVERAGE_WINDOW);
			averageAnalogInputs(AnADC_Values, AnADC_Read, ANALOG_COUNT, AVERAGE_WINDOW);
			calculateAxisData(&tkkJoystickBorder, &AnADC_Values[0], &fittedAnAdc_Values[0], XAXIS);
			calculateAxisData(&tkkJoystickBorder, &AnADC_Values[1], &fittedAnAdc_Values[1], YAXIS);
			sampleAnalogInputs = false;
		}

		sampleDigitalInputs(getDigitalInputs());
		SDADC_ScanConversion();
}

void checkBootloader(void)
{
	bootValue = *(uint32_t*)CONFIG_BOOTMODE_INIT_OFFSET;
	if(bootValue == 0x00CD)
	{
			HAL_FLASH_Unlock();
			static FLASH_EraseInitTypeDef eraseInit;
			eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
			eraseInit.PageAddress = CONFIG_BOOTMODE_BASE_ADDR; // düzenlenecek !!!!!!!!!!!!!!!!!!!!*/
			eraseInit.NbPages     = 1;

			if (HAL_FLASHEx_Erase(&eraseInit, &PAGEError) != HAL_OK)
			{
			}
			HAL_Delay(100);
			HAL_FLASH_Lock();
		bootloaderInit();
	}
}

void bootloaderCommand(void)
{
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct_BOOT;
	EraseInitStruct_BOOT.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct_BOOT.PageAddress = CONFIG_BOOTMODE_BASE_ADDR; // düzenlenecek !!!!!!!!!!!!!!!!!!!!*/
	EraseInitStruct_BOOT.NbPages     = 1;
	if (HAL_FLASHEx_Erase(&EraseInitStruct_BOOT, &PAGEError) != HAL_OK)
	{

	}

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_BOOTMODE_INIT_OFFSET, 0x00CD);
	HAL_FLASH_Lock();
	bootValue = *(uint32_t*)CONFIG_BOOTMODE_INIT_OFFSET;

	if(bootValue == 0x00CD)
	{
		if(userTkkConfig.tkkModSelection == TKK_MOD_RS422)
		{
			sendAckUart();
		}

		else if(userTkkConfig.tkkModSelection == TKK_MOD_CAN)
		{
			sendAckCan();
		}
		HAL_NVIC_SystemReset();
	}
}

void bootloaderInit(void)
{
	SysMemBootJump = (void(*)(void)) (*((uint32_t*) (0x1FFFD800 + 4)));

	HAL_RCC_DeInit();
	SysTick -> CTRL = 0;
	SysTick -> LOAD = 0;
	SysTick -> VAL  = 0;
	__set_PRIMASK(1);

	__set_MSP(DEFAULT_MSP_VALUE);
	SysMemBootJump();
	while(1);
}
void calculateAxisData(joystickBorder* tempJoystickBorder, uint32_t* tempAnADC_Value, int32_t* fittedAnAdc_Values, bool axisData)
{
	joystickBorder* joystickBorderPtr;
	joystickBorderPtr = tempJoystickBorder;
	errX = GPIO_PIN_RESET;
	errY = GPIO_PIN_RESET;
	//calculate x-Axis
	if(axisData == XAXIS)
	{
		if(*tempAnADC_Value <= joystickBorderPtr->xLeftLow)
		{
			//*fittedAnAdc_Values = -32766;
			//*fittedAnAdc_Values = -2048;  #huzeyfe
			*fittedAnAdc_Values = 0;
			errX = GPIO_PIN_SET;
		}
		else if(*tempAnADC_Value >= joystickBorderPtr->xRightHigh)
		{
			//*fittedAnAdc_Values = 32766;
			errX = GPIO_PIN_SET;
			//*fittedAnAdc_Values = 2047;  #huzeyfe
			*fittedAnAdc_Values = 65535;
		}

		else if(*tempAnADC_Value >= joystickBorderPtr -> xRightLow)
		{
//			*fittedAnAdc_Values = (((int)((*tempAnADC_Value) - (joystickBorderPtr->xRightLow))) * 2047) / (joystickBorderPtr -> xRightRange); #huzeyfe
//			if(*fittedAnAdc_Values >= 2047)
//			{
//				*fittedAnAdc_Values = 2047;
//			}

			*fittedAnAdc_Values = (((int)((*tempAnADC_Value) - (joystickBorderPtr->xRightLow))) * 65535) / (joystickBorderPtr -> xRightRange);
			if(*fittedAnAdc_Values >= 65535)
			{
				*fittedAnAdc_Values = 65535;
			}
		}
		else if(*tempAnADC_Value < (joystickBorderPtr -> xLeftHigh))
		{
//			*fittedAnAdc_Values = -((((int)((int)(joystickBorderPtr->xLeftHigh) - (int)(*tempAnADC_Value))) * (int)2047) / (joystickBorderPtr -> xLeftRange)); #huzeyfe
//			if(*fittedAnAdc_Values <= -2048)
//			{
//				*fittedAnAdc_Values = -2048;
//			}

			*fittedAnAdc_Values = -((((int)((int)(joystickBorderPtr->xLeftHigh) - (int)(*tempAnADC_Value))) * (int)65535) / (joystickBorderPtr -> xLeftRange));
			if(*fittedAnAdc_Values <= 0)
			{
				*fittedAnAdc_Values = 0;
			}
		}
	}

	else
	{
		if(*tempAnADC_Value <= joystickBorderPtr->yDownLow)
		{
			//*fittedAnAdc_Values = 2048;
			errY = GPIO_PIN_SET;
			//*fittedAnAdc_Values = -2048;  #huzeyfe
			*fittedAnAdc_Values = 0;
		}
		else if(*tempAnADC_Value >= joystickBorderPtr->yUpHigh)
		{
			//*fittedAnAdc_Values = -2048;
			//*fittedAnAdc_Values = 2047;
			*fittedAnAdc_Values = 65535;
			errY = GPIO_PIN_SET;
		}
		else if(*tempAnADC_Value >= joystickBorderPtr -> yUpLow)
		{
//			*fittedAnAdc_Values = (((int)((*tempAnADC_Value) - (joystickBorderPtr->yUpLow))) * 2047) / (joystickBorderPtr -> yUpRange); #huzeyfe
//			if(*fittedAnAdc_Values >= 2047)
//			{
//				*fittedAnAdc_Values = 2047;
//			}

			*fittedAnAdc_Values = (((int)((*tempAnADC_Value) - (joystickBorderPtr->yUpLow))) * 65535) / (joystickBorderPtr -> yUpRange);
			if(*fittedAnAdc_Values >= 65535)
			{
				*fittedAnAdc_Values = 65535;
			}
		}
		else if(*tempAnADC_Value < (joystickBorderPtr -> yDownHigh))
		{
//			*fittedAnAdc_Values = -((((int)((int)(joystickBorderPtr->yDownHigh) - (int)(*tempAnADC_Value))) * (int)2047) / (joystickBorderPtr -> yDownRange)); #huzeyfe
//
//			if(*fittedAnAdc_Values <= -2048)
//			{
//				*fittedAnAdc_Values = -2048;
//			}

			*fittedAnAdc_Values = -((((int)((int)(joystickBorderPtr->yDownHigh) - (int)(*tempAnADC_Value))) * (int)65535) / (joystickBorderPtr -> yDownRange));

			if(*fittedAnAdc_Values <= 0)
			{
				*fittedAnAdc_Values = 0;
			}
		}

	}
}
/* calculating joystick borders*/
void calculateJostickBorders(ConfigData* tempConfigData, joystickBorder* tempJoystickBorder)
{
	ConfigData* configDataPtr;
	configDataPtr = tempConfigData;

	joystickBorder* joystickBorderPtr;
	joystickBorderPtr = tempJoystickBorder;

	joystickBorderPtr -> xLeftHigh 		= (configDataPtr -> xMid) - 1;
	joystickBorderPtr -> xLeftLow  		= configDataPtr -> xMin;
	joystickBorderPtr -> xLeftRange 	= (joystickBorderPtr -> xLeftHigh) - (joystickBorderPtr -> xLeftLow);

	joystickBorderPtr -> xRightLow 		= (configDataPtr -> xMid) + 1;
	joystickBorderPtr -> xRightHigh  	= configDataPtr -> xMax;
	joystickBorderPtr -> xRightRange 	= (joystickBorderPtr -> xRightHigh) - (joystickBorderPtr -> xRightLow);

	joystickBorderPtr -> yUpHigh 		= configDataPtr -> yMax;
	joystickBorderPtr -> yUpLow  		= (configDataPtr -> yMid) + 1;
	joystickBorderPtr -> yUpRange 		= (joystickBorderPtr -> yUpHigh) - (joystickBorderPtr -> yUpLow);

	joystickBorderPtr -> yDownLow 		= configDataPtr -> yMin;
	joystickBorderPtr -> yDownHigh  	= (configDataPtr -> yMid) - 1;
	joystickBorderPtr -> yDownRange 	= (joystickBorderPtr -> yDownHigh) - (joystickBorderPtr -> yDownLow);

}

/*ADC CONV Complete CallBack*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(sampleAnalogInputs == true)
	{
		//averageAnalogInputs(&FinADC_Values[0], &FinADC_Read[0], ANALOG_COUNT, AVERAGE_WINDOW);
		//averageAnalogInputs(&AnADC_Values[0], &AnADC_Read[0], ANALOG_COUNT, AVERAGE_WINDOW);
		averageAnalogInputs(AnADC_Values, AnADC_Read, ANALOG_COUNT, AVERAGE_WINDOW);
		sampleAnalogInputs = false;
	}
}
/** @brief 				: function to average analog input values
		@Param 				: none
		@description	:

*/
void averageAnalogInputs(uint32_t* ADC_BUFFERPtr, uint32_t* ADC_READPtr, uint32_t COUNT, uint32_t AVERAGE_WINDOW)
{
	uint32_t i = 0;
	uint32_t* tempADC_BUFFERPtr = ADC_BUFFERPtr;
	uint32_t* tempADC_READPtr = ADC_READPtr;

	for(i = 0; i < COUNT; i++)
	{
		*tempADC_BUFFERPtr = ((*tempADC_BUFFERPtr)*(AVERAGE_WINDOW - 1) + (*tempADC_READPtr))/AVERAGE_WINDOW;
		tempADC_BUFFERPtr++;
		tempADC_READPtr++;
	}
}

/** @brief 				: function to get AN_ADCx analog input values
		@Param 				: none
		@description	:

*/
void SDADC_ScanConversion(void)
{


		if(HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_4, SDADC_CONTINUOUS_CONV_ON) != HAL_OK)
		{
			Error_Handler();
		}

		HAL_SDADC_Start(&hsdadc1);
		if(HAL_SDADC_PollForConversion(&hsdadc1, 1000) != HAL_OK)
		{
			 SDADCErrorCounter++;
		}
		else
		{
			AnADC_Read[0] = (HAL_SDADC_GetValue(&hsdadc1) + 0x8000) & 0xFFFF;

		}
		HAL_SDADC_Stop(&hsdadc1);

		if(HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_5, SDADC_CONTINUOUS_CONV_ON) != HAL_OK)
		{
			Error_Handler();
		}

		HAL_SDADC_Start(&hsdadc1);
		if(HAL_SDADC_PollForConversion(&hsdadc1, 1000) != HAL_OK)
		{
			 SDADCErrorCounter++;
		}
		else
		{
			AnADC_Read[1] = (HAL_SDADC_GetValue(&hsdadc1) + 0x8000) & 0xFFFF;
		}
		HAL_SDADC_Stop(&hsdadc1);

}

/** @brief 				: function to initialize digitalInputs
		@Param : digitalInputStruct array
		@description

				Initial Values:
				inputVal = GPIO_PIN_RESET;
				readFlag = false;
				readState = GPIO_PIN_RESET;
*/

void initDigitalInputs(digitalInput *digitalInputPtr)
{
	/****************************************************
	***	CUSTOM VALUES SHOULD BE INITIALIZED MANUALLY	***
	****************************************************/
	tutamakDigitalInputs[0].inputPin 	= BTN_01_Pin;
	tutamakDigitalInputs[0].inputPort	= BTN_01_GPIO_Port;
	tutamakDigitalInputs[1].inputPin 	= BTN_02_Pin;
	tutamakDigitalInputs[1].inputPort	= BTN_02_GPIO_Port;
	tutamakDigitalInputs[2].inputPin 	= BTN_03_Pin;
	tutamakDigitalInputs[2].inputPort	= BTN_03_GPIO_Port;
	tutamakDigitalInputs[3].inputPin 	= BTN_04_Pin;
	tutamakDigitalInputs[3].inputPort	= BTN_04_GPIO_Port;
	tutamakDigitalInputs[4].inputPin 	= BTN_05_Pin;
	tutamakDigitalInputs[4].inputPort	= BTN_05_GPIO_Port;
	tutamakDigitalInputs[5].inputPin 	= BTN_06_Pin;
	tutamakDigitalInputs[5].inputPort	= BTN_06_GPIO_Port;
	tutamakDigitalInputs[6].inputPin 	= BTN_07_Pin;
	tutamakDigitalInputs[6].inputPort	= BTN_07_GPIO_Port;
	tutamakDigitalInputs[7].inputPin 	= BTN_08_Pin;
	tutamakDigitalInputs[7].inputPort	= BTN_08_GPIO_Port;
	tutamakDigitalInputs[8].inputPin 	= BTN_09_Pin;
	tutamakDigitalInputs[8].inputPort	= BTN_09_GPIO_Port;
	tutamakDigitalInputs[9].inputPin 	= BTN_10_Pin;
	tutamakDigitalInputs[9].inputPort	= BTN_10_GPIO_Port;
	tutamakDigitalInputs[10].inputPin 	= BTN_11_Pin;
	tutamakDigitalInputs[10].inputPort	= BTN_11_GPIO_Port;
	tutamakDigitalInputs[11].inputPin 	= BTN_12_Pin;
	tutamakDigitalInputs[11].inputPort	= BTN_12_GPIO_Port;
	tutamakDigitalInputs[12].inputPin 	= BTN_13_Pin;
	tutamakDigitalInputs[12].inputPort	= BTN_13_GPIO_Port;
	tutamakDigitalInputs[13].inputPin 	= BTN_14_Pin;
	tutamakDigitalInputs[13].inputPort	= BTN_14_GPIO_Port;
	tutamakDigitalInputs[14].inputPin 	= BTN_15_Pin;
	tutamakDigitalInputs[14].inputPort	= BTN_15_GPIO_Port;
	tutamakDigitalInputs[15].inputPin 	= BTN_16_Pin;
	tutamakDigitalInputs[15].inputPort	= BTN_16_GPIO_Port;
	tutamakDigitalInputs[16].inputPin 	= BTN_17_Pin;
	tutamakDigitalInputs[16].inputPort	= BTN_17_GPIO_Port;
	tutamakDigitalInputs[17].inputPin 	= BTN_18_Pin;
	tutamakDigitalInputs[17].inputPort	= BTN_18_GPIO_Port;
	tutamakDigitalInputs[18].inputPin 	= BTN_19_Pin;
	tutamakDigitalInputs[18].inputPort	= BTN_19_GPIO_Port;
	tutamakDigitalInputs[19].inputPin 	= BTN_20_Pin;
	tutamakDigitalInputs[19].inputPort	= BTN_20_GPIO_Port;
	tutamakDigitalInputs[20].inputPin 	= BTN_21_Pin;
	tutamakDigitalInputs[20].inputPort	= BTN_21_GPIO_Port;
	tutamakDigitalInputs[21].inputPin 	= BTN_22_Pin;
	tutamakDigitalInputs[21].inputPort	= BTN_22_GPIO_Port;
	tutamakDigitalInputs[22].inputPin 	= BTN_23_Pin;
	tutamakDigitalInputs[22].inputPort	= BTN_23_GPIO_Port;
	tutamakDigitalInputs[23].inputPin 	= BTN_24_Pin;
	tutamakDigitalInputs[23].inputPort	= BTN_24_GPIO_Port;
	tutamakDigitalInputs[24].inputPin 	= BTN_25_Pin;
	tutamakDigitalInputs[24].inputPort	= BTN_25_GPIO_Port;
	tutamakDigitalInputs[25].inputPin 	= BTN_26_Pin;
	tutamakDigitalInputs[25].inputPort	= BTN_26_GPIO_Port;
	tutamakDigitalInputs[26].inputPin 	= BTN_27_Pin;
	tutamakDigitalInputs[26].inputPort	= BTN_27_GPIO_Port;
	tutamakDigitalInputs[27].inputPin 	= BTN_28_Pin;
	tutamakDigitalInputs[27].inputPort	= BTN_28_GPIO_Port;
	tutamakDigitalInputs[28].inputPin 	= BTN_29_Pin;
	tutamakDigitalInputs[28].inputPort	= BTN_29_GPIO_Port;
	tutamakDigitalInputs[29].inputPin 	= BTN_30_Pin;
	tutamakDigitalInputs[29].inputPort	= BTN_30_GPIO_Port;
	tutamakDigitalInputs[30].inputPin 	= BTN_31_Pin;
	tutamakDigitalInputs[30].inputPort	= BTN_31_GPIO_Port;
	tutamakDigitalInputs[31].inputPin 	= BTN_32_Pin;
	tutamakDigitalInputs[31].inputPort	= BTN_32_GPIO_Port;
	/***************************************************/

	uint32_t i;
	digitalInput* tempDigitalInputPtr = digitalInputPtr;

	for(i = 0; i < BUTTON_COUNT; i++)
	{
		tempDigitalInputPtr	->	inputVal = GPIO_PIN_RESET;
		tempDigitalInputPtr	->	readFlag = false;
		tempDigitalInputPtr ->  readState= GPIO_PIN_RESET;
		tempDigitalInputPtr ->  debounceTimer = 0;
		//for prototype debounce factor
		tempDigitalInputPtr ->  debounceFactor = 20; /*!!!!!!!!!!!!! config structtan cekilecek !!!!!!!!!!!!!!!!!*/
		tempDigitalInputPtr++;
	}

}

/** @brief 				: function to sample input value
		@Param 				: digitalInput array
		@description	:

*/

void sampleDigitalInputs(digitalInput *digitalInputPtr)
{
	uint32_t i;
	digitalInput* tempDigitalInputPtr = digitalInputPtr;

	for(i = 0; i < BUTTON_COUNT; i++)
	{
		if(tempDigitalInputPtr -> readFlag == true)
		{
			if(tempDigitalInputPtr -> readState == GPIO_PIN_RESET)
			{
				if(tempDigitalInputPtr -> inputVal == GPIO_PIN_RESET)
				{
					if(tempDigitalInputPtr -> debounceTimer < tempDigitalInputPtr -> debounceFactor)
					{
						tempDigitalInputPtr -> debounceTimer++;
					}
					if(tempDigitalInputPtr ->debounceTimer >= tempDigitalInputPtr ->debounceFactor)
					{
						tempDigitalInputPtr -> debounceTimer = 0;
						tempDigitalInputPtr -> readState = GPIO_PIN_SET;
					}
				}
				else
				{
					tempDigitalInputPtr -> debounceTimer = 0;
				}
			}

			else if(tempDigitalInputPtr -> readState == GPIO_PIN_SET)
			{
				if(tempDigitalInputPtr -> inputVal == GPIO_PIN_SET)
				{
					if(tempDigitalInputPtr -> debounceTimer < tempDigitalInputPtr -> debounceFactor)
					{
						tempDigitalInputPtr -> debounceTimer++;
					}
					if(tempDigitalInputPtr ->debounceTimer >= tempDigitalInputPtr ->debounceFactor)
					{
						tempDigitalInputPtr -> debounceTimer = 0;
						tempDigitalInputPtr -> readState = GPIO_PIN_RESET;
					}
				}
				else
				{
					tempDigitalInputPtr -> debounceTimer = 0;
				}
			}

			tempDigitalInputPtr   ->  readFlag = false;

			tempDigitalInputPtr++;
		}
	}
}

/** @brief 				: function to read digital inputs sequantially
	* @param 	uint8_t DIN_NUMBER number of digital inputs to read
	* @retval None
	*/
void readDigitalInputs(digitalInput *digitalInputPtr)
{
	uint32_t i;
	digitalInput* tempDigitalInputPtr = digitalInputPtr;

	for(i=0;i<BUTTON_COUNT;i++)
	{
		tempDigitalInputPtr		->		inputVal = HAL_GPIO_ReadPin(tempDigitalInputPtr->inputPort, tempDigitalInputPtr->inputPin);
		tempDigitalInputPtr		->		readFlag = true;
		tempDigitalInputPtr++;
	}
}

/** @brief 				: function to frame input value
		@Param 				: digitalInput array ptr
		@description	:

*/

digitalInput* getDigitalInputs(void)
{
		return &tutamakDigitalInputs[0];
}

/** @brief 				: function to sample analog input values
		@Param 				: ----
		@description	: ----

*/
void sendUartFrame(UART_HandleTypeDef *huart,uint8_t* rs422Frame_, uint16_t Size, uint32_t Timeout, bool* sendFlag)
{
	unsigned int i;
	if(*sendFlag == true && userTkkConfig.tkkModSelection == TKK_MOD_RS422)
	{
		rs422Frame_[0] = RS422_HEADER;


		rs422Frame_[1] = 0x00 |
					   ( tutamakDigitalInputs[14].readState << 2) 	|									//SW4-DOWN
					   ( tutamakDigitalInputs[12].readState << 3) 	|									//SW4-UP
					   (((~(tutamakDigitalInputs[0].readState)) << 4) & 0x10);							//SW1

		rs422Frame_[2] = 0x00 |
						(((~(tutamakDigitalInputs[11].readState)) << 0) & 0x01)	| 						//SW7
							(tutamakDigitalInputs[9].readState << 2) 			|						//SW6
							(tutamakDigitalInputs[2].readState << 4);									//SW2

		rs422Frame_[3] = (tutamakDigitalInputs[7].readState) 		|									//SW5-RIGHT
						 (tutamakDigitalInputs[5].readState << 1) 	|									//SW5-LEFT
						 (tutamakDigitalInputs[3].readState << 2) 	|									//SW5-DOWN
						 (tutamakDigitalInputs[1].readState << 3) 	|									//SW5-UP
						 (tutamakDigitalInputs[10].readState << 4)	|									//SW3-RIGHT
						 (tutamakDigitalInputs[8].readState << 5)	|									//SW3-LEFT
						 (tutamakDigitalInputs[6].readState << 6)	|									//SW3-DOWN
						 (tutamakDigitalInputs[4].readState << 7);										//SW3-UP

		if(remoteMode == 0)
		{
			//AnADC_Values[0]=( AnADC_Values[0] >> 3) & 0x1FFF;
			//AnADC_Values[1]=( AnADC_Values[1] >> 3) & 0x1FFF;

			rs422Frame_[4] = (AnADC_Values[0] >> 8) & 0xFF;// & 0x1F;			//AN2_7-0
			rs422Frame_[5] =  AnADC_Values[0]       & 0xFF; 					//AN2_15-8

			rs422Frame_[6] = (AnADC_Values[1] >> 8) & 0xFF;      				//AN3_7-0
			rs422Frame_[7] =  AnADC_Values[1]       & 0xFF; 					//AN3_15-8
		}
		else
		{
			//fitted kullaniliyor

			//xAxisData = (AnADC_Values[0] >> 3) & 0x1FFF;
			//yAxisData = (AnADC_Values[1] >> 3) & 0x1FFF;

			rs422Frame_[4] = (fittedAnAdc_Values[0] >> 7) & 0x1F;
			rs422Frame_[5] =  fittedAnAdc_Values[0]       & 0x7F;

			rs422Frame_[6] = (fittedAnAdc_Values[1] >> 7) & 0x1F;
			rs422Frame_[7] =  fittedAnAdc_Values[1]       & 0x7F;
		}

		rs422Frame_[8] = 0x00;
		//rs422Frame_[8] = 0x00 |
		//								errY << 7 |
		//								errX << 6;//error byte

		/* checksum calculation */
		rs422Frame_[RS422_FRAME_SIZE-1] = 0;
		for(i=1;i<RS422_FRAME_SIZE-1;i++)
		{
			rs422Frame_[RS422_FRAME_SIZE-1] += rs422Frame_[i];
		}
		rs422Frame_[RS422_FRAME_SIZE-1] = (255-rs422Frame_[RS422_FRAME_SIZE-1]) + 1;
		rs422Frame_[RS422_FRAME_SIZE -1] = rs422Frame[RS422_FRAME_SIZE - 1] & 0x7F;

		/* transmit frame */
		  HAL_UART_Transmit(huart, (uint8_t*)rs422Frame_, 10, Timeout);
		*sendFlag = false;
	}
	else if(*sendFlag == true && userTkkConfig.tkkModSelection == TKK_MOD_CAN)
	{

		rs422Frame_[0] = (((!tutamakDigitalInputs[0].readState))		|
						 (   tutamakDigitalInputs[1].readState 	<< 1)	|
						 (   tutamakDigitalInputs[2].readState 	<< 2) 	|
						 (   tutamakDigitalInputs[3].readState 	<< 3) 	|
						 (   tutamakDigitalInputs[4].readState  << 4) 	|
						 (   tutamakDigitalInputs[5].readState 	<< 5) 	|
						 (   tutamakDigitalInputs[6].readState 	<< 6) 	|
						 (   tutamakDigitalInputs[7].readState) << 7);

		rs422Frame_[1] = (((tutamakDigitalInputs[8].readState)			|
						 (  tutamakDigitalInputs[9].readState 	<< 1)	|
						 (  tutamakDigitalInputs[10].readState 	<< 2) 	|
						 ((!tutamakDigitalInputs[11].readState) << 3) 	|
						 (  tutamakDigitalInputs[12].readState  << 4) 	|
						 (  tutamakDigitalInputs[13].readState 	<< 5) 	|
						 (  tutamakDigitalInputs[14].readState 	<< 6) 	|
						 (  tutamakDigitalInputs[15].readState) << 7));

		rs422Frame_[2] =((( tutamakDigitalInputs[16].readState)			|
						(   tutamakDigitalInputs[17].readState 	<< 1)	|
						(   tutamakDigitalInputs[18].readState 	<< 2) 	|
						(   tutamakDigitalInputs[19].readState 	<< 3) 	|
						(   tutamakDigitalInputs[20].readState  << 4) 	|
						(   tutamakDigitalInputs[21].readState 	<< 5) 	|
						(   tutamakDigitalInputs[22].readState 	<< 6) 	|
						(   tutamakDigitalInputs[23].readState) << 7));

		rs422Frame_[3] =((( tutamakDigitalInputs[24].readState)			|
						(   tutamakDigitalInputs[25].readState 	<< 1)	|
						(   tutamakDigitalInputs[26].readState 	<< 2) 	|
						(   tutamakDigitalInputs[27].readState 	<< 3) 	|
						(   tutamakDigitalInputs[28].readState  << 4) 	|
						(   tutamakDigitalInputs[29].readState 	<< 5) 	|
						(   tutamakDigitalInputs[30].readState 	<< 6) 	|
						(   tutamakDigitalInputs[31].readState) << 7));

		if(remoteMode == 0)
		{
			rs422Frame_[4] = (AnADC_Values[0] >> 8) & 0xFF;// & 0x1F;			//AN2_7-0
			rs422Frame_[5] =  AnADC_Values[0]       & 0xFF; 					//AN2_15-8

			rs422Frame_[6] = (AnADC_Values[1] >> 8) & 0xFF;      				//AN3_7-0
			rs422Frame_[7] =  AnADC_Values[1]       & 0xFF; 					//AN3_15-8
		}
		else
		{
			rs422Frame_[4] = (fittedAnAdc_Values[0] >> 7) & 0x1F;
			rs422Frame_[5] =  fittedAnAdc_Values[0]       & 0x7F;

			rs422Frame_[6] = (fittedAnAdc_Values[1] >> 7) & 0x1F;
			rs422Frame_[7] =  fittedAnAdc_Values[1]       & 0x7F;
		}

	//CAN
	CAN_TxHeaderTypeDef txHeader = {0};

	txHeader.StdId = 0x321;
	txHeader.IDE   = CAN_ID_STD;
	txHeader.RTR   = CAN_RTR_DATA;
	txHeader.DLC   = 8;
	txHeader.TransmitGlobalTime = DISABLE;

	uint8_t data[8] = { rs422Frame_[0],
	rs422Frame_[1],
	rs422Frame_[2],
	rs422Frame_[3],
	rs422Frame_[4],
	rs422Frame_[5],
	rs422Frame_[6],
	rs422Frame_[7]};

	uint32_t txMailbox;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK)
	{
	// TX kuyruğu dolu vs. durumunda hata yönetimi
	}
	*sendFlag = false;
	}
}

/** @brief 				: function to erase flash page that contains user configurations
		@Param 			: none
		@description	:
*/

void eraseFlashUserConf(void)
{
	HAL_FLASH_Unlock();

	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = CONFIG_DATA_BASE_ADDR; // düzenlenecek !!!!!!!!!!!!!!!!!!!!*/
	EraseInitStruct.NbPages     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		while (1)
		{

		}
	}

	HAL_FLASH_Lock();
}

/** @brief 				: function to write configuration values to page

//16 bitlik yazmaya bakilacak
		@Param 				: none
		@description	: Page Erase procedure is used before write function

*/
void refreshFlashUserConf(ConfigData *configDataPtr)
{
	volatile uint32_t i = 0;
	ConfigData* tempConfigDataPtr = configDataPtr;
	eraseFlashUserConf();
	HAL_FLASH_Unlock();

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_DEFAULT_SELECT		, 0x00 									);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_INTERFACE_OFFSET		, tempConfigDataPtr -> tkkModSelection 	);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_X_MIDDLEPOINT_OFFSET	, tempConfigDataPtr -> xMid				);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_X_MINPOINT_OFFSET		, tempConfigDataPtr -> xMin				);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_X_MAXPOINT_OFFSET		, tempConfigDataPtr -> xMax				);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_Y_MIDDLEPOINT_OFFSET	, tempConfigDataPtr -> yMid				);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_Y_MINPOINT_OFFSET		, tempConfigDataPtr -> yMin				);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_Y_MAXPOINT_OFFSET		, tempConfigDataPtr -> yMax				);

	for(i = 0; i < 32; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CONFIG_DATA_DEBOUNCEFACTOR_BASE + i*CONFIG_FLASH_ADDR_INCREMENT, tempTkkConfig.debounceFactor[i]);
	}

	HAL_FLASH_Lock();

	readFlashUserConf(&userTkkConfig);
}

/** @brief 				: function to read flash page that contains user configurations
		@Param 				: none
		@description	:

*/
void readFlashUserConf(ConfigData *configDataPtr)
{
	volatile uint32_t i = 0;
	ConfigData* tempConfigDataPtr = configDataPtr;

	tempConfigDataPtr -> tkkModSelection 	= (*(uint32_t*)CONFIG_DATA_INTERFACE_OFFSET		);

	tempConfigDataPtr -> xMid 				= (*(uint32_t*)CONFIG_DATA_X_MIDDLEPOINT_OFFSET	);
	tempConfigDataPtr -> xMin 				= (*(uint32_t*)CONFIG_DATA_X_MINPOINT_OFFSET	);
	tempConfigDataPtr -> xMax 				= (*(uint32_t*)CONFIG_DATA_X_MAXPOINT_OFFSET	);

	tempConfigDataPtr -> yMid 				= (*(uint32_t*)CONFIG_DATA_Y_MIDDLEPOINT_OFFSET	);
	tempConfigDataPtr -> yMin 				= (*(uint32_t*)CONFIG_DATA_Y_MINPOINT_OFFSET	);
	tempConfigDataPtr -> yMax 				= (*(uint32_t*)CONFIG_DATA_Y_MAXPOINT_OFFSET	);

	for(i = 0; i < 32; i++)
	{
		tempConfigDataPtr -> debounceFactor [i] = (*(uint32_t*)(CONFIG_DATA_DEBOUNCEFACTOR_BASE + i*CONFIG_FLASH_ADDR_INCREMENT));
	}
}
void loadDefaultValues(ConfigData *configDataPtr)
{
	volatile uint32_t i = 0;
	ConfigData* tempConfigDataPtr = configDataPtr;

	tempConfigDataPtr -> tkkModSelection 	= (*(uint32_t*)DEFAULT_CONFIG_DATA_INTERFACE_OFFSET		);

	tempConfigDataPtr -> xMid 				= (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET	);
	tempConfigDataPtr -> xMin 				= (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET	);
	tempConfigDataPtr -> xMax 				= (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET	);

	tempConfigDataPtr -> yMid 				= (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET	);
	tempConfigDataPtr -> yMin 				= (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET	);
	tempConfigDataPtr -> yMax 				= (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET	);

	for(i = 0; i < 32; i++)
	{
		tempConfigDataPtr ->debounceFactor [i] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_DEBOUNCEFACTOR_BASE + i*CONFIG_FLASH_ADDR_INCREMENT));
	}
}

void sendAckUart(void)
{
	volatile uint8_t uartACK[9];

	uartACK[0] = COMMAND_HEADER;
	uartACK[1] = COMMAND_ACK;
	uartACK[2] = 0x00;
	uartACK[3] = 0x00;
	uartACK[4] = 0x00;
	uartACK[5] = 0x00;
	uartACK[6] = 0x00;
	uartACK[7] = 0x00;
	uartACK[8] = CHECKSUM_ACK;
	HAL_UART_Transmit(&huart1, (uint8_t*)uartACK, 9, 5000);
}

void sendAckCan(void)
{
	CAN_TxHeaderTypeDef txHeader = {0};
	uint32_t txMailbox;

	txHeader.StdId = 0x321;
	txHeader.IDE   = CAN_ID_STD;
	txHeader.RTR   = CAN_RTR_DATA;
	txHeader.DLC   = 8;
	txHeader.TransmitGlobalTime = DISABLE;

	uint8_t canACK[9];

	canACK[0] = COMMAND_HEADER;
	canACK[1] = COMMAND_ACK;
	canACK[2] = 0x00;
	canACK[3] = 0x00;
	canACK[4] = 0x00;
	canACK[5] = 0x00;
	canACK[6] = 0x00;
	canACK[7] = 0x00;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canACK, &txMailbox) != HAL_OK)
	{
		// TX kuyruğu dolu vs. durumunda hata yönetimi
	}
}

void checkCommand(uint8_t* rxBuffer)
{
	volatile uint32_t i = 0;

	if(userTkkConfig.tkkModSelection == TKK_MOD_RS422)
	{
		switch (rxBuffer[1])
			{
				case COMMAND_MODSEL_WRITE:
					tempTkkConfig.tkkModSelection = rxBuffer[2];
					sendAckUart();
				break;

				case COMMAND_XCALIB_WRITE:
					tempTkkConfig.xMax = rxBuffer[2]<<8 | rxBuffer[3];
					tempTkkConfig.xMin = rxBuffer[4]<<8 | rxBuffer[5];
					tempTkkConfig.xMid = rxBuffer[6]<<8 | rxBuffer[7];
					sendAckUart();
				break;

				case COMMAND_YCALIB_WRITE:
					tempTkkConfig.yMax = rxBuffer[2]<<8 | rxBuffer[3];
					tempTkkConfig.yMin = rxBuffer[4]<<8 | rxBuffer[5];
					tempTkkConfig.yMid = rxBuffer[6]<<8 | rxBuffer[7];
					sendAckUart();
				break;

				case COMMAND_DEBOUNCE_WRITE://debounce tarafi düzenlenecek
				break;

				case COMMAND_MODSEL_READ:
					rsSendFormat[0] = COMMAND_HEADER;
					rsSendFormat[1] = COMMAND_MODSEL_READ;
					rsSendFormat[2] = (*(uint32_t*)CONFIG_DATA_INTERFACE_OFFSET);;
					rsSendFormat[3] = 0x00;
					rsSendFormat[4] = 0x00;
					rsSendFormat[5] = 0x00;
					rsSendFormat[6] = 0x00;
					rsSendFormat[7] = 0x00;
					for(i = 1; i < 8; i++)
					{
						rsSendFormat[8] += rsSendFormat[i];
					}
					HAL_UART_Transmit(&huart1, (uint8_t*)rsSendFormat, 9, 5000);
				break;

				case COMMAND_XCALIB_READ:
					rsSendFormat[0] = COMMAND_HEADER;
					rsSendFormat[1] = COMMAND_XCALIB_READ;
					rsSendFormat[2] = (*(uint32_t*)(CONFIG_DATA_X_MAXPOINT_OFFSET + 1));
					rsSendFormat[3] = (*(uint32_t*)CONFIG_DATA_X_MAXPOINT_OFFSET);
					rsSendFormat[4] = (*(uint32_t*)(CONFIG_DATA_X_MINPOINT_OFFSET + 1));
					rsSendFormat[5] = (*(uint32_t*)CONFIG_DATA_X_MINPOINT_OFFSET);
					rsSendFormat[6] = (*(uint32_t*)(CONFIG_DATA_X_MIDDLEPOINT_OFFSET + 1));
					rsSendFormat[7] = (*(uint32_t*)CONFIG_DATA_X_MIDDLEPOINT_OFFSET);
					for(i = 1; i < 8; i++)
					{
						rsSendFormat[8] += rsSendFormat[i];
					}
					HAL_UART_Transmit(&huart1, (uint8_t*)rsSendFormat, 9, 5000);

				break;

				case COMMAND_YCALIB_READ:
					rsSendFormat[0] = COMMAND_HEADER;
					rsSendFormat[1] = COMMAND_YCALIB_READ;
					rsSendFormat[2] = (*(uint32_t*)(CONFIG_DATA_Y_MAXPOINT_OFFSET + 1));
					rsSendFormat[3] = (*(uint32_t*)CONFIG_DATA_Y_MAXPOINT_OFFSET);
					rsSendFormat[4] = (*(uint32_t*)(CONFIG_DATA_Y_MINPOINT_OFFSET + 1));
					rsSendFormat[5] = (*(uint32_t*)CONFIG_DATA_Y_MINPOINT_OFFSET);
					rsSendFormat[6] = (*(uint32_t*)(CONFIG_DATA_Y_MIDDLEPOINT_OFFSET + 1));
					rsSendFormat[7] = (*(uint32_t*)CONFIG_DATA_Y_MIDDLEPOINT_OFFSET);
					for(i = 1; i < 8; i++)
					{
						rsSendFormat[8] += rsSendFormat[i];
					}
					HAL_UART_Transmit(&huart1, (uint8_t*)rsSendFormat, 9, 5000);

				break;
					//default***********************
					case COMMAND_DEFAULT_MODSEL_READ:
					rsSendFormat[0] = COMMAND_HEADER;
					rsSendFormat[1] = COMMAND_DEFAULT_MODSEL_READ;
					rsSendFormat[2] = (*(uint32_t*)DEFAULT_CONFIG_DATA_INTERFACE_OFFSET);;
					rsSendFormat[3] = 0x00;
					rsSendFormat[4] = 0x00;
					rsSendFormat[5] = 0x00;
					rsSendFormat[6] = 0x00;
					rsSendFormat[7] = 0x00;
					for(i = 1; i < 8; i++)
					{
						rsSendFormat[8] += rsSendFormat[i];
					}
					HAL_UART_Transmit(&huart1, (uint8_t*)rsSendFormat, 9, 5000);
				break;

				case COMMAND_DEFAULT_XCALIB_READ:
					rsSendFormat[0] = COMMAND_HEADER;
					rsSendFormat[1] = COMMAND_DEFAULT_XCALIB_READ;
					rsSendFormat[2] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET + 1)); //MSB
					rsSendFormat[3] = (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET);		 //LSB
					rsSendFormat[4] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET + 1));
					rsSendFormat[5] = (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET);
					rsSendFormat[6] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET + 1));
					rsSendFormat[7] = (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET);

					for(i = 1; i < 8; i++)
					{
						rsSendFormat[8] += rsSendFormat[i];
					}

					HAL_UART_Transmit(&huart1, (uint8_t*)rsSendFormat, 9, 5000);

				break;

				case COMMAND_DEFAULT_YCALIB_READ:
					rsSendFormat[0] = COMMAND_HEADER;
					rsSendFormat[1] = COMMAND_DEFAULT_YCALIB_READ;
					rsSendFormat[2] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET + 1));
					rsSendFormat[3] = (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET);
					rsSendFormat[4] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET + 1));
					rsSendFormat[5] = (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET);
					rsSendFormat[6] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET + 1));
					rsSendFormat[7] = (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET);
					for(i = 1; i < 8; i++)
					{
						rsSendFormat[8] += rsSendFormat[i];
					}
					HAL_UART_Transmit(&huart1, (uint8_t*)rsSendFormat, 9, 5000);

				break;

					//default ****************************/
				case COMMAND_BOOT_MODE://düzenleenecek
					bootloaderCommand();
				break;

				case COMMAND_SYSTEM_RESET:
					sendAckUart();
					HAL_NVIC_SystemReset();
				break;

				case COMMAND_CALIBRATION_START:
					remoteMode = 0;
					sendAckUart();
				break;

				case COMMAND_REFRESH_CONFIG_DATA:
					refreshFlashUserConf(&tempTkkConfig);
					calculateJostickBorders(&userTkkConfig, &tkkJoystickBorder);
					remoteMode = 1;
					sendAckUart();
				break;

				case COMMAND_SET_DEFAULT_CONFIG_DATA:

				break;
			}
	}

	else if(userTkkConfig.tkkModSelection == TKK_MOD_CAN)
	{
		//CAN
		CAN_TxHeaderTypeDef txHeader = {0};

		txHeader.StdId = 0x321;
		txHeader.IDE   = CAN_ID_STD;
		txHeader.RTR   = CAN_RTR_DATA;
		txHeader.DLC   = 8;
		txHeader.TransmitGlobalTime = DISABLE;

//		uint8_t data[8] = { rs422Frame_[0],
//							rs422Frame_[1],
//							rs422Frame_[2],
//							rs422Frame_[3],
//							rs422Frame_[4],
//							rs422Frame_[5],
//							rs422Frame_[6],
//							rs422Frame_[7]};

		uint32_t txMailbox;



		switch (rxBuffer[1])
			{
				case COMMAND_MODSEL_WRITE:
					tempTkkConfig.tkkModSelection = rxBuffer[1];
					sendAckCan();
				break;

				case COMMAND_XCALIB_WRITE:
					tempTkkConfig.xMax = rxBuffer[1]<<8 | rxBuffer[2];
					tempTkkConfig.xMin = rxBuffer[3]<<8 | rxBuffer[4];
					tempTkkConfig.xMid = rxBuffer[5]<<8 | rxBuffer[6];
					sendAckCan();
				break;

				case COMMAND_YCALIB_WRITE:
					tempTkkConfig.yMax = rxBuffer[1]<<8 | rxBuffer[2];
					tempTkkConfig.yMin = rxBuffer[3]<<8 | rxBuffer[4];
					tempTkkConfig.yMid = rxBuffer[5]<<8 | rxBuffer[6];
					sendAckCan();
				break;

				case COMMAND_DEBOUNCE_WRITE://debounce tarafi düzenlenecek
				break;

				case COMMAND_MODSEL_READ:
					canSendFormat[0] = COMMAND_HEADER;
					canSendFormat[1] = COMMAND_MODSEL_READ;
					canSendFormat[2] = (*(uint32_t*)CONFIG_DATA_INTERFACE_OFFSET);;
					canSendFormat[3] = 0x00;
					canSendFormat[4] = 0x00;
					canSendFormat[5] = 0x00;
					canSendFormat[6] = 0x00;
					canSendFormat[7] = 0x00;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canSendFormat, &txMailbox) != HAL_OK)
					{
					// TX kuyruğu dolu vs. durumunda hata yönetimi
					}
				break;

				case COMMAND_XCALIB_READ:
					canSendFormat[0] = COMMAND_HEADER;
					canSendFormat[1] = COMMAND_XCALIB_READ;
					canSendFormat[2] = (*(uint32_t*)(CONFIG_DATA_X_MAXPOINT_OFFSET + 1));
					canSendFormat[3] = (*(uint32_t*)CONFIG_DATA_X_MAXPOINT_OFFSET);
					canSendFormat[4] = (*(uint32_t*)(CONFIG_DATA_X_MINPOINT_OFFSET + 1));
					canSendFormat[5] = (*(uint32_t*)CONFIG_DATA_X_MINPOINT_OFFSET);
					canSendFormat[6] = (*(uint32_t*)(CONFIG_DATA_X_MIDDLEPOINT_OFFSET + 1));
					canSendFormat[7] = (*(uint32_t*)CONFIG_DATA_X_MIDDLEPOINT_OFFSET);

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canSendFormat, &txMailbox) != HAL_OK)
					{
					// TX kuyruğu dolu vs. durumunda hata yönetimi
					}
				break;

				case COMMAND_YCALIB_READ:
					canSendFormat[0] = COMMAND_HEADER;
					canSendFormat[1] = COMMAND_YCALIB_READ;
					canSendFormat[2] = (*(uint32_t*)(CONFIG_DATA_Y_MAXPOINT_OFFSET + 1));
					canSendFormat[3] = (*(uint32_t*)CONFIG_DATA_Y_MAXPOINT_OFFSET);
					canSendFormat[4] = (*(uint32_t*)(CONFIG_DATA_Y_MINPOINT_OFFSET + 1));
					canSendFormat[5] = (*(uint32_t*)CONFIG_DATA_Y_MINPOINT_OFFSET);
					canSendFormat[6] = (*(uint32_t*)(CONFIG_DATA_Y_MIDDLEPOINT_OFFSET + 1));
					canSendFormat[7] = (*(uint32_t*)CONFIG_DATA_Y_MIDDLEPOINT_OFFSET);

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canSendFormat, &txMailbox) != HAL_OK)
					{
					// TX kuyruğu dolu vs. durumunda hata yönetimi
					}
				break;
					//default***********************
					case COMMAND_DEFAULT_MODSEL_READ:
					canSendFormat[0] = COMMAND_HEADER;
					canSendFormat[1] = COMMAND_DEFAULT_MODSEL_READ;
					canSendFormat[2] = (*(uint32_t*)DEFAULT_CONFIG_DATA_INTERFACE_OFFSET);;
					canSendFormat[3] = 0x00;
					canSendFormat[4] = 0x00;
					canSendFormat[5] = 0x00;
					canSendFormat[6] = 0x00;
					canSendFormat[7] = 0x00;

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canSendFormat, &txMailbox) != HAL_OK)
					{
					// TX kuyruğu dolu vs. durumunda hata yönetimi
					}
				break;

				case COMMAND_DEFAULT_XCALIB_READ:
					canSendFormat[0] = COMMAND_HEADER;
					canSendFormat[1] = COMMAND_DEFAULT_XCALIB_READ;
					canSendFormat[2] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET + 1)); //MSB
					canSendFormat[3] = (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET);		 //LSB
					canSendFormat[4] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET + 1));
					canSendFormat[5] = (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET);
					canSendFormat[6] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET + 1));
					canSendFormat[7] = (*(uint32_t*)DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET);

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canSendFormat, &txMailbox) != HAL_OK)
					{
					// TX kuyruğu dolu vs. durumunda hata yönetimi
					}
				break;

				case COMMAND_DEFAULT_YCALIB_READ:
					canSendFormat[0] = COMMAND_HEADER;
					canSendFormat[1] = COMMAND_DEFAULT_YCALIB_READ;
					canSendFormat[2] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET + 1));
					canSendFormat[3] = (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET);
					canSendFormat[4] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET + 1));
					canSendFormat[5] = (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET);
					canSendFormat[6] = (*(uint32_t*)(DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET + 1));
					canSendFormat[7] = (*(uint32_t*)DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET);

					if (HAL_CAN_AddTxMessage(&hcan, &txHeader, canSendFormat, &txMailbox) != HAL_OK)
					{
					// TX kuyruğu dolu vs. durumunda hata yönetimi
					}
				break;

					//default ****************************/
				case COMMAND_BOOT_MODE://düzenleenecek
					bootloaderCommand();
				break;

				case COMMAND_SYSTEM_RESET:
					sendAckCan();
					HAL_NVIC_SystemReset();
				break;

				case COMMAND_CALIBRATION_START:
					remoteMode = 0;
					sendAckCan();
				break;

				case COMMAND_REFRESH_CONFIG_DATA:
					refreshFlashUserConf(&tempTkkConfig);
					calculateJostickBorders(&userTkkConfig, &tkkJoystickBorder);
					remoteMode = 1;
					sendAckCan();
				break;

				case COMMAND_SET_DEFAULT_CONFIG_DATA:

				break;
			}
	}

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_arg)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  if(userTkkConfig.tkkModSelection == TKK_MOD_CAN)
  {
	  if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
	    {
	      return;
	    }

	    if (rxHeader.IDE == CAN_ID_STD && rxHeader.RTR == CAN_RTR_DATA)
	    {
	  	  if(rxHeader.StdId == 0x101)
	  	  {
	  		    checkCommand(rxData);
	  	  }
	    }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	unsigned int i;
	tempRxDataIn = rxDataIn;

	if(userTkkConfig.tkkModSelection == TKK_MOD_RS422)
	{
		if(rxBufferDataCounter==0 && rxDataIn==0xA5)
			{
				rxBuffer[0] = rxDataIn;
				rxBufferDataCounter++;
			}
			else if(rxBufferDataCounter>0 && rxBufferDataCounter<8)
			{
				rxBuffer[rxBufferDataCounter] = rxDataIn;
				rxBufferDataCounter++;
			}
			else if(rxBufferDataCounter==8)
			{
				/* checksum control */
				rxDataCheksum = 0;
				rxBuffer[rxBufferDataCounter] = rxDataIn;

				for(i=1;i<9;i++)
				{
					rxDataCheksum += rxBuffer[i];
				}
				if(rxDataCheksum==0)
				{
					/* checkSum OK, process the command */
					checkCommand(rxBuffer);
					rxBufferDataCounter = 0; /* test breakpoint icin */
				}
				else
				{
					/* error, do nothing*/
					errorCounter++;
				}
				rxBufferDataCounter = 0;
			}
			else
			{
				rxBufferDataCounter = 0;
			}
	}
}

void writeDefaultConfigPage(void)
{
	uint32_t index = 0;

	defaultTkkConfig.tkkModSelection = TKK_MOD_RS422;

	defaultTkkConfig.yMin = 32900;
	defaultTkkConfig.yMax = 65535;
	defaultTkkConfig.yMid = 49220;

	defaultTkkConfig.xMin = 32900;
	defaultTkkConfig.xMax = 65535;
	defaultTkkConfig.xMid = 49220;

	for(index = 0; index < BUTTON_COUNT; index++)
	{
		defaultTkkConfig.debounceFactor[index] = 20;
	}

	volatile uint32_t i = 0;

	HAL_FLASH_Unlock();

	static FLASH_EraseInitTypeDef EraseInitStructDefaultPage;
	EraseInitStructDefaultPage.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStructDefaultPage.PageAddress = DEFAULT_CONFIG_DATA_BASE_ADDR;
	EraseInitStructDefaultPage.NbPages = 1;

	if(HAL_FLASHEx_Erase(&EraseInitStructDefaultPage, &PAGEError) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_INTERFACE_OFFSET		, defaultTkkConfig.tkkModSelection );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET	, defaultTkkConfig.xMid);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET		, defaultTkkConfig.xMin);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET		, defaultTkkConfig.xMax);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET	, defaultTkkConfig.yMid);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET		, defaultTkkConfig.yMin);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET		, defaultTkkConfig.yMax);

	for(i = 0; i < 32; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, DEFAULT_CONFIG_DATA_DEBOUNCEFACTOR_BASE + i*CONFIG_FLASH_ADDR_INCREMENT, defaultTkkConfig.debounceFactor[i]);
	}

	HAL_FLASH_Lock();
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
