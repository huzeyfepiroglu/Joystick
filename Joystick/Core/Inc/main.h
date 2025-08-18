/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flashAddr.h"
#include "mxconstants.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_18_Pin GPIO_PIN_13
#define BTN_18_GPIO_Port GPIOC
#define BTN_17_Pin GPIO_PIN_14
#define BTN_17_GPIO_Port GPIOC
#define BTN_23_Pin GPIO_PIN_15
#define BTN_23_GPIO_Port GPIOC
#define BTN_24_Pin GPIO_PIN_0
#define BTN_24_GPIO_Port GPIOC
#define BTN_21_Pin GPIO_PIN_1
#define BTN_21_GPIO_Port GPIOC
#define BTN_22_Pin GPIO_PIN_2
#define BTN_22_GPIO_Port GPIOC
#define BTN_07_Pin GPIO_PIN_3
#define BTN_07_GPIO_Port GPIOC
#define BTN_08_Pin GPIO_PIN_0
#define BTN_08_GPIO_Port GPIOA
#define BTN_05_Pin GPIO_PIN_1
#define BTN_05_GPIO_Port GPIOA
#define BTN_06_Pin GPIO_PIN_2
#define BTN_06_GPIO_Port GPIOA
#define BTN_03_Pin GPIO_PIN_3
#define BTN_03_GPIO_Port GPIOA
#define BTN_19_Pin GPIO_PIN_4
#define BTN_19_GPIO_Port GPIOA
#define BTN_01_Pin GPIO_PIN_5
#define BTN_01_GPIO_Port GPIOA
#define BTN_02_Pin GPIO_PIN_6
#define BTN_02_GPIO_Port GPIOA
#define FIN_ADC4_Pin GPIO_PIN_7
#define FIN_ADC4_GPIO_Port GPIOA
#define FIN_ADC3_Pin GPIO_PIN_4
#define FIN_ADC3_GPIO_Port GPIOC
#define FIN_ADC2_Pin GPIO_PIN_5
#define FIN_ADC2_GPIO_Port GPIOC
#define FIN_ADC4B0_Pin GPIO_PIN_0
#define FIN_ADC4B0_GPIO_Port GPIOB
#define AN_ADC4_Pin GPIO_PIN_1
#define AN_ADC4_GPIO_Port GPIOB
#define AN_ADC3_Pin GPIO_PIN_2
#define AN_ADC3_GPIO_Port GPIOB
#define AN_ADC2_Pin GPIO_PIN_8
#define AN_ADC2_GPIO_Port GPIOE
#define AN_ADC1_Pin GPIO_PIN_9
#define AN_ADC1_GPIO_Port GPIOE
#define BTN_04_Pin GPIO_PIN_14
#define BTN_04_GPIO_Port GPIOB
#define BTN_20_Pin GPIO_PIN_15
#define BTN_20_GPIO_Port GPIOB
#define BTN_14_Pin GPIO_PIN_8
#define BTN_14_GPIO_Port GPIOD
#define BTN_15_Pin GPIO_PIN_6
#define BTN_15_GPIO_Port GPIOC
#define BTN_16_Pin GPIO_PIN_7
#define BTN_16_GPIO_Port GPIOC
#define BTN_13_Pin GPIO_PIN_8
#define BTN_13_GPIO_Port GPIOC
#define BTN_12_Pin GPIO_PIN_9
#define BTN_12_GPIO_Port GPIOC
#define BTN_29_Pin GPIO_PIN_8
#define BTN_29_GPIO_Port GPIOA
#define BTN_11_Pin GPIO_PIN_6
#define BTN_11_GPIO_Port GPIOF
#define BTN_10_Pin GPIO_PIN_7
#define BTN_10_GPIO_Port GPIOF
#define BTN_09_Pin GPIO_PIN_15
#define BTN_09_GPIO_Port GPIOA
#define BTN_27_Pin GPIO_PIN_10
#define BTN_27_GPIO_Port GPIOC
#define BTN_28_Pin GPIO_PIN_11
#define BTN_28_GPIO_Port GPIOC
#define BTN_25_Pin GPIO_PIN_12
#define BTN_25_GPIO_Port GPIOC
#define BTN_26_Pin GPIO_PIN_3
#define BTN_26_GPIO_Port GPIOB
#define BTN_32_Pin GPIO_PIN_4
#define BTN_32_GPIO_Port GPIOB
#define BTN_31_Pin GPIO_PIN_5
#define BTN_31_GPIO_Port GPIOB
#define BTN_30_Pin GPIO_PIN_6
#define BTN_30_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
static const uint32_t DEFAULT_MSP_VALUE = 0x20001258;

//ADC Constants
enum { AVERAGE_WINDOW 	= 2 };
enum { ANALOG_COUNT 	= 2 };
enum { BUTTON_COUNT 	= 32 };

//const uint32_t 	AVERAGE_WINDOW 		= 2;
//const uint32_t	ANALOG_COUNT 		= 2;
//const uint32_t 	BUTTON_COUNT 		= 32;

//Digital Constants and declerations
typedef struct digitalInput_Struct
{
	GPIO_PinState inputVal;			//INIT - GPIO_PIN_RESET 	check if error is occured, change variable type to bit
	bool readFlag;					//INIT -> false
	uint16_t inputPin;				//CUSTOM
	GPIO_TypeDef *inputPort;		//CUSTOM
	GPIO_PinState readState;
	uint32_t debounceTimer;			//INIT -> 0
	uint32_t debounceFactor;		//CUSTOM-INIT

} digitalInput;

typedef struct joystickBorder_Struct
{
	uint32_t xLeftLow;
	uint32_t xLeftHigh;
	uint32_t xLeftRange;

	uint32_t xRightLow;
	uint32_t xRightHigh;
	uint32_t xRightRange;

	uint32_t yDownLow;
	uint32_t yDownHigh;
	uint32_t yDownRange;

	uint32_t yUpLow;
	uint32_t yUpHigh;
	uint32_t yUpRange;

} joystickBorder;

//Configuration Definitions & prototypes
typedef enum
{
	TKK_MOD_RS422 = 0,
	TKK_MOD_USB,
	TKK_MOD_CAN
}TKK_Modsel;

typedef struct Config_Struct
{
	uint16_t tkkModSelection;

	uint16_t xMin;
	uint16_t xMax;
	uint16_t xMid;

	uint16_t yMin;
	uint16_t yMax;
	uint16_t yMid;

	uint16_t debounceFactor[32];
} ConfigData;

#define CONFIG_FLASH_ADDR_INCREMENT 					0x04

#define CONFIG_BOOTMODE_BASE_ADDR 	  					ADDR_FLASH_PAGE_20
#define CONFIG_BOOTMODE_HEADER_OFFSET					(CONFIG_BOOTMODE_BASE_ADDR + 0x00)
#define CONFIG_BOOTMODE_INIT_OFFSET 					(CONFIG_BOOTMODE_BASE_ADDR + 0x02)
#define CONFIG_BOOTMODE_CHECKSUM_OFFSET   				(CONFIG_BOOTMODE_BASE_ADDR + 0x04)

#define CONFIG_DATA_BASE_ADDR 							ADDR_FLASH_PAGE_28 /*!!!!!!!!!!!!!!!!! düzenlenecek*/  		/* Base @ of Page 28, 2 Kbytes */
#define DEFAULT_CONFIG_DATA_BASE_ADDR 					ADDR_FLASH_PAGE_30											/*Default Base @ of Page 29, 2 Kbytes */

#define CONFIG_DATA_DEFAULT_SELECT						(CONFIG_DATA_BASE_ADDR + 0x00)
#define CONFIG_DATA_INTERFACE_OFFSET 					(CONFIG_DATA_BASE_ADDR + 0x02)
#define CONFIG_DATA_X_MIDDLEPOINT_OFFSET				(CONFIG_DATA_BASE_ADDR + 0x04)
#define CONFIG_DATA_X_MINPOINT_OFFSET					(CONFIG_DATA_BASE_ADDR + 0x06)
#define CONFIG_DATA_X_MAXPOINT_OFFSET					(CONFIG_DATA_BASE_ADDR + 0x08)
#define CONFIG_DATA_Y_MIDDLEPOINT_OFFSET				(CONFIG_DATA_BASE_ADDR + 0x0A)
#define CONFIG_DATA_Y_MINPOINT_OFFSET					(CONFIG_DATA_BASE_ADDR + 0x0C)
#define CONFIG_DATA_Y_MAXPOINT_OFFSET					(CONFIG_DATA_BASE_ADDR + 0x0E)
#define CONFIG_DATA_DEBOUNCEFACTOR_BASE					(CONFIG_DATA_BASE_ADDR + 0x50) 	//DEBOUNCE_FACTOR of btn0, increment by 0x02 for other buttons

#define DEFAULT_CONFIG_DATA_INTERFACE_OFFSET 			(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x00)/* COMMAND KISMI Düzelecek */
#define DEFAULT_CONFIG_DATA_X_MIDDLEPOINT_OFFSET		(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x02)
#define DEFAULT_CONFIG_DATA_X_MINPOINT_OFFSET			(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x04)
#define DEFAULT_CONFIG_DATA_X_MAXPOINT_OFFSET			(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x06)
#define DEFAULT_CONFIG_DATA_Y_MIDDLEPOINT_OFFSET		(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x08)
#define DEFAULT_CONFIG_DATA_Y_MINPOINT_OFFSET			(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x0A)
#define DEFAULT_CONFIG_DATA_Y_MAXPOINT_OFFSET			(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x0C)
#define DEFAULT_CONFIG_DATA_DEBOUNCEFACTOR_BASE			(DEFAULT_CONFIG_DATA_BASE_ADDR + 0x0E) //DEFAULT DEBOUNCE_FACTOR of btn0, increment by 0x02 for other buttons

#define COMMAND_HEADER 									0xA5
#define COMMAND_ACK 									0xFF
#define CHECKSUM_ACK 									0x01

#define COMMAND_REFRESH_CONFIG_DATA 					0xBB
#define COMMAND_SET_DEFAULT_CONFIG_DATA 				0xCC
#define COMMAND_CALIBRATION_START 						0xDD

#define COMMAND_MODSEL_WRITE 							0x01
#define COMMAND_XCALIB_WRITE 							0x02
#define COMMAND_YCALIB_WRITE 							0x03
#define COMMAND_DEBOUNCE_WRITE 							0x04

#define COMMAND_MODSEL_READ 							0x81
#define COMMAND_XCALIB_READ 							0x82
#define COMMAND_YCALIB_READ 							0x83
#define COMMAND_DEBOUNCE_READ 							0x84

#define COMMAND_DEFAULT_MODSEL_READ 					0x71
#define COMMAND_DEFAULT_XCALIB_READ 					0x72
#define COMMAND_DEFAULT_YCALIB_READ 					0x73
#define COMMAND_DEFAULT_DEBOUNCE_READ 					0x74

#define COMMAND_BOOT_MODE								0xC5
#define COMMAND_SYSTEM_RESET    						0xD5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
