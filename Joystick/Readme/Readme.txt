YAZILIM:
STM32 ST-LINK Utility

JOYSTICK 26LI HARWIN KONNEKTÖRÜ
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |    10   |    11   |    12   |    13   |   
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
| 28VRTN  |   CANL  |  CANH   | CANGND  | MCUGND  |  SWDIO  |  SWCLK  | 3.3VDC  |  NRST   | BYPSIN1 | BYPSIN2 | BYPSIN3 | BYPSIN4 |         
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|   28V   | RS422GND| RS422RXP| RS422RXN| RS422TXP| RS422TXP| RS422TXN|         |         | USB5V   | MCUGND  | USB_N   | USB_P   |        
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    14   |    15   |    16   |    17   |    18   |    19   |    20   |    21   |    22   |    23   |    24   |    25   |    26   |   
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+

STLINK KONNEKTÖRÜ
                                        +---------+---------+
                                        +---------+---------+
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|   10    |    9    |    8    |    7    |    6    |    5    |    4    |    3    |    2    |    1    |
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|   3.3V  |         |         |         |         |   CLK   |   DATA  |         |         |         |
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|         |         |         |         |         |         |   GND   |         |         |         |
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    11   |    12   |    13   |    14   |    15   |    16   |    17   |    18   |    19   |    20   |
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



COMMAND_MODSEL_WRITE 
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |  BYTE NUMBER 
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|  KEY ID | FUNCNUM |  MODE   |    X    |    X    |    X    |    X    |    X    |    X    |CHECKNUM |  DESCRIPTION 
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    A5   |   01    |   00    |   00    |   00    |    00   |   00    |   00    |   00    |   FF    |  EXAMPLE
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+

KEY ID:	  A5	
FUNCNUM:  01
MODE:     REMOTE = 00 CALIBRATION= 01 
CHECKNUM: BYTE2-BYTE9 TOPLAMINDAN OLUŞAN "CHECKSUM8 2s COMPLEMENT" METODUYLA HESAPLANAN DEĞERDİR. 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
COMMAND_XCALIB_WRITE 
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |  BYTE NUMBER  
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|  KEY ID | FUNCNUM | xMAX MSB| xMAX LSB| xMIN MSB| xMIN LSB| xMID MSB| xMID LSB|    X    |CHECKNUM |  DESCRIPTION
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    A5   |   02    |   12    |   34    |   34    |    56   |   56    |   78    |   00    |   60    |  EXAMPLE
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+

KEY ID:	  A5	
FUNCNUM:  02
xMAX MSB: 16 BIT X EKSENİ MAXIMUM VERİSİNİN EN ANLAMLI BİTİ 
xMAX LSB: 16 BIT X EKSENİ MAXIMUM VERİSİNİN EN ANLAMSIZ BİTİ 
xMIN MSB: 16 BIT X EKSENİ MINIMUM VERİSİNİN EN ANLAMLI BİTİ 
xMIN LSB: 16 BIT X EKSENİ MINIMUM VERİSİNİN EN ANLAMSIZ BİTİ 
xMID MSB: 16 BIT X EKSENİ MIDDLE VERİSİNİN EN ANLAMLI BİTİ 
xMID LSB: 16 BIT X EKSENİ MIDDLE VERİSİNİN EN ANLAMSIZ BİTİ 
CHECKNUM: BYTE2-BYTE9 TOPLAMINDAN OLUŞAN "CHECKSUM8 2s COMPLEMENT" METODUYLA HESAPLANAN DEĞERDİR. 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
COMMAND_YCALIB_WRITE 
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    0    |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |  BYTE NUMBER  
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|  KEY ID | FUNCNUM | yMAX MSB| yMAX LSB| yMIN MSB| yMIN LSB| yMID MSB| yMID LSB|    X    |CHECKNUM |  DESCRIPTION
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
|    A5   |   03    |   12    |   34    |   34    |    56   |   56    |   78    |   00    |   60    |  EXAMPLE
+---------+---------+---------+---------+---------+---------+---------+---------+---------+---------+

KEY ID:	  A5	
FUNCNUM:  03
yMAX MSB: 16 BIT Y EKSENİ MAXIMUM VERİSİNİN EN ANLAMLI BİTİ 
yMAX LSB: 16 BIT Y EKSENİ MAXIMUM VERİSİNİN EN ANLAMSIZ BİTİ 
yMIN MSB: 16 BIT Y EKSENİ MINIMUM VERİSİNİN EN ANLAMLI BİTİ 
yMIN LSB: 16 BIT Y EKSENİ MINIMUM VERİSİNİN EN ANLAMSIZ BİTİ 
yMID MSB: 16 BIT Y EKSENİ MIDDLE VERİSİNİN EN ANLAMLI BİTİ 
yMID LSB: 16 BIT Y EKSENİ MIDDLE VERİSİNİN EN ANLAMSIZ BİTİ 
CHECKNUM: BYTE2-BYTE9 TOPLAMINDAN OLUŞAN "CHECKSUM8 2s COMPLEMENT" METODUYLA HESAPLANAN DEĞERDİR. 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
aşağısı girilmedi

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