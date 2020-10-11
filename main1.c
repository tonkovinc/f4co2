/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
static uint8_t GAS_CONCENTRATION_READ[] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
uint8_t CO2_RESPONSE[9];
uint16_t ppm = 0;
char trans_str[64] = {0,};

uint8_t buf[64]; //???? ???????? ???????? ???????
uint8_t pos = 0; //??????? ???????? ????????
volatile uint8_t fakebuf[2]; //??? ?????, ????? ??????????? ?????? HAL ? CubeMX
int err = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int mystrlen(char * s1);
void puts0(char* data);
void intToAscii(uint32_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int mycmp(unsigned char* s1, char* s2, int f)
{
    int i;
    for (i = 0; i < f; i++)
    {
        if (*s1 != *s2)
        {
            return 0;
        }
        s1++;
        s2++;
    }
    return 1;
}

void clr_str(unsigned char* s1, unsigned int len)
{
        while (len != 0)
        {
            *s1 = 0;
            s1++;
            len--;
        }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{					
		if (huart->Instance==USART2) {
			buf[pos] = fakebuf[0]; 
			pos++;
			if ((pos>0) && ((buf[pos-1] == '\n') || (buf[pos-1] == '\r') || (pos > 63)))
				{
					if(pos>0) {buf[pos - 1] = '\0';} else buf[0] = 0;
					err = 1;
					
					if(mycmp(buf, "OK", 2)) { err = 0;}
					if(mycmp(buf, "Get_inf", 10)) {
							//puts0(Device_model_SW);puts0("\n");
							//puts0(SerialNum);puts0("\n");
							//puts0(SW_Version);//puts0("\n");//already there
							err = 0;
						}
					if(mycmp(buf, "calib", 5)) {
					 uint8_t setrangeA_cmd[9] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB}; // ?????? ???????? 0 - 5000ppm
					 HAL_UART_Transmit(&huart1, setrangeA_cmd, 9, 0xFFFF);

						err = 0;
					}

				if(err) {puts0("ERR\n>\n");}
				else { puts0("OK\n>\n");}
	  

				clr_str(buf, pos);
				pos=0;
			}
	HAL_UART_Receive_IT(&huart2, (uint8_t *)fakebuf,1); 
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	NVIC_EnableIRQ (USART2_IRQn); 
	__enable_irq ();

	HAL_UART_Receive_IT(&huart2, (uint8_t *)fakebuf,1);
	puts0("Program started\n");
	uint8_t ABClogic[9] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
	uint8_t ABClogic_responce[9];
	//HAL_UART_Transmit(&huart1, ABClogic, 9, 0xFFFF);
	/*HAL_UART_Receive(&huart1, ABClogic_responce,9, 0x0FFF);
  uint8_t ABC_i;
  uint8_t ABC_crc = 0;
  for (ABC_i = 1; ABC_i < 8; ABC_i++) {
			ABC_crc+=ABClogic_responce[ABC_i];
	}
  ABC_crc = 0xFF - ABC_crc;
  ABC_crc += 1;
  if ( !((ABClogic_responce[0] == 0xFF) && (ABClogic_responce[1] == 0x99) && (ABClogic_responce[8] == ABC_crc)) ) {
		puts0("ABC error\n");  
  } 
	else puts0("ABC OK!\n"); */ 
  /* USER CODE END 2 */
	
	uint8_t setrangeA_cmd[9] = {0xFF, 0x01, 0x99, 0x13, 0x88, 0x00, 0x00, 0x00, 0xCB}; // ?????? ???????? 0 - 5000ppm
	uint8_t setrangeA_response[9];
	HAL_UART_Transmit(&huart1, setrangeA_cmd, 9, 0xFFFF);
	HAL_UART_Receive(&huart1, setrangeA_response,9, 0x0FFF);
  /*uint8_t setrangeA_i;
  uint8_t setrangeA_crc = 0;
  for (setrangeA_i = 1; setrangeA_i < 8; setrangeA_i++) {
			setrangeA_crc+=setrangeA_response[setrangeA_i];
	}
  setrangeA_crc = 0xFF - setrangeA_crc;
  setrangeA_crc += 1;
  if ( !((setrangeA_response[0] == 0xFF) && (setrangeA_response[1] == 0x99) && (setrangeA_response[8] == setrangeA_crc)) ) {
		puts0("CRC error\n");  
  } 
	else puts0("CRC OK!\n");*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			HAL_UART_Transmit(&huart1, GAS_CONCENTRATION_READ, 9, 0xFFFF);
			HAL_UART_Receive(&huart1, CO2_RESPONSE,9, 0x0FFF);
			snprintf(trans_str, 48, "CO2_RESPONSE==%x,%x,%x,%x,%x,%x,%x,%x,%x\n", CO2_RESPONSE[0],CO2_RESPONSE[1],CO2_RESPONSE[2],CO2_RESPONSE[3],CO2_RESPONSE[4],CO2_RESPONSE[5],CO2_RESPONSE[6],CO2_RESPONSE[7],CO2_RESPONSE[8]);
			puts0(trans_str);
			uint8_t i, crc =  0;//32
			for (i = 1; i < 8; i++) crc+=CO2_RESPONSE[i];
			crc = 255 - crc;
			crc ++;
			if ( !((CO2_RESPONSE[0] == 0xFF) && (CO2_RESPONSE[1] == 0x86) && (CO2_RESPONSE[8] == crc)) ) {
				puts0("Checksum error\n");
			}
			else puts0("Checksum OK\n");
			//ppm = (((uint16_t)CO2_RESPONSE[2]) << 8) + CO2_RESPONSE[3];
			uint32_t responseHigh = (unsigned int) CO2_RESPONSE[2];
			uint32_t responseLow = (unsigned int) CO2_RESPONSE[3];
			uint32_t ppm = (256*responseHigh) + responseLow;
			snprintf(trans_str, 63, "ppm==%d\n",ppm );
			//intToAscii(ppm);
			puts0(trans_str);
			if (ppm > 800) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
 			HAL_Delay(10000);   
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int mystrlen(char * s1)
{
    unsigned int i = 0;
    while (*s1 != 0)
    {
        s1++;
        i++;
    }
    return i;
}

void puts0(char* data) 
{
	HAL_UART_Transmit(&huart2,(uint8_t*)data,mystrlen(data),0xFFFF);
}

void intToAscii(uint32_t value) {
    uint8_t i;
    uint8_t j = 0;
    uint8_t digit_start = 0;
    uint16_t digit = 0;
    uint32_t denom = 1000000000;

    if (value == 0) {
        trans_str[0] = '0';
        trans_str[1] = NULL;
    } else {
        for(i = 10; i > 0; i--) {
            digit = value / denom;
            if((digit_start == 1) || (digit != 0)) {
                digit_start = 1;
                value %= denom;
                trans_str[j++] = (digit + '0');
            }
            denom /= 10;
        }
        trans_str[j++] = NULL;
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
