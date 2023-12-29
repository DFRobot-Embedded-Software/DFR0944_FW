/**
  ******************************************************************************
  * @file    main.c
	* @author  MCU Software Team
	* @Version V1.0.0
  * @Date    21-Oct-2019
  * @brief   main function
  ******************************************************************************
  * This example is used to test uart0,when interrupt received data reaches 10, 
  * it is sent out from serial port.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"	

#define WWDG_TEST_TIME				0X20			//time 1401.6ms,PCLK=24M
#define WWDG_TEST_WINDOW_TIME	0X10  	  //700.8ms
#define WWDG_TEST_PRESCALER		0XFFFFF		//43.8ms
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t *RTU_UART_Data = NULL;
uint8_t DMXData[513];
uint64_t count_Us;


//static uint8_t ucRxCompleteFlag = 0;
volatile uint8_t receData;
volatile uint8_t RecIndexLen = 0;

/* Private macros-------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
UART_HandleTypeDef sUartxHandle = {0};
GPIO_InitTypeDef GPIO_InitStruct1 = {0};
WWDG_HandleTypeDef wwdg_test = {0};
FlagStatus wwdg_int = RESET;


void RCC_DelayF(uint32_t mdelay)
{
  __IO uint32_t Delay = mdelay * (24000000 / 8U / 1000000U);
  do 
  {
    __NOP();
  } 
  while (Delay --);
}



void gpio_init(void){
  GPIO_InitStruct1.Pin = GPIO_PIN_2;
	GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT;
	GPIO_InitStruct1.OpenDrain = GPIO_PUSHPULL;	
	GPIO_InitStruct1.Debounce.Enable = GPIO_DEBOUNCE_DISABLE;
	GPIO_InitStruct1.SlewRate = GPIO_SLEW_RATE_HIGH;
	GPIO_InitStruct1.DrvStrength = GPIO_DRV_STRENGTH_LOW;
	GPIO_InitStruct1.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct1);

  
}

/**
  * @brief WWDG Interrupt call
  * @retval None
  */
void HAL_WWDG_INT_Callback(void)
{
	wwdg_int = SET;
	
}
/**
  * @brief WWDG Init Configuration
  * @retval None
  */
void WWdg_Init(void)
{
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != true)
			printf("Test WWDG RESET ,NO RESET!\r\n");
	else
	{
		printf("Test WWDG RESET ,HAVE RESET!!!!\r\n");
		__HAL_RCC_CLEAR_RESET_FLAGS(RCC_FLAG_WWDGRST);
	}
		
	/*set init */
	wwdg_test.Instance = WWDG;
	wwdg_test.Init.Reload = WWDG_TEST_TIME;
	wwdg_test.Init.Window = WWDG_TEST_WINDOW_TIME;
	wwdg_test.Init.Prescaler = WWDG_TEST_PRESCALER;
	wwdg_test.Init.INTSet = WWDG_INT_ENABLE;
	/* Set interrupt priority and turn on interrupt*/	
	HAL_NVIC_SetPriority(WWDG_IRQn, PRIORITY_LOW);
  HAL_NVIC_EnableIRQ(WWDG_IRQn);
	HAL_WWDG_Init(&wwdg_test);
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();		
	
  /* Configure the system clock to HIRC 24MHz*/
  SystemClock_Config();
			
  /* Peripheral clock enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_I2C_CLK_ENABLE();
	__HAL_RCC_UART0_CLK_ENABLE();
  __HAL_RCC_UART1_CLK_ENABLE();
  __HAL_RCC_I2C_CLK_ENABLE();
  __HAL_RCC_WWDG_CLK_ENABLE();

	UartInit();
  GPIO_InitStruct1.Pin = GPIO_PIN_5;
	GPIO_InitStruct1.Mode = GPIO_MODE_OUTPUT;
	GPIO_InitStruct1.OpenDrain = GPIO_PUSHPULL;	
	GPIO_InitStruct1.Debounce.Enable = GPIO_DEBOUNCE_DISABLE;
	GPIO_InitStruct1.SlewRate = GPIO_SLEW_RATE_HIGH;
	GPIO_InitStruct1.DrvStrength = GPIO_DRV_STRENGTH_LOW;
	GPIO_InitStruct1.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct1);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
    
	sUartxHandle.Instance = UART0;
	sUartxHandle.Init.BaudRate = 250000;
	sUartxHandle.Init.BaudDouble = UART_BAUDDOUBLE_DISABLE;	// 
	sUartxHandle.Init.WordLength = UART_WORDLENGTH_8B;
	sUartxHandle.Init.Parity = UART_PARITY_NONE;
	sUartxHandle.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&sUartxHandle);
  HAL_UART_Receive_IT(&sUartxHandle, (uint8_t *)&receData, 1);	
	HAL_NVIC_EnableIRQ(UART0_IRQn); // 
  i2cInitSlave();
  i2cIRQConfig();
  memset(DMXData,0,513);
  WWdg_Init();//初始化看门狗
  while (1){
    if(wwdg_int != RESET)
		{
			HAL_WWDG_Refresh(&wwdg_test);
		}
    //DMX512协议规定
    HAL_UART_DeInit(&sUartxHandle);
    gpio_init();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
    RCC_DelayF(88);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
    RCC_DelayF(6);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_2);
    HAL_UART_Init(&sUartxHandle);
    RCC_DelayF(88);
    HAL_UART_Transmit_IT(&sUartxHandle, DMXData, 513);
    HAL_Delay(50);
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};	
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HIRC;
  RCC_OscInitStruct.HIRCState = RCC_HIRC_ON;
  RCC_OscInitStruct.HIRCCalibrationValue = RCC_HIRCCALIBRATION_24M;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
	
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HIRC;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APBCLKDivider = RCC_PCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }	
}
 

/**
  * @brief  UART MSP Init.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
		GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**if UARTx is UART0 
		GPIO Configuration:    
    PA1     ------> UART0_RXD
    PA2     ------> UART0_TXD
    */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF;
	GPIO_InitStruct.OpenDrain = GPIO_PUSHPULL;	
	GPIO_InitStruct.Debounce.Enable = GPIO_DEBOUNCE_DISABLE;
	GPIO_InitStruct.SlewRate = GPIO_SLEW_RATE_HIGH;
	GPIO_InitStruct.DrvStrength = GPIO_DRV_STRENGTH_LOW;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF5_UART0_TXD;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF;
	GPIO_InitStruct.Alternate = GPIO_AF5_UART0_RXD;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

     
}


void copyData(uint8_t* buf,uint16_t lne){
  uint16_t addr = 0;
  //memset(DMXData,0,513);
  addr = buf[2] << 8 | buf[1];
  //printf("len:%d",lne);
  memcpy(&DMXData[addr],&buf[3],lne-3);

}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&sUartxHandle, (uint8_t *)&receData, 1);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1);
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


/* Private function -------------------------------------------------------*/



