/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include <math.h>
#include "stm32f4xx_adc.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "core_cm4.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "portmacro.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "portmacro.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define SET_RX() 	GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define SET_TX() 	GPIO_SetBits(GPIOA, GPIO_Pin_0)

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/
extern void _delay_ms(uint16_t val);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : vApplicationIdleHook
 * Description    :
 *******************************************************************************/
void vApplicationIdleHook(void)
{
}

/*******************************************************************************
 * Function Name  : vApplicationMallocFailedHook
 * Description    :
 *******************************************************************************/
void vApplicationMallocFailedHook(void)
{
	for(;;);
}

/*******************************************************************************
 * Function Name  : vApplicationStackOverflowHook
 * Description    :
 *******************************************************************************/
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	for(;;);
}

/*******************************************************************************
 * Function Name  : vApplicationTickHook
 * Description    :
 *******************************************************************************/
void vApplicationTickHook(void)
{
}

/*******************************************************************************
 * Function Name  : InitPeriph
 * Description    :
 *******************************************************************************/
void InitPeriph()
{
	/** DISCOVERY LEDS */
	//GREEN  - LD4 - PD12
	//ORANGE - LD3 - PD13
	//RED 	 - LD5 - PD14
	//BLUE	 - LD6 - PD15

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	/* RNG Peripheral enable */
	RNG_Cmd(ENABLE);
}

void EXTI15_10_IRQHandler()
{
	/* Make sure that interrupt flag is set */
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
}

/*******************************************************************************
 * Function Name  :
 * Description    :
 *******************************************************************************/
void vLedTask(void *pvParameters)
{
	(void)pvParameters;

	while(1)
	{
		static uint16_t temp = 0;
		if(++temp > 20)
		{
			temp = 0;
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15); //clear LEDs
			GPIOD->ODR ^= GPIO_Pin_14;
		}

		static uint16_t temp2 = 0;
		if(++temp2 > 100)
		{
			temp2 = 0;
			SET_TX();
			USART2->DR = 'A';  //Echo
			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
			SET_RX();
		}

		vTaskDelay(10);
	}
	vTaskDelete(NULL);}

/*******************************************************************************
 * Function Name  : main
 *******************************************************************************/
int main()
{
	InitPeriph();
	SystemInit();


	GPIO_InitTypeDef GPIO_InitStruct;  		// this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct;  	// this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure;  	// this is used to configure the NVIC (nested vector interrupt controller)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStruct.USART_BaudRate = 9600*3;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStruct);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);

	SET_RX();

	xTaskCreate(vLedTask, "task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);

	vTaskStartScheduler();

	return 0;
}

void USART2_IRQHandler()
{
	GPIO_SetBits(GPIOD, GPIO_Pin_15);

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		//GREEN  - LD4 - PD12
		//ORANGE - LD3 - PD13
		//RED 	 - LD5 - PD14
		//BLUE	 - LD6 - PD15

		uint8_t recvData = USART_ReceiveData(USART2);
		SET_TX();
		USART2->DR = USART2->DR;  //Echo
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		SET_RX();

		if(recvData == 0x00)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
		}
		else if(recvData == 0xFF)
		{
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
		}
	}
}

void assert_failed(uint8_t* file, uint32_t line)
{
	vTaskEndScheduler();

}
