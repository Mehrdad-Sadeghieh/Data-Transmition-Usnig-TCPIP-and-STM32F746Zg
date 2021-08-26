/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code implements a http server application based on 
  *          Netconn API of LwIP stack and FreeRTOS. This application uses 
  *          STM32F7xx the ETH HAL API to transmit and receive data. 
  *          The communication is done with a web browser of a remote PC.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"

#include "httpserver-netconn.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif gnetif; /* network interface structure */
int Pulses=0;
int i=0;
osThreadId MotorTaskID;

//osMailQDef(Motor_q, 300,const char *);
osMailQId Motor_q_id;
osSemaphoreDef (my_semaphore);    // Declare semaphore
osSemaphoreId  my_semaphore_id; // Semaphore ID
void *addrRecvDataToQ;
//osMailQDef(Motor_q, 5, uint32_t);
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void StartThread(void const * argument);
static void Netif_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
void sendstring(char *s);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void TimerInit(){
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOBEN ;
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOAEN ;
	RCC-> APB1ENR |=RCC_APB1ENR_TIM3EN;
	RCC-> APB1ENR |=RCC_APB1ENR_TIM4EN;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL5_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL6_1;
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL7_1;
	GPIOA->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	GPIOB->MODER |= GPIO_MODER_MODER6_1;
	GPIOB->MODER |= GPIO_MODER_MODER7_1;
 //------------------------General timer Setup
	TIM3->ARR = 0xFFFF;
	TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0);
	TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	TIM3->SMCR |= TIM_SMCR_SMS_0;// | TIM_SMCR_SMS_1;   //step 5
	TIM3->CR1 |= TIM_CR1_CEN ;
	//------------------------General timer Setup
	TIM4->ARR = 0xFFFF;
	TIM4->CCMR1 |= (TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0);
	TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	TIM4->SMCR |= TIM_SMCR_SMS_0;// | TIM_SMCR_SMS_1;   //step 5
	TIM4->CR1 |= TIM_CR1_CEN ;
	//------------------------Advanced Timer Setup
//	RCC-> APB2ENR |=RCC_APB2ENR_TIM1EN;
//	GPIOA->AFR[1] |= GPIO_AFRH_AFRH1_0;
//	GPIOB->AFR[1] |= GPIO_AFRH_AFRH0_0;
//	GPIOA->MODER |= GPIO_MODER_MODER8_1;
//	GPIOA->MODER |= GPIO_MODER_MODER9_1;
//	TIM1->ARR = 0xFFFF;
//	TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0);
//	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
//	TIM1->SMCR |= TIM_SMCR_SMS_0; //| TIM_SMCR_SMS_1;   //step 5
//	TIM1->CR1 |= TIM_CR1_CEN ;
	
}

void MotorInit(){
	
	RCC->AHB1ENR|= RCC_AHB1ENR_GPIOAEN ;
	GPIOA->MODER |= GPIO_MODER_MODER8_1;
	GPIOA->AFR[1] |= GPIO_AFRH_AFRH0_0;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* Peripheral clock enable */
//Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
/* Set the Timer prescaler to get 1MHz as counter clock */
TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); /* Select the up counter mode */
TIM1->CR1 |= TIM_COUNTERMODE_UP;
TIM1->CR1 &= ~TIM_CR1_CKD;
TIM1->CR1 |= TIM_CLOCKDIVISION_DIV1; /* Set the clock division to 1*/
TIM1->ARR = 2000; /* Set the Autoreload value */
TIM1->CCR1 = 1000; /* Set the Pulse value */
TIM1->PSC = 216; /* Set the Prescaler value */
TIM1->RCR = 200 - 1; /* Set the Repetition counter value */
TIM1->EGR = TIM_EGR_UG; /* Generate an update event to reload the Prescaler
and the repetition counter value immediately */
TIM1->SMCR = RESET; /* Configure the Internal Clock source */
TIM1->CR1 |= TIM_CR1_OPM; /* Select the OPM Mode */
TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_OC1M;
TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_CC1S;
TIM1->CCMR1 |= TIM_OCMODE_PWM1;
/* Select the Channel 1 Output Compare and the Mode */
TIM1->CCER &= (uint16_t)~TIM_CCER_CC1P;
/* Set the Output Compare Polarity to High */
TIM1->CCER |= TIM_OCPOLARITY_HIGH;
TIM1->CCER = TIM_CCER_CC1E; /* Enable the Compare output channel 1 */
TIM1->BDTR |= TIM_BDTR_MOE; /* Enable the TIM main Output */
TIM1->CR1 |= TIM_CR1_CEN; /* Enable the TIM peripheral */
	
}
void MotorTask(void const * argument){
//osSemaphoreWait(my_semaphore_id, osWaitForever);
while(1){
osEvent event = osMailGet(Motor_q_id, osWaitForever);
MotorData MotorRecvQ = *(MotorData *)event.value.p;       // ".p" indicates that the message is a pointer
//const char *temp= *((char**)MotorRecvQ);
Pulses=atoi(MotorRecvQ.RcvPulses);	
//sscanf(temp,"%d",&Pulses);	
osMailFree(Motor_q_id,&MotorRecvQ );
//if(Pulses!=0){
	
TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); /* Select the up counter mode */
TIM1->CR1 |= TIM_COUNTERMODE_UP;
TIM1->CR1 &= ~TIM_CR1_CKD;
TIM1->CR1 |= TIM_CLOCKDIVISION_DIV1; /* Set the clock division to 1*/
TIM1->ARR = 1000; /* Set the Autoreload value */
TIM1->CCR1 = 1000; /* Set the Pulse value */
TIM1->PSC = 216; /* Set the Prescaler value */
TIM1->RCR = Pulses - 1; /* Set the Repetition counter value */
TIM1->EGR = TIM_EGR_UG; /* Generate an update event to reload the Prescaler
and the repetition counter value immediately */
TIM1->SMCR = RESET; /* Configure the Internal Clock source */
TIM1->CR1 |= TIM_CR1_OPM; /* Select the OPM Mode */
TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_OC1M;
TIM1->CCMR1 &= (uint16_t)~TIM_CCMR1_CC1S;
TIM1->CCMR1 |= TIM_OCMODE_PWM1;
/* Select the Channel 1 Output Compare and the Mode */
TIM1->CCER &= (uint16_t)~TIM_CCER_CC1P;
/* Set the Output Compare Polarity to High */
TIM1->CCER |= TIM_OCPOLARITY_HIGH;
TIM1->CCER = TIM_CCER_CC1E; /* Enable the Compare output channel 1 */
TIM1->BDTR |= TIM_BDTR_MOE; /* Enable the TIM main Output */
TIM1->CR1 |= TIM_CR1_CEN;
Pulses=0;	
	//}
//osThreadSetPriority(MotorTaskID,osPriorityIdle);
//osSemaphoreRelease(my_semaphore_id);
//osThreadYield(); 
	}

osThreadTerminate(MotorTaskID);
}


int main(void)
{
  /* Configure the MPU attributes as Device memory for ETH DMA descriptors */
  MPU_Config();
  
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();  
  //char buffer[8];
	//sprintf (buffer,"%s\r\n","Test");
  /* Configure the system clock to 216 MHz */
  SystemClock_Config(); 

  /*configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
	TimerInit();
	MotorInit();

	
	
	
  /* Init thread */
#if defined(__GNUC__)
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
#else
 osThreadDef(Start, StartThread, osPriorityNormal, 0,configMINIMAL_STACK_SIZE*5);
#endif

my_semaphore_id = osSemaphoreCreate(osSemaphore(my_semaphore), 1);
//  osThreadDef(Motor,MotorTask,osPriorityNormal,1,configMINIMAL_STACK_SIZE);
//	MotorTaskID=osThreadCreate(osThread(Motor), "TASK");
	osThreadCreate (osThread(Start), NULL);
  
  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

/**
  * @brief  Start Thread 
  * @param  argument not used
  * @retval None
  */
static void StartThread(void const * argument)
{ 
  /* Create tcp_ip stack thread */
  tcpip_init(NULL, NULL);
  
  /* Initialize the LwIP stack */
  Netif_Config();
  
  /* Initialize webserver demo */
  http_server_netconn_init();
  
  /* Notify user about the network interface config */
  User_notification(&gnetif);
  
#ifdef USE_DHCP
  /* Start DHCPClient */
  osThreadDef(DHCP, DHCP_thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
  osThreadCreate (osThread(DHCP), &gnetif);
#endif

  for( ;; )
  {
    /* Delete the Init Thread */ 
    osThreadTerminate(NULL);
  }
}

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
 
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP_ADDR4(&netmask,NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
  IP_ADDR4(&gw,GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  /* add the network interface */    
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
  /*  Registers the default network interface. */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  //RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}
void send_char(char ch)
{
	while(!(USART3 -> ISR & USART_ISR_TXE)); //Check if the transmitter buffer is empty otherwise stay
	USART3 ->TDR =(ch & 0x0ff);
}

void sendstring(char *s)
{
	while((*s)) //terminates when a return character has encountered
	{
		
		send_char(*s);
		s++;
	}

}


/**
  * @brief  Configure the MPU attributes .
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();
  
  /* Configure the MPU as Normal Non Cacheable for Ethernet Buffers in the SRAM2 */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x2004C000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Configure the MPU as Device for Ethernet Descriptors in the SRAM2 */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x2004C000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
