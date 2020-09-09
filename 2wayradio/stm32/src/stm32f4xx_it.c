/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
//#include "main.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_CR1_RENEIE          ((uint16_t)0x0020)
#define USART_CR1_TXEIE           ((uint16_t)0x0080)


#define USART6_SR_BASE            (0x40011400)
#define USART6_CR1_BASE           (0x4001140C)

#define USART6_SR                *(__IO uint16_t * )(USART6_SR_BASE)
#define USART6_CR1               *(__IO uint16_t * )(USART6_CR1_BASE)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



#define USART6_RX_BUF_SIZE 1000
#define USART6_TX_BUF_SIZE 1000
__IO uint8_t USART6_Rx_Buffer[USART6_RX_BUF_SIZE];
__IO uint8_t USART6_Tx_Buffer[USART6_TX_BUF_SIZE];
__IO uint16_t USART6_Rx_ptr_in = 0;
__IO uint16_t USART6_TxCount = 0;
__IO uint16_t USART6_Tx_Start_Position = 0;
__IO uint16_t USART6_Tx_End_Position = 320;
__IO uint16_t USART6_Tx_Length = 320; /* End_Position - Start_Position */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
extern void GPIO_EXTI_Callback( uint16_t GPIO_Pin);
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

void USART6_IRQHandler(void)
{
//  if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
  /* USART_GetITStatusº¯ÊýÕ¼ÓÃÊ±¼ä¹ý¶à£¬µ¼ÖÂ¸ßËÙÍ¨Ñ¶ÒÅÂ©½ÓÊÕÊý¾Ý,¸ÄÎªÖ±½ÓÅÐ¶Ï±êÖ¾Î» */
//  if (USART6->SR  & USART_FLAG_RXNE){
  if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
	  USART6_Rx_Buffer[USART6_Rx_ptr_in]= USART_ReceiveData(USART6 );
    //USART6_Rx_Buffer[USART6_Rx_ptr_in] = USART6->DR;
    USART_SendData(USART6,USART6_Tx_Buffer[USART6_Rx_ptr_in]);
    USART6_Rx_ptr_in++;

    USART6_TxCount++;
  }

 USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}

void USART2_IRQHandler(void)
{
 /*   char c;
    
    buffer_select=!buffer_select; 
    
    if(USART_GetFlagStatus(USART2,USART_FLAG_TC)==SET&&uart_trans_En=1)
    {
       USART_ClearITPendingBit(USART2,USART_FLAG_TC);    
     
        if( buffer_select==1)
         {            
            USART_SendData(USART2)=uart_send_0[wireless_send_pointer];           
         }
         else        
         { 
            USART_SendData(USART2)=uart_send_1[wireless_send_pointer];
         }
    
    }
    
    if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE)==SET)
    {
       USART_ClearITPendingBit(USART2,USART_IT_RXNE);
       
        
        if( buffer_select==1)
         { 
           uart_receive_0[wireless_send_pointer]=USART_ReceiveData(USART2);
         }
         else        
         { 
           uart_receive_1[wireless_send_pointer]=USART_ReceiveData(USART2);
         }
    }
     wireless_send_pointer++;*/
        
}



void DMA2_Stream2_IRQHandler(void)//RX
{
	/*if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2)==SET)
	{
		DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);

		Flag_USART6_RX_DMA_Finish=1;
	}*/
/*	else if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_HTIF2)==SET)
	{
		DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_HTIF2);
	}
*/



}
void DMA2_Stream6_IRQHandler(void)//TX
{
	/*if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6)==SET)
	{
		DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);
		 DMA_Cmd(DMA2_Stream6, DISABLE);//stop tx
		Flag_USART6_TX_DMA_Finish2=1;
	}*/
	/*else if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_HTIF6)==SET)
	{
		DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_HTIF6);
	}*/
}
void SPI2_IRQHandler(void)
{

	return;
}


void  EXTI15_10_IRQHandler (void)
{
    if(EXTI_GetITStatus(EXTI_Line11)!= RESET)	
    {	 
	 EXTI_ClearITPendingBit(EXTI_Line11);  
         GPIO_EXTI_Callback( GPIO_Pin_11);
         
      }  
}
void EXTI0_IRQHandler(void)
{
 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
 {  EXTI_ClearITPendingBit(EXTI_Line0);
    GPIO_EXTI_Callback( GPIO_Pin_0);   
}
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
