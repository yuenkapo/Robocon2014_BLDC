/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               systick.c
** Descriptions:            Using SysTick to implement simple delay
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-10-30
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "systick.h"
#include "stm32f10x_systick.h"

/* Private variables ---------------------------------------------------------*/	 
static uint8_t  delay_fac_us = 0;  /* us delay multiplier */
static uint16_t delay_fac_ms = 0;  /* ms delay multiplier */
static FlagStatus  Status;


/*******************************************************************************
* Function Name  : delay_init
* Description    : Initialize Delay Functions
* Input          : None
* Output         : None
* Return         : None
* Attention		 : SYSTICK's clock is fixed as 1/8 HCLK
*******************************************************************************/
void delay_init(void)
{
    RCC_ClocksTypeDef RCC_ClocksStatus;

    RCC_GetClocksFreq(&RCC_ClocksStatus);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  /*Select externel clock HCLK/8 */
	SysTick_ITConfig(DISABLE);
    delay_fac_us = RCC_ClocksStatus.HCLK_Frequency / 8000000;
    delay_fac_ms = RCC_ClocksStatus.HCLK_Frequency / 8000;      
}
					
/*******************************************************************************
* Function Name  : delay_us
* Description    : Microsecond Delay 
* Input          : Nus, N microseconds
* Output         : None
* Return         : None
* Attention		 : Maximum value: 0xffffff / (HCLK / 8000000)
*******************************************************************************/            
void delay_us(u32 Nus)
{ 
    SysTick_SetReload(delay_fac_us * Nus);
    SysTick_CounterCmd(SysTick_Counter_Clear);
    SysTick_CounterCmd(SysTick_Counter_Enable);
    do
    {
        Status = SysTick_GetFlagStatus(SysTick_FLAG_COUNT);
    }while (Status != SET);
    SysTick_CounterCmd(SysTick_Counter_Disable);
	SysTick_CounterCmd(SysTick_Counter_Clear);
}


/*******************************************************************************
* Function Name  : delay_ms
* Description    : Milisecond Delay 
* Input          : nms, n miliseconds
* Output         : None
* Return         : None
* Attention		 : Maximum value: nms<=0xffffff*8*1000/SYSCLK, while using 72 MHz,nms<=1864 
*******************************************************************************/  
void delay_ms(uint16_t nms)
{    
    uint32_t temp = delay_fac_ms * nms;

    if (temp > 0x00ffffff)
    {
        temp = 0x00ffffff;
    }
    SysTick_SetReload(temp);
    SysTick_CounterCmd(SysTick_Counter_Clear);
    SysTick_CounterCmd(SysTick_Counter_Enable);
    do
    {
        Status = SysTick_GetFlagStatus(SysTick_FLAG_COUNT);
    }while (Status != SET);
    SysTick_CounterCmd(SysTick_Counter_Disable);
	SysTick_CounterCmd(SysTick_Counter_Clear);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
















