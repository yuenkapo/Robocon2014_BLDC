#include "BLDC.h"
#include "systick.h"

/* Hardware 				          	| Usage
 * -----------------------------|-----------------------
 * SPI1, GPIOA 4/5/6/7			    | SPI Interface
 * GPIOB 0/1/2				        	| BLDC hall sensor
 * GPIOB 5						          | Overcurrent input
 * GPIOB 8-13				          	| BTN H-bridge Interface
 * TIM2						            	| BLDC PWM Output
 * TIM3						            	| WatchDOG Interrupt
 * TIM4						            	| PID Counter    */

#define SENSOR_A		GPIO_Pin_0
#define SENSOR_B 		GPIO_Pin_1
#define SENSOR_C 		GPIO_Pin_2
#define SENSOR_I		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)

#define A_INH		GPIO_Pin_8
#define B_INH		GPIO_Pin_10
#define C_INH		GPIO_Pin_12
#define A_IN		GPIO_Pin_9
#define B_IN		GPIO_Pin_11
#define C_IN		GPIO_Pin_13

#define MAX_PWM 100

// *********************************** Variables
// Local
volatile uint16_t INMaskPWMon;
volatile uint16_t INMaskPWMoff;
volatile bool motorPWMLow;
// Global
volatile int32_t targetPosition;
volatile fault_t safetyshutdown;
volatile bool watchdogtrap;

// *********************************** Functions
void InitPeriph(void) {
	// Init Structs
	GPIO_InitTypeDef		GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;
	SPI_InitTypeDef			SPI_InitStructure;

	// Init Clocks
	RCC_APB1PeriphClockCmd	(RCC_APB1Periph_TIM2,	ENABLE);
	RCC_APB1PeriphClockCmd	(RCC_APB1Periph_TIM3,	ENABLE);
	RCC_APB1PeriphClockCmd	(RCC_APB1Periph_TIM4,	ENABLE);
	RCC_APB2PeriphClockCmd	(RCC_APB2Periph_GPIOB,	ENABLE);
	RCC_APB2PeriphClockCmd	(RCC_APB2Periph_GPIOA,	ENABLE);
	RCC_APB2PeriphClockCmd	(RCC_APB2Periph_SPI1,	ENABLE);
	RCC_APB2PeriphClockCmd	(RCC_APB2Periph_AFIO,	ENABLE);

	// Init SPI Inputs
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Init SPI Output
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// SPI init
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 					= SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL					= SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA					= SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS						= SPI_NSS_Hard;
	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	// Init Current and Hall Sensor Input
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Init BTN Driver Control Pins
	GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Init TIM2 TimeBase: Output PWM
	TIM_TimeBaseStructure.TIM_Period		=	5000;		// Counts from 0 - 4999
	TIM_TimeBaseStructure.TIM_Prescaler		=	0; 			// Timer frequency = SysClk/(PSC+1)
	TIM_TimeBaseStructure.TIM_ClockDivision	=	0;
	TIM_TimeBaseStructure.TIM_CounterMode	=	TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	// Init Variables for PWM Generation
	INMaskPWMon = 0;
	INMaskPWMoff = 0;

	// Init TIM2 OC1
	TIM_OCInitStructure.TIM_OCMode	=	TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_Pulse	=	0;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	// Init TIM3 TimeBase: WatchDOG Interrupt
	TIM_TimeBaseStructure.TIM_Period		=	10000;		// Counts from 0 - 9999
	TIM_TimeBaseStructure.TIM_Prescaler		=	359; 		// Timer frequency = SysClk/(PSC+1)
	TIM_TimeBaseStructure.TIM_ClockDivision	=	0;
	TIM_TimeBaseStructure.TIM_CounterMode	=	TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_SetCounter(TIM3, 0);

	// Init TIM4 TimeBase: PID Counter
	TIM_TimeBaseStructure.TIM_Period		=	65535;		// Counts from 0 - 65534
	TIM_TimeBaseStructure.TIM_Prescaler		=	0; 		// Timer frequency = SysClk/(PSC+1)
	TIM_TimeBaseStructure.TIM_ClockDivision	=	0;
	TIM_TimeBaseStructure.TIM_CounterMode	=	TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// Set TIM Interrupt Sources
	TIM_ITConfig	(TIM2,	TIM_IT_Update,	ENABLE);
	TIM_ITConfig	(TIM2,	TIM_IT_CC1,		ENABLE);
	TIM_ClearFlag	(TIM2,	TIM_FLAG_CC1);
	TIM_ClearFlag	(TIM2,	TIM_FLAG_Update);
	TIM_ITConfig	(TIM3,	TIM_IT_Update,	ENABLE);
	TIM_ClearFlag	(TIM3,	TIM_FLAG_Update);

	// Config 2bits for preemption priority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Init TIM2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Init TIM3 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel						=	TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Init SPI1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel						=	SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Start TIMs
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}
state_t ReadMotorSensor(void) {
	uint16_t value = GPIO_ReadInputData(GPIOB);
	value &= SENSOR_A | SENSOR_B | SENSOR_C;
	if(value == 2) return A;
	else if(value == 5) return D;
	else if(value == 3) return B;
	else if(value == 4) return E;
	else if(value == 1) return C;
	else if(value == 6) return F;
	else return U;
}
void SetMotorState(state_t nextState) {
	switch(nextState) {
		case  A: INMaskPWMon = A_IN | A_INH | B_INH; INMaskPWMoff = A_IN | A_INH; break;
		case  D: INMaskPWMon = B_IN | B_INH | A_INH; INMaskPWMoff = B_IN | B_INH; break;
		case  B: INMaskPWMon = C_IN | C_INH | B_INH; INMaskPWMoff = C_IN | C_INH; break;
		case  E: INMaskPWMon = B_IN | B_INH | C_INH; INMaskPWMoff = B_IN | B_INH; break;
		case  C: INMaskPWMon = C_IN | C_INH | A_INH; INMaskPWMoff = C_IN | C_INH; break;
		case  F: INMaskPWMon = A_IN | A_INH | C_INH; INMaskPWMoff = A_IN | A_INH; break;
		default: INMaskPWMon = 0; INMaskPWMoff = 0; break;
	}
}
state_t RotateState(state_t oldState, dir_t direction) {
	if(oldState == U) return U;
	if(direction == CW) {
		oldState++;
		if(oldState > F) oldState = A;
	} else { //CCW
		oldState--;
		if(oldState < A) oldState = F;
	}
	return oldState;
}
void SetPWM(float PWM) {
	uint32_t value;
	if(PWM > 100) PWM = 100;
	if(PWM < 0) PWM = 0;
	value = ((float)PWM/100.0)*((float)MAX_PWM/100.0)*4999;
	if(value < 100) {
		//PWM with too low value, causing the ISR not enough time to process, will generate un expected PWM pattern
		TIM_SetCompare1(TIM2, 100);
		motorPWMLow = true;
	} else {
		TIM_SetCompare1(TIM2, value);
		motorPWMLow = false;
	}
}
int8_t DiffState(state_t newState, state_t oldState) {
	if(newState == U || oldState == U) return 0;
	if(newState == oldState) {
		return 0;
	} else if(newState > oldState) {
		if((newState - oldState) == 3) {
			return 0;
		} else if((newState - oldState) > 3) {
			return (newState - 6 - oldState);
		} else {
			return (newState - oldState);
		}
	} else {
		if((oldState - newState) == 3) {
			return 0;
		} else if((oldState - newState) > 3) {
			return (newState + 6 - oldState);
		} else {
			return (newState - oldState);
		}
	}
}
void SafetyShutdown(fault_t faulttype) {
	SetPWM(0);
	SetMotorState(U);
	safetyshutdown = faulttype;
	while(1) { }
}

// *********************************** Interrupt Handelers
void TIM2_IRQHandler(void) { //PWM Generator
	if(motorPWMLow) {
		GPIO_Write(GPIOB, 0); //Clear All
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		else //if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	} else {
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
		{	// Output Compare 1 Interrupt: PWM Off
			GPIO_Write(GPIOB, INMaskPWMoff);
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		}
		else //if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		{	// Auto-reload Interrupt: PWM On
			GPIO_Write(GPIOB, INMaskPWMon);
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		}
	}
}

void TIM3_IRQHandler(void) { //WatchDOG
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {	// Auto-reload Interrupt: WatchDOG Interrupt
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		watchdogtrap = true;
	}
}
void SPI1_IRQHandler(void) { //SPI Data in Interrupt
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET) {
		uint16_t newdata;
		newdata = ((uint16_t)SPI_I2S_ReceiveData(SPI1));
		
		if(newdata == 0x8000) { //Reset
			NVIC_SystemReset(); //Software Reset, return to initial state
		}
		
		if(safetyshutdown != NORMAL) {
			//In Safety Shutdown trap, sending error message
			if(safetyshutdown == OVERERROR) {
				SPI_I2S_SendData(SPI1, 0x6969); //Return OVERERROR Error Message
			} else if(safetyshutdown == OVERPWM) {
				SPI_I2S_SendData(SPI1, 0x5757); //Return OVERPWM Error Message
			} else {
				SPI_I2S_SendData(SPI1, 0x7878); //Return Unknown Error Message
			}
		} else {
			//Normal mode, case newdata to newvalue, keeping sign information
			int16_t newvalue = (int16_t)((int32_t)((uint32_t)newdata));
			SPI_I2S_SendData(SPI1, 0x0000); //Prevent buffered error
			if(newvalue != targetPosition) {
				targetPosition = newvalue;
			}
		}
		
	}
}
