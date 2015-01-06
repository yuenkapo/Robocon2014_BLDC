#ifndef BLDC_H_
#define BLDC_H_

// *********************************** Includes
#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// *********************************** Types
//Motor Direction
typedef enum{CW = 1, CCW = 2} dir_t;

//Motor States
typedef enum{U = 0, A = 1, B = 2, C = 3, D = 4, E = 5, F = 6} state_t; //A->B->C->D->E->F is CW, U is unknown (maybe sensor not connected)

//Fault Types
typedef enum{UNKNOWN = -1, NORMAL = 0, OVERERROR = 1, OVERPWM = 2} fault_t; //OVERERROR means error > SAEF_ERROR, OVERPWM means pwm output > SAFE_PWM continuesly for watchdog time

// *********************************** Function Prototypes
void InitPeriph(void);
state_t ReadMotorSensor(void);
void SetMotorState(state_t nextState);
state_t RotateState(state_t oldState, dir_t direction);
void SetPWM(float PWM);
int8_t DiffState(state_t newState, state_t oldState);
void SafetyShutdown(fault_t);

// *********************************** Variables
// Global
extern volatile int32_t targetPosition;
extern volatile fault_t safetyshutdown;
extern volatile bool watchdogtrap;

// *********************************** Macros
#define RESET_WATCHDOG	TIM_SetCounter(TIM3, 0)
#define START_WATCHDOG	TIM_Cmd(TIM3, ENABLE)
#define STOP_WATCHDOG	TIM_Cmd(TIM3, DISABLE)


#endif /* BLDC_H_ */
