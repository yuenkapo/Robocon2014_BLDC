#include "BLDC.h"
#include "systick.h"

#define SAFE_PWM 40
#define SAEF_ERROR 300
#define COEFFD_MAX 130

int main() {
	float pidvalue;
	float pidvalue_smoothed;
	float error_last;
	uint32_t loopcounter;
	float coeffI;
	float coeffD_smoothed;
	int32_t currentPosition;
	state_t laststate;

	InitPeriph();
	delay_init();
	
	SetMotorState(U);
	SetPWM(0);

	watchdogtrap = false;
	safetyshutdown = NORMAL;
	targetPosition = 0;
	currentPosition = 0;
	laststate = ReadMotorSensor();
	
	error_last = 0;
	loopcounter = 0;
	coeffI = 0;
	coeffD_smoothed = 0;
	pidvalue = 0;
	pidvalue_smoothed = 0;
	
	STOP_WATCHDOG;
	RESET_WATCHDOG;

	while(1) {
	//============================ENDLESSLOOP============================
		float error_current;
		state_t state_temp;

		//Read Sensor and Calculate Offset
		state_temp = ReadMotorSensor();
		if(state_temp != laststate) {
			int8_t offset = DiffState(state_temp, laststate);
			currentPosition += offset;
			laststate = state_temp;
			RESET_WATCHDOG;
		}
		
		//Collect Error Data and Time Keeping
		loopcounter += (float)TIM_GetCounter(TIM4);
		TIM_SetCounter(TIM4, 0);
		error_current = targetPosition - currentPosition;
		if(error_current > SAEF_ERROR || -error_current > SAEF_ERROR) SafetyShutdown(OVERERROR);
		
		//Calculate PID
		if(error_current != error_last || loopcounter >= 720000) {
			float coeffD;
			float looptime =  ((float)loopcounter) / 36000000.0;
			coeffI += error_current*looptime;
			coeffD = (error_current - error_last)/looptime;
			if(coeffD > COEFFD_MAX) coeffD = COEFFD_MAX;
			if(coeffD < -COEFFD_MAX) coeffD = -COEFFD_MAX;
			coeffD_smoothed = coeffD_smoothed * 0.99 + coeffD * 0.01;
			pidvalue = 0.35*error_current + 1.2*coeffI + 0.6*coeffD_smoothed; //PID calculation
			loopcounter = 0;
			error_last = error_current;
		}

		//Smoothing PID value
		pidvalue_smoothed = pidvalue_smoothed*0.95 + pidvalue*0.05;
		
		//Check WatchDog Trap
		if(watchdogtrap) SafetyShutdown(OVERPWM);
		
		//Get amplitude and direction from PID and Set motor
		if(pidvalue_smoothed >= 0) {
			if(pidvalue_smoothed > 100) pidvalue_smoothed = 100;
			SetMotorState(RotateState(RotateState(state_temp, CW), CW));
			SetPWM(pidvalue_smoothed);
			if(pidvalue_smoothed > SAFE_PWM) {
				START_WATCHDOG;
			} else {
				STOP_WATCHDOG;
				RESET_WATCHDOG;
			}
		} else {
			if(pidvalue_smoothed < -100) pidvalue_smoothed = -100;
			SetMotorState(RotateState(RotateState(state_temp, CCW), CCW));
			SetPWM(-pidvalue_smoothed);
			if(-pidvalue_smoothed > SAFE_PWM) {
				START_WATCHDOG;
			} else {
				STOP_WATCHDOG;
				RESET_WATCHDOG;
			}
		}
	//============================ENDLESSLOOP============================
	}
}
