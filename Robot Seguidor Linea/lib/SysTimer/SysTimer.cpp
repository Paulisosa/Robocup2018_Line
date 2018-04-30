/*
 * systime..cpp
 *
 *  Created on: 23 ene. 2018
 *      Author: Federico Pinna(fedepinna13@gmail.com)
 */
#include "SysTimer.h"

volatile uint32_t _millis;

extern "C" void SysTick_Handler(void) {
    _millis++;
}

void initSystemClock(){
	SysTick_Config(SystemCoreClock / 1000);
}

volatile uint32_t* getPointerMillis(){
	return &_millis;
}

void delay_ms(uint32_t ms){
	uint32_t ts=_millis;
	while((_millis-ts)<=ms);
}


