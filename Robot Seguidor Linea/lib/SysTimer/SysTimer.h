/*
 * systime.h
 *
 *  Created on: 23 ene. 2018
 *      Author: Federico Pinna(fedepinna13@gmail.com)
 */

#ifndef LIB_SYSTIME_SYSTIME_H_
#define LIB_SYSTIME_SYSTIME_H_

#include <mbed.h>

void initSystemClock();
volatile uint32_t* getPointerMillis();
void delay_ms(uint32_t ms);

#endif /* LIB_SYSTIME_SYSTIME_H_ */
