// main.h
// Jared Carreno
// jcarreno@hmc.edu
// 10/4/2025

#ifndef MAIN_H
#define MAIN_H

#include "STM32L432KC.h"
#include <stm32l432xx.h>

///////////////////////////////////////////////////////////////////////////////
// Custom defines
///////////////////////////////////////////////////////////////////////////////

#define PIN_A PA1 // rated for 5V
#define PIN_B PA2 // rated for 5V
#define DELAY_TIM TIM2
#define PPR 408

void compute_velocity(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);

#endif // MAIN_H