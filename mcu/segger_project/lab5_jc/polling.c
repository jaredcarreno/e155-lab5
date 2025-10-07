// button_polling.c
// Josh Brake
// jbrake@hmc.edu
// 10/31/22

/*
  This program polls the user button on the Nucleo-L432KC board and has a
  delay within the main loop to simulate the problems with polling for 
  catching events.
*/

#include "main.h"

int main(void) {
    // Enable button as input
    gpioEnable(GPIO_PORT_A);
    pinMode(PIN_A, GPIO_INPUT);
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b10); // Set PA2 as pull-down

    // Initialize timer
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    initTIM(DELAY_TIM);

    int volatile A_curr = digitalRead(PIN_A);
    int volatile A_prev = A_curr;

    while(1){
        A_prev = A_curr;
        A_curr = digitalRead(PIN_A);
        if ((A_prev == 1) && (A_curr == 0)) {
          togglePin(PA0);
        }
        delay_millis(DELAY_TIM, 200);
    }
}