// main.h
// Jared Carreno
// jcarreno@hmc.edu
// 10/4/2025

#include "main.h"

int32_t counter;


int main(void) {
  gpioEnable(GPIO_PORT_A);
  pinMode(PIN_A, GPIO_INPUT);
  pinMode(PIN_B, GPIO_INPUT);
  GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD1, 0b10); // Set PA1 as pull-down
  GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b10); // Set PA2 as pull-down

  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

  initTIM(DELAY_TIM);

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock domain in RCC

  SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI1, 0b000); // Select PA1
  SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI2, 0b000); // Select PA2

  __enable_irq(); // enable interrupts globally

//////////////////////////
//// Configuring interrupt
//////////////////////////

  // 1. Configure mask bit
  EXTI->IMR1 |= (1 << gpioPinOffset(PIN_A)); // Configure the mask bit
  EXTI->IMR1 |= (1 << gpioPinOffset(PIN_B)); // Configure the mask bit

  // 2. Enable rising edge trigger
  EXTI->RTSR1 |= (1 << gpioPinOffset(PIN_A)); // Enable rising edge trigger
  EXTI->RTSR1 |= (1 << gpioPinOffset(PIN_B)); // Enable rising edge trigger

  // 3. Enable falling edge trigger
  EXTI->FTSR1 |= (1 << gpioPinOffset(PIN_A));// Enable falling edge trigger
  EXTI->FTSR1 |= (1 << gpioPinOffset(PIN_B));// Enable falling edge trigger

  // 4. Turn on EXTI interrupt in NVIC_ISER
  NVIC->ISER[0] |= (1 << EXTI1_IRQn); // EXTI interrupt for PA1
  NVIC->ISER[0] |= (1 << EXTI2_IRQn); // EXTI interrupt for PA2

  while(1){
    compute_velocity();
    delay_millis(TIM2, 1000);
  }

} 

void EXTI1_IRQHandler(void) {
  //if (EXTI->PR1 & (1 << gpioPinOffset(PIN_A))) {
    
  //}

  EXTI->PR1 |= (1 << gpioPinOffset(PIN_A));

    if(digitalRead(PIN_A) == digitalRead(PIN_B)) {
      counter--;    // CW rising edge on B when A is high
    }
    
    else {
      counter++;    // CCW rising edge on B when A is low  
    }

}

void EXTI2_IRQHandler(void) {
  //if (EXTI->PR1 & (1 << gpioPinOffset(PIN_B))) {
    
  //}
  EXTI->PR1 |= (1 << gpioPinOffset(PIN_B));

    if(digitalRead(PIN_A) == digitalRead(PIN_B)) {
      counter++;    // CW rising edge on B when A is high
    }
    
    else {
      counter--;    // CCW rising edge on B when A is low  
    }
    
}

 void compute_velocity(void) { 
  double velocity = ((double)counter)/(4.0*PPR);
  printf("counter: %i \n", counter);
  counter = 0;
  
  printf("The motor is spinning at %f rev/s \n", velocity);
}