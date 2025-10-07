// ======================================================================
// ============================================================================
// main.c
// Quadrature encoder using EXTI interrupts + UART printf + direction
// ============================================================================

#include "main.h"

volatile int32_t encoder_count = 0;
volatile int32_t last_encoder_count = 0;
volatile float   encoder_velocity = 0.0f;
volatile int8_t  encoder_dir = 0;       // 1 = CW, -1 = CCW, 0 = stopped
volatile uint32_t print_counter = 0;

// ---------------------------------------------------------------------------
// UART2 initialization and printf redirection
// ---------------------------------------------------------------------------
void UART2_Init(void)
{
    // Enable GPIOA and USART2 clocks
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // Configure PA2 (TX) and PA3 (RX) as AF7
    GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
    GPIOA->MODER |=  (0x2 << GPIO_MODER_MODE2_Pos) | (0x2 << GPIO_MODER_MODE3_Pos);
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOA->AFR[0] |=  (7 << (2 * 4)) | (7 << (3 * 4));  // AF7 = USART2

    // Configure USART2: 115200 baud, 8N1
    USART2->BRR = SystemCoreClock / 115200; // assuming APB1 = SystemCoreClock
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

// Retarget printf to USART2
int __io_putchar(int ch)
{
    while (!(USART2->ISR & USART_ISR_TXE)); // wait for TX buffer empty
    USART2->TDR = ch;
    return ch;
}

// ---------------------------------------------------------------------------
// Encoder + EXTI setup
// ---------------------------------------------------------------------------
void Encoder_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // PA1 and PA2 as input with pull-ups
    GPIOA->MODER &= ~(GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD1_Msk | GPIO_PUPDR_PUPD2_Msk);
    GPIOA->PUPDR |=  (1U << GPIO_PUPDR_PUPD1_Pos) | (1U << GPIO_PUPDR_PUPD2_Pos);

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Map EXTI lines to PA1, PA2 (EXTI1, EXTI2)
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI1_Msk | SYSCFG_EXTICR1_EXTI2_Msk);

    EXTI->IMR1  |= (EXTI_IMR1_IM1 | EXTI_IMR1_IM2);
    EXTI->RTSR1 |= (EXTI_RTSR1_RT1 | EXTI_RTSR1_RT2);
    EXTI->FTSR1 |= (EXTI_FTSR1_FT1 | EXTI_FTSR1_FT2);

    NVIC_SetPriority(EXTI1_IRQn, 2);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI2_IRQn, 2);
    NVIC_EnableIRQ(EXTI2_IRQn);
}

// ---------------------------------------------------------------------------
// TIM2 1 ms periodic interrupt
// ---------------------------------------------------------------------------
void Timer2_Init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    DELAY_TIM->PSC = (SystemCoreClock / 1000000) - 1; // 1 MHz tick
    DELAY_TIM->ARR = 1000 - 1;                        // 1 ms overflow
    DELAY_TIM->CNT = 0;
    DELAY_TIM->DIER |= TIM_DIER_UIE;
    DELAY_TIM->CR1  |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);
}

// ---------------------------------------------------------------------------
// Compute velocity (called every 1 ms by TIM2 interrupt)
// ---------------------------------------------------------------------------
void compute_velocity(void)
{
    int32_t delta = encoder_count - last_encoder_count;
    last_encoder_count = encoder_count;

    // Determine direction based on delta sign
    if (delta > 0)      encoder_dir = 1;
    else if (delta < 0) encoder_dir = -1;
    else                encoder_dir = 0;

    // rev/s = delta / (PPR * 4 * 0.001s)
    encoder_velocity = (float)delta / (PPR * 4.0f * 0.001f);
}

// ---------------------------------------------------------------------------
// Encoder update helper
// ---------------------------------------------------------------------------
static inline void Encoder_Update(void)
{
    uint32_t A = (GPIOA->IDR & (1 << 1)) ? 1 : 0;
    uint32_t B = (GPIOA->IDR & (1 << 2)) ? 1 : 0;

    if (A == B)
        encoder_count++;
    else
        encoder_count--;
}

// ---------------------------------------------------------------------------
// Interrupt Handlers
// ---------------------------------------------------------------------------
void EXTI1_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF1)
    {
        EXTI->PR1 = EXTI_PR1_PIF1;
        Encoder_Update();
    }
}

void EXTI2_IRQHandler(void)
{
    if (EXTI->PR1 & EXTI_PR1_PIF2)
    {
        EXTI->PR1 = EXTI_PR1_PIF2;
        Encoder_Update();
    }
}

void TIM2_IRQHandler(void)
{
    if (DELAY_TIM->SR & TIM_SR_UIF)
    {
        DELAY_TIM->SR &= ~TIM_SR_UIF;

        compute_velocity();

        // Print every 100 ms
        print_counter++;
        if (print_counter >= 100)
        {
            print_counter = 0;

            if (encoder_dir == 0)
                printf("Stopped (0.00 rev/s)\r\n");
            else
                printf("Speed: %.2f rev/s (%s)\r\n",
                       encoder_velocity * encoder_dir,   // signed speed
                       (encoder_dir > 0) ? "CW" : "CCW");
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void)
{
    SystemCoreClockUpdate();
    UART2_Init();
    Encoder_Init();
    Timer2_Init();

    printf("\r\nQuadrature Encoder + Direction Test Start\r\n");

    while (1)
    {
        __WFI(); // Sleep until interrupt
    }
}
