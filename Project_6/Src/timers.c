#include "timers.h"
#include "servo.h"
#include <string.h>

/*
 * Timer2_init() - initializes timer registers for Timer2 counting
 */
void Timer2_init()
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //enable the clock of TIM2

    // Timer configuration
    TIM2->PSC |= 0x1F3F;
	TIM2->ARR = 1000;
    TIM2->EGR |= TIM_EGR_UG; //update generator

	TIM2->CR1 |= TIM_CR1_DIR;
	TIM2->DIER |= TIM_DIER_UIE;
    TIM2->EGR |= TIM_EGR_UG; //update generator
    NVIC_EnableIRQ(TIM2_IRQn);
}

/*
 * Timer5_init() - initializes timer registers for Timer5 PWM generation
 */
/*
void Timer5_init(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

	TIM5->PSC |= 0x1F3F;
	//TIM5->EGR |= TIM_EGR_UG;

	TIM5->CCMR1 &= ~(0x03 << 8);
    TIM5->CCMR2 &= ~(0x03 << 0);
    TIM5->CCMR1 &= ~(0x07 << 12);
	TIM5->CCMR2 &= ~(0x07 << 12);
    TIM5->CCMR1 |= (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
	TIM5->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);

	TIM5->CR1 |= TIM_CR1_ARPE;
	TIM5->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3E);

	TIM5->ARR = 200;

	TIM5->CCR2 = 14;
	TIM5->CCR3 = 14;

	TIM5->EGR |= TIM_EGR_UG;

	//TIM5->CR1 |= TIM_CR1_CEN;
}*/
