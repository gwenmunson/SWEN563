#include "timers.h"
#include "servo.h"

/*
 * SetPosition - Sets the position of the servo
 * servo - the number servo to move
 * position - the position to move the servo to
 */
void SetPosition(int servo, int position){
	if(servo == 0){
		TIM5->CCR1 = position;
		TIM5->EGR |= TIM_EGR_UG;
	}
	else{
		TIM5->CCR2 = position;
		TIM5->EGR |= TIM_EGR_UG;
	}
}
