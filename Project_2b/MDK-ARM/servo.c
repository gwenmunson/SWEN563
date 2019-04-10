#include "timers.h"
#include "servo.h"

/*
 * SetPosition - Sets the position of the servo
 * servo - the number servo to move
 * position - the position to move the servo to
 */
void SetPosition(int servo, int position){
	if(servo == 0){
		switch(position){
			case 1:
				TIM5->CCR2 = POS1;
				break;
			case 2:
				TIM5->CCR2 = POS2;
				break;
			case 3:
				TIM5->CCR2 = POS3;
				break;
			case 4:
				TIM5->CCR2 = POS4;
				break;
			case 5:
				TIM5->CCR2 = POS5;
				break;
			default:
				TIM5->CCR2 = POS0;
		}
	TIM5->EGR |= TIM_EGR_UG;
	}
	// servo = false, servo 2
	else{
		switch(position){
			case 1:
				TIM5->CCR3 = POS1;
				break;
			case 2:
				TIM5->CCR3 = POS2;
				break;
			case 3:
				TIM5->CCR3 = POS3;
				break;
			case 4:
				TIM5->CCR3 = POS4;
				break;
			case 5:
				TIM5->CCR3 = POS5;
				break;
			default:
				TIM5->CCR3 = POS0;
		}
	TIM5->EGR |= TIM_EGR_UG;
	}
}
