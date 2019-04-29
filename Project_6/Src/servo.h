#ifndef __STM32L476G_DISCOVERY_SERVO_H
#define __STM32L476G_DISCOVERY_SERVO_H

#define COMPUTER_SERVO 0
#define PLAYER_SERVO 1

#define MIN_SERVO_PWM (500) //position 0, ~0.5ms
#define MAX_SERVO_PWM (2000) //position 5, ~2.0ms

#define POS0 (500) //position 0, ~0.4ms
#define POS1 (700)
#define POS2 (1000)
#define POS3 (1400)
#define POS4 (1700)
#define POS5 (2000) //position 5, ~2.0ms
void SetPosition(int servo, int position);

#endif
