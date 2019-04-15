#ifndef __STM32L476G_DISCOVERY_SERVO_H
#define __STM32L476G_DISCOVERY_SERVO_H

#define POS0 (5) //position 0, ~0.4ms
#define POS1 (7)
#define POS2 (10)
#define POS3 (14)
#define POS4 (17)
#define POS5 (20) //position 5, ~2.0ms

void SetPosition(int servo, int position);

#endif
