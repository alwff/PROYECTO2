
#ifndef PWM2_H_
#define PWM2_H_

#include <avr/io.h>

void Timer2_Fast_PWM_Init(void);
void PWM2_dca(uint8_t dc);
void PWM2_dcb(uint8_t dc);

#endif /* PWM2_H_ */