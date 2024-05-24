/* Universidad del Valle de Guatemala
IE2023:: Programación de Microcontroladores
PWM1.h
Hardware: ATMEGA328P

****************************************************************** */


#ifndef PWM1_H_
#define PWM1_H_

#include <avr/io.h>
#include <stdint.h>

/*
#define yes 1
#define nop 0
#define normal 1
#define settedUp 2
#define channelA 1
#define channelB 2

//Use it before setChannel();
//Settings for Fast PWM 1, 16 bits
//(modePWM <- normal/settedUp, prescaler <- 1,8,64,256,1024)

void initFastPWM1(uint8_t modePWM, uint16_t prescaler);

//(channel <- channelA/channelB, inverted <- yes/nop)
void channel(uint8_t setChannel, uint8_t inverted);

//topValue, used if mode is settedUp
void topValue(uint16_t top);

//conversion for servos, mapping values
//(analogIN = ADCH, channelA/channelB)
void convertServo(uint16_t analogIn, uint8_t selChannel);

*/

void PWM1_init(void);
void servo_writeA(float valADC);
void servo_writeB(float valADC);
float map(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* PWM1_H_ */