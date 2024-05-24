#include "PWM2.h"
//Configuración del timer2 tomada de IA, chatgpt.

void Timer2_Fast_PWM_Init(void) {
	// Configurar los pines PD3 y PB3 como salidas
	DDRD |= (1 << PD3);
	DDRB |= (1 << PB3);

	// Configurar el Timer/Counter2 para Fast PWM
	TCCR2A = 0;
	TCCR2A |= (1 << WGM21) | (1 << WGM20);			// Modo Fast PWM
	TCCR2A |= (1 << COM2A1);						// Clear OC2A on Compare Match, set OC2A at BOTTOM (non-inverting mode)
	TCCR2A |= (1 << COM2B1);						// Clear OC2B on Compare Match, set OC2B at BOTTOM (non-inverting mode)

	// Configurar el prescaler a 1024
	TCCR2B =0;
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

	// Inicializar los registros de comparación de salida a 0
	OCR2A = 0x00;
	OCR2B = 0x00;
	
}

void PWM2_dca(uint8_t dc)
{
	OCR2A = dc;
}

void PWM2_dcb(uint8_t dc)
{
	OCR2B = dc; 
	
}

