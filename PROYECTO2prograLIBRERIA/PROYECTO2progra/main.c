/* Universidad del Valle de Guatemala
IE2023:: Programación de Microcontroladores
PROYECTO2progra.c
Autor: Alejandra Cardona
Hardware: ATMEGA328P
Creado: 08/04/2024
Última modificación: 09/04/2024

****************************************************************** */

/* LIBRERÍAS

****************************************************************** */


#define F_CPU 16000000 //Frecuencia en la que opera el sistema - 16 MHz

#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "PWM1/PWM1.h"
#include "PWM2/PWM2.h"
#include <avr/eeprom.h> //Libreria EEPROM

//A4 y A5 LEDS
#define LED1 PC4 // Izquierda
#define LED2 PC5 // Derecha

/* ******************************************************************

VARIABLES

Variable: Tipo nombre = Valor

****************************************************************** */

//uint16_t valueADCforPot = 0;

//Botones
int boton1 = 0; //selector de modo
int boton2 = 0; //reproductor
int boton3 = 0; //grabador

//Valores de ADC para cada canal
float canal0 = 0;
float canal1 = 0;
float canal2 = 0;
float canal3 = 0;

//UART
volatile char bufferRX; //Es volatil porque va a estar dentro de la interrupcion, pues puede cambiar en todo momento
volatile char bufferRX_copia;
char bufferpp[10];  
int convertido = 0;
int UARTyes = 0;

//EEPROM
int buffer1 = 0; //VALOR PWM SERVO1
int buffer2 = 0; //VALOR PWM SERVO2
int buffer3 = 0; //VALOR PWM DC1
int buffer4 = 0; //VALOR PWM DC2

//Almacenar los valores de EEPROM para reproducir las posiciones
int valueEPROM1 = 0; 
int valueEPROM2 = 0; 
int valueEPROM3 = 0;
int valueEPROM4 = 0;

int grabar = 0;
int reproducir = 0;

/* ******************************************************************

FUNCIONES

****************************************************************** */

void setup(void);
void modes(void);
void initADC(void);
void ADCselector(void);
uint16_t selector(uint8_t);
void initUART09600(void);
void writeUART(char character); //Funcion para escribir
void writeTextUART(char* text); //Puntero
int intforport(char bufferRX); //Convertir de char a int

/* ******************************************************************

CÓDIGO

****************************************************************** */

int main(void){ //Función main
	
	cli();  //Deshabilita interrupciones
	setup();
	initADC();
	PWM1_init();
	Timer2_Fast_PWM_Init();
	initUART09600();
	sei(); // Habilita las interrupciones 
	
	//LOOP 
	while (1){
		modes();
	}
	
}

void setup(void){ //Se utiliza void cuando no se emplean parámetros
		
	//DDRB |= (1 << PORTB2) | (1 << PORTB1);		//PB2 y PB1 como salida (OC0A and OCR0B) - para servo

	//Pines PB0, PB4 y PB5 como entradas
	DDRB &= ~(1 << PINB0) | ~(1 << PINB4) | ~(1 << PINB5);
	PORTB |= (1 << PINB0) | (1 << PINB4) | (1 << PINB5);

	//DDRC = 0; //Puerto C como entradas
	
	//Puerto C (PC0, PC1, PC2, PC3) como entradas
	DDRC &= ~(1 << PINC0) | ~(1 << PINC1) | ~(1 << PINC2) | ~(1 << PINC3);
	PORTC |= (1 << PINC0) | (1 << PINC1) | (1 << PINC2) | (1 << PINC3);
	
	//PC4 Y 5 como salida
	DDRC |= (1 << PC4) | (1 << PC5);
	//PORTD &= ~(1 << PC4) | (1 << PC5);
	
	
	// Puerto D como salidas
	DDRD = 0xFF;	
	
	// Interrupción
	PCICR |= (1<<PCIE0); // PCIE0 para puerto B
	PCMSK0 |= ((1<<PCINT0)|(1<<PCINT4)|(1<<PCINT5)); // Habilita la interrupción en el puerto C -- PCMSK0 corresponde al puerto B -- PCINT0-PB0, PCINT4-PB4 y PCINT5-PB5
	
}

void modes(void){ //Seleccion de modo
	
	switch (boton1)
	{
//------------------------------------------CASE0--------------------------------------------------------------------

		case 0: //Modo de control manual a través de potenciometros y guardado para EEPROM
		
		//Marcador del modo LED
		PORTC |= (1 << LED2);
		PORTC &= ~(1 << LED1);		
		
		ADCselector(); //Selecciona el canal de ADC para leer los potenciometros
		
		if (grabar==1){ //Pot 1 - Servo 1
			buffer1 = canal0;
			eeprom_write_byte((uint8_t*)boton2, buffer1);
			
			//Pot 2 - Servo 2
			buffer2 = canal1;
			int temporalvaluebotonx1= (boton2+10);
			eeprom_write_byte((uint8_t*)temporalvaluebotonx1, buffer2);
			
			//Pot 3 - DC 1
			buffer3 = canal2;
			int temporalvaluebotonx2= (boton2+20);
			eeprom_write_byte((uint8_t*)temporalvaluebotonx2, buffer3);
			
			//Pot 4 - DC 2
			buffer4 = canal3;
			int temporalvaluebotonx3= (boton2+30);
			eeprom_write_byte((uint8_t*)temporalvaluebotonx3, buffer4);
			
			grabar = 0;
			
			sprintf(bufferpp, "%d", buffer1);
			writeTextUART("canal 0: ");
			writeTextUART(bufferpp);
			writeTextUART("\n");
			
			sprintf(bufferpp, "%d", buffer2);
			writeTextUART("canal 1: ");
			writeTextUART(bufferpp);
			writeTextUART("\n");
			
			sprintf(bufferpp, "%d", buffer3);
			writeTextUART("canal 2: ");
			writeTextUART(bufferpp);
			writeTextUART("\n");
			
			sprintf(bufferpp, "%d", buffer4);
			writeTextUART("canal 3: ");
			writeTextUART(bufferpp);
			writeTextUART("\n");
		}
		
		break;
		
//------------------------------------------CASE1--------------------------------------------------------------------	
	
		case 1: //Modo de memoria EEPROM
		
		//Marcador del modo LED
		PORTC |= (1 << LED1);
		PORTC &= ~(1 << LED2);		
			
		if (reproducir==1){//Pot 1 - Servo 1
		valueEPROM1 =  eeprom_read_byte((uint8_t*)boton3);
		
		//Pot 2 - Servo 2
		int temporalvalueboton1= (boton3+10);
		valueEPROM2 = eeprom_read_byte((uint8_t*)temporalvalueboton1);
		
		//Pot 3 - DC 1
		int temporalvalueboton2= (boton3+20);
		valueEPROM3 = eeprom_read_byte((uint8_t*)temporalvalueboton2);
		
		//Pot 4 - DC 2
		int temporalvalueboton3= (boton3+30);
		valueEPROM4 = eeprom_read_byte((uint8_t*)temporalvalueboton3);
		
		sprintf(bufferpp, "%d", valueEPROM1);
		writeTextUART("eep1: ");
		writeTextUART(bufferpp);
		writeTextUART("\n");
		
		sprintf(bufferpp, "%d", valueEPROM2);
		writeTextUART("Valor de variable: ");
		writeTextUART(bufferpp);
		writeTextUART("\n");
		
		sprintf(bufferpp, "%d", valueEPROM3);
		writeTextUART("Valor de variable: ");
		writeTextUART(bufferpp);
		writeTextUART("\n");
		
		sprintf(bufferpp, "%d", valueEPROM4);
		writeTextUART("Valor de variable: ");
		writeTextUART(bufferpp);
		writeTextUART("\n");
		
		reproducir = 0;
		}

				
		//Servo segun el valor guardado en la EEPROM
		servo_writeA(valueEPROM1); // Mover el servo a la posición 
		servo_writeB(valueEPROM2); // Mover el servo a la posición 
		
		//DC motor segun el valor guardado en la EEPROM
		PWM2_dca(valueEPROM3);
		PWM2_dcb(valueEPROM4);
		
		break;

//------------------------------------------CASE2--------------------------------------------------------------------
	
		case 2: //Modo de control UART a través de ADAFRUIT
		PORTC |= (1 << LED1);
		PORTC |= (1 << LED2);
		
		//UARTyes = 1;
		
		/*
		writeTextUART("Inserte un valor inferior a 255 para los motores");		
		if ((convertido<=255)&&(convertido>=0)){
			convertServo(convertido, channelA); //OCR1A
			convertServo(convertido, channelB); //OCR1B
		}
		else{
			writeTextUART("Error, valor fuera del parámetro permitido!");
		}
		*/
		
		break;

//------------------------------------------DEFAULT--------------------------------------------------------------------

		default:
		boton1=0; //El valor predefinido es de 0
	}	
}

/* ******************************************************************

Interrupciones

****************************************************************** */

//BOTONES
ISR(PCINT0_vect){
	
	if (!(PINB & (1 << PINB0))) { 
		// Cuando botón 0 presionado:
		boton1++;
		
		}
	else if (!(PINB & (1 << PINB4))) { //BOTON DE GUARDADO
		// Cuando botón 1 presionado:
		
		if (boton2<=4) {
			grabar = 1;
		}
		
		else {
			boton2=0;
		}
		
		boton2++;
		
	}
	else if (!(PINB & (1 << PINB5))) { //BOTON DE REPRODUCCION
		// Cuando botón 2 presionado:
		
		if (boton3<=4) {
			reproducir=1;
		}
		
		else {
			boton3=0;
		}
		
		boton3++;
	}
	
	
}

//UART
ISR(USART_RX_vect){
	//Actuando como un eco, lo que se envia, regresa lo  mismo
	bufferRX = UDR0;
	while(!(UCSR0A&(1<<UDRE0))); //Si esto esta vacio entonces se manda
	UDR0 = bufferRX; //Mete a UDR0 lo que esta en el buffer para enviarlo
	//bufferRX_copia = bufferRX;
	//convertido = intforport(bufferRX_copia);
	
	}

/* ******************************************************************

ADC

****************************************************************** */

void initADC(void){ 
	// Configurando bits de ADC
	ADMUX |= (1<<REFS0);	// VCC REF
	ADMUX &= ~(1<<REFS1);
	ADMUX &= ~(1<<ADLAR);	// 10 bits
	// PRESCALER 128 > 16M/128 = 125KHz
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRA |= (1<<ADEN);	// ADC ON

}

uint16_t selector(uint8_t ADCcanal){
	ADMUX = (ADMUX&0xF0)|ADCcanal; //ADMUX en el canal seleccionado
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1 << ADSC));  // Esperar a que termine la conversión
	return(ADC);
}

void ADCselector(void){
	canal0 = selector(0); //Entra al canal 0
	servo_writeA(canal0); //OCR1A
	_delay_ms(5);
	
	canal1 = selector(1); //Entra al canal 1
	servo_writeB(canal1); //OCR1B
	_delay_ms(5);
	
	canal2 = selector(2);
	canal3 = selector(3);
	
	if (canal2 <= 100){
		PWM2_dca(0);
	} else if(canal3 <= 100) {
		PWM2_dca(canal2);
		PWM2_dcb(0);
	}
	
	
	if (canal3 <= 100){
		PWM2_dcb(0);
		} else if (canal2 <= 100){
		PWM2_dcb(canal3);
		PWM2_dca(0);
	}

}

/* ******************************************************************

UART

****************************************************************** */

void initUART09600(void){
	//Configurando los pines RX y TX
	DDRD &= ~(1<<DDD0); //D0 RX como entrada
	DDRD |= (1<<DDD1); //D1 TX como salida
	
	//Se define el modo de trabajo
	UCSR0A = 0;
	UCSR0A |= (1<<U2X0); // Usando el registro de control A. Se configura en Modo Fast U2X0 = 1
	
	//Se configura el registro de control B
	UCSR0B = 0;
	UCSR0B |= (1<<RXCIE0); // Se habilita la interrupcion ISR RX
	UCSR0B |= (1<<RXEN0); // Se habilita RX
	UCSR0B |= (1<<TXEN0); // Se habilita TX
	
	//Se configura el registro de control C
	//Se define el frame
	UCSR0C = 0;
	UCSR0C = (1<<UCSZ01)| (1<<UCSZ00); // Define el numero de bits de data en el frame del receiver y el transmitter
	
	//Calculos
	//UBRRn=(fosc/mhz*BAUD)-1
	//BAUD=(fosc/mhz*(UBRRn+1))
	//Error%=((BAUDcalculado/BAUD)-1)*100%
	//Usando fosc de 16 MHZ
	//UBRRn = 207
	//BAUDRATE = 9600

	UBRR0 = 207;
}

//Escritura - Verifica si el UDREn buffer esta vacio
void writeUART(char character){
	while(!(UCSR0A&(1<<UDRE0))); //Si esto esta vacio entonces se manda
	//Mientras while no sea cierto, y como esta vacio simplemente se atora en el while hasta que se cumpla la condicion
	UDR0 = character;
}

void writeTextUART(char* text){
	uint8_t i;
	for (i=0; text[i]!='\0'; i++){ //Inicia en 0 y para cuando sea nulo \0
		while(!(UCSR0A&(1<<UDRE0)));
		UDR0 = text[i]; //Barre todo el arreglo hasta acabar
	}
	
}

//Convierte lo recibido por el buffer en un entero para poder leerlo
int intforport(char bufferRX_copia){
	return bufferRX_copia - '0';
}

/* ******************************************************************

EEPROM 

****************************************************************** */

//eeprom_read_byte((uint8_t*)buffer1); //
//eeprom_write_byte((uint8_t*)uiAddress,buffer1); //


