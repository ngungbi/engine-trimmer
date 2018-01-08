#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "adc.h"

#include "config.h"

//volatile uint8_t timeL = 0;
volatile uint8_t timerH = 0;
volatile uint16_t time0 = 0;
volatile uint16_t time1 = 0;

uint16_t trimLow = 0;
uint16_t trimHigh = 0;

volatile uint16_t pwmIn = 2000;
volatile uint16_t pwmOut = 2000;
//volatile uint8_t timeout = 0;

// manual PWM, adakah cara yang lebih baik? ._.
ISR(TIMER1_OVF_vect){
	PORTB |= (1<<2);
}
ISR(TIMER1_COMPA_vect){
	PORTB &= ~(1<<2);
}

// pwnhitung pwm input
ISR(TIMER0_OVF_vect){
	timerH++;
}
ISR(INT0_vect){
	uint16_t timeNow = ((uint16_t)timerH<<8) | TCNT0;
	//PORTB = (~PORTB)&0x07;
	if( PIND&(1<<2) ){ // awal sinyal pwm
		PORTB |= (1<<1);
		time0 = timeNow;
	}else{ // akhir sinyal pwm
		PORTB &= ~(1<<1);
		time1 = timeNow;
		pwmIn = (time1-time0);
		if(pwmIn<2000) pwmIn = 2000;
		else if(pwmIn>4000) pwmIn = 4000;
		//OCR1A = pwmIn;
	}
}

int main( void ){

	DDRB = 0x00;
	DDRC = 0x00;

	// input di PD2 dan PD3 (INT0, INT1)
	DDRD = 0x00;

	// output di PB1 dan PB2 (OCR1A, OCR1B
	DDRB = (1<<1) | (1<<2);

	// pakai timer0 untuk hitung pwm input
	TCCR0=(0<<CS02) | (1<<CS01) | (0<<CS00);
	TCNT0=0x00;

	// manual pwm untuk output
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x4E;
	ICR1L=0x20;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

/*
	// PWM Servo, PD3
	DDRB |= (1<<3);
	TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (1<<WGM20);
	TCCR2B = (0<<WGM22) | (1<<CS22) | (0<<CS21) | (1<<CS20);
	OCR2A = 130;
*/
	// Tachometer, PD2
//	PORTD = (1<<2);
//	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
//	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);

	// external interrupt di INT0 untuk baca pwm
	//PORTD |= 0x02;
	GICR|=(0<<INT1) | (1<<INT0);
	MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (1<<ISC00);
	GIFR=(0<<INTF1) | (1<<INTF0);
	
	TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (1<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (1<<TOIE0);

	ADC_Init();

	sei();
 	OCR1A = 2000;
	while(1){
		trimLow  = ADC_Read(0)>>1;
		trimHigh = ADC_Read(1)>>1;
		uint16_t range = 2000-trimHigh-trimLow;
		float value = (float)(pwmIn-2000)/2000.0 * range;
		pwmOut = (uint16_t)value + 2000 + trimLow;
		OCR1A = pwmOut;
		_delay_ms(10);
	}
}