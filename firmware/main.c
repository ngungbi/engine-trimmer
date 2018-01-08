#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "adc.h"

#include "config.h"

//volatile uint8_t timerL = 0;
volatile uint8_t timerH = 0;
volatile uint16_t time0 = 0;
volatile uint16_t time1 = 0;

volatile uint16_t trimLow = 0;
volatile uint16_t trimHigh = 0;
volatile uint16_t range = 1000;

volatile uint16_t pwmIn = 2000;
volatile uint16_t pwmOut = 2000;

// manual PWM, adakah cara yang lebih baik? ._.
ISR(TIMER1_OVF_vect){
	PORTB |= (1<<2);
}
ISR(TIMER1_COMPA_vect){
	PORTB &= ~(1<<2);
}

// hitung pwm input
ISR(TIMER0_OVF_vect){
	timerH++;
}
ISR(INT0_vect){
	uint16_t timeNow = ((uint16_t)timerH<<8) | TCNT0;
	
	if( PIND&(1<<2) ){ // awal sinyal pwm
		PORTB |= (1<<1); // passthru
		time0 = timeNow;
	}else{ // akhir sinyal pwm
		PORTB &= ~(1<<1);
		time1 = timeNow;
		pwmIn = (time1-time0);

		// kasih batas nilai
		if(pwmIn<2000) pwmIn = 2000;
		else if(pwmIn>4000) pwmIn = 4000;

		// remap
		float value = (float)(pwmIn-2000)/2000.0 * range;
		pwmOut = (uint16_t)value + 2000 + trimLow;
		OCR1A = pwmOut;
	}
}

int main( void ){

	DDRB = 0x00;
	DDRC = 0x00;

	// input di PD2 dan PD3 (INT0, INT1)
	DDRD = 0x00;

	// output di PB1 dan PB2 (OCR1A, OCR1B)
	DDRB = (1<<1) | (1<<2);

	// pakai timer0 untuk hitung pwm input
	TCCR0=(0<<CS02) | (1<<CS01) | (0<<CS00);
	TCNT0=0x00;

	// manual pwm untuk output
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	TCNT1=0x0000;
	ICR1=0x4E20; // overflow di 10ms
	OCR1A=0x0000;
	OCR1B=0x0000;

	TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (1<<OCIE1A) | (0<<OCIE1B) | (1<<TOIE1) | (1<<TOIE0);

	// external interrupt di INT0 untuk baca pwm
	//PORTD |= 0x02;
	GICR|=(0<<INT1) | (1<<INT0);
	MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (1<<ISC00);
	GIFR=(0<<INTF1) | (1<<INTF0);
	
	ADC_Init();
	sei();
 	
 	PORTD |= (1<<3); // seharusnya untuk tombol input
 	OCR1A = 2000; // inisialisasi pwm 1000us

	while(1){
		//if(PIND&(1<<3)==0){
			// seharusnya jangan update setiap saat, hanya update saat tombol ditekan
			// tapi belum diimplementasi
			// dan seharusnya disimpan di eeprom :p
			trimLow  = ADC_Read(0);
			trimHigh = ADC_Read(1);
			range = 2000 - ( (trimHigh+trimLow)>>1 );
		//}
		
		_delay_ms(20);
	}
}