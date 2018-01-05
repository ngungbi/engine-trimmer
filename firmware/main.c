#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

#include "serial.h"
#include "hx711.h"
#include "adc.h"

#include "config.h"

volatile uint16_t time0 = 0;
volatile uint16_t timeD = 0;

uint32_t loadcell1 = 0;
uint32_t loadcell2 = 0;
uint32_t loadcell3 = 0;
uint16_t voltage = 0;
uint16_t current = 0;

volatile uint8_t pwm = 0;
volatile uint8_t timeout = 0;

ISR(TIMER1_OVF_vect){
	timeout++;
	if(timeout>=4){
		OCR2A = 130;
	}
}
ISR(TIMER1_COMPA_vect){
	time0 = TCNT1;
	timeD = 0;
}
ISR(INT0_vect){
	uint16_t time1 = TCNT1;
	timeD = time1 - time0;
	time0 = time1;
	OCR1A = time1 - 1;
}
ISR(USART_RX_vect){
	uint8_t cc = UDR;
	//uint8_t checkbit = 0;
	//for(uint8_t i=0; i<8; i++){
	//	checkbit ^= (cc>>i);
	//}
	//if(checkbit&0x01 == 0x00) {
		//pwm = (cc>>1) & 0x0f;
		pwm = cc>>1;
		OCR2A = pwm + 130;
		timeout = 0;
	//}
}

char txBuffer[20];
void prepareData(void){
	txBuffer[0] = 'V';
	txBuffer[1] = 'T';
	// loadcell 3x3
	// tachometer 2
	// voltage 2
	// current 2
	// total 15, untuk sementara 9
	//loadcell = 5194303;
	txBuffer[2] = loadcell1 & 0xFF;
	txBuffer[3] = (loadcell1>>8) & 0xFF;
	txBuffer[4] = (loadcell1>>16) & 0xFF;
	txBuffer[5] = loadcell2 & 0xFF;
	txBuffer[6] = (loadcell2>>8) & 0xFF;
	txBuffer[7] = (loadcell2>>16) & 0xFF;
	txBuffer[8] = loadcell3 & 0xFF;
	txBuffer[9] = (loadcell3>>8) & 0xFF;
	txBuffer[10] = (loadcell3>>16) & 0xFF;
	txBuffer[11] = timeD & 0xFF;
	txBuffer[12] = (timeD>>8) & 0xFF;
	txBuffer[13] = voltage & 0xFF;
	txBuffer[14] = (voltage>>8) & 0xFF;
	txBuffer[15] = current & 0xFF;
	txBuffer[16] = (current>>8) & 0xFF;
	txBuffer[17] = 0;
	for(uint8_t i=2; i<=16; i++){
		txBuffer[17] ^= txBuffer[i];
	}
	//txBuffer[12] = 0;
}

int main( void ){
	char str[10];


	
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	SerialInit(115200);
	SerialWriteBuffer("Halo dunia\n");

	HX711_Init();

	// PWM Servo, PD3
	DDRB |= (1<<3);
	TCCR2A = (1<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (1<<WGM20);
	TCCR2B = (0<<WGM22) | (1<<CS22) | (0<<CS21) | (1<<CS20);
	OCR2A = 130;

	// Tachometer, PD2
	PORTD = (1<<2);
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
	EICRA = (0<<ISC11) | (0<<ISC10) | (1<<ISC01) | (1<<ISC00);
	EIMSK = (0<<INT1) | (1<<INT0);
	EIFR = (0<<INTF1) | (1<<INTF0);
	TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (1<<TOIE1);

	ADC_Init();

	sei();

	while(1){
		HX711_Read();
		loadcell1 = HX711_GetValue(0);
		loadcell2 = HX711_GetValue(1);
		loadcell3 = HX711_GetValue(2);
		voltage = ADC_Read(0);
		current = ADC_Read(1);
		//sprintf(str,"%ld\n",loadcell1);
		//SerialWriteBuffer(str);
		//SerialWriteBuffer("\n");
		_delay_ms(19);
		//HX711_Read();
		//loadcell1 = 100;//(loadcell1 + HX711_GetValue(0))>>1;
		//loadcell2 = (loadcell2 + HX711_GetValue(1))>>1;
		//loadcell3 = (loadcell3 + HX711_GetValue(2))>>1;
		prepareData();
		SerialSendBuffer(txBuffer, 18);
		_delay_ms(78);
	}
}