/*
 * Line_Follower.c
 *
 * Created: 07-02-2020 13:07:21
 *  Author: srini
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "uart.c"
#define USART0_ENABLED


void line_follower_init()
{
	DDRC &= ~( (1 << PC0) | (1 << PC1) | (1 << PC2) ); //Enable as input for getting white line sensor values
	PORTC &= ~( (1 << PC0) | (1 << PC1) | (1 << PC2) ); //pull up disable for white line sensor input
	DDRB |= ( (1 << PB0) | (1 << PB1) | (1 << PB4) | (1 << PB5) ); //Motor direction control pins for motor driver
	DDRD |= ( (1 << PD5) | (1 << PD6) ); //PWM pins enable for motor driver; Timer0 is used
}

void adc_init(){
	ACSR = (1 << ACD);
	ADMUX = (1 << ADLAR);
	ADCSRA = ( (1 << ADEN) |  (1 << ADPS2) | (1 << ADPS1) ) ;
}

void buzzer_init()
{
	DDRD |= (1 << PD4); //pin 4
	PORTD |= (1 << PD4); //buzzer off
}

void servo_init(void){
	DDRB    |= (1 << PB3); //pin11; 9g servo
	DDRD   |= (1 << PD3); //pin3; 50gservo
}

char uart0_readByte(void)
{
	uint16_t rx;
	uint8_t rx_status, rx_data;
	rx = uart0_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);
	if(rx_status == 0 && rx_data != 0)
	{
		return rx_data;
	} 
	else 
	{
		return -1;
	}

}

void buzzer_1()
{
	PORTD &= ~(1 << PD4);
	_delay_ms(2000);
	PORTD |= (1 << PD4);
}

void buzzer_5()
{
	PORTD &= ~(1 << PD4);
	_delay_ms(5000);
	PORTD |= (1 << PD4);
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	Ch = Ch & 0b00000111;
	ADMUX = 0x60 | Ch;
	ADCSRA |= (1 << ADSC);
	while((ADCSRA & (1 << ADIF) ) == 0);
	a = ADCH;
	ADCSRA |= (1 << ADIF);
	return a;
}

void timer0_init()
{
	TCCR0B = 0x00;
	TCNT0 = 0xFF;
	OCR0A = 0xFF;
    TCCR0A |= (1 << COM0A1);
	TCCR0A &= ~(1 << COM0A0);
	TCCR0A |= (1 << COM0B1);
	TCCR0A &= ~(1 << COM0B0);
	TCCR0A |= (1 << WGM00);
	TCCR0A |= (1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	TCCR0B &= ~((1 << CS01) | (1 << CS00));
	TCCR0B |= (1 << CS02);
}

/*void timer1_init()
{
	OCR0A = 25;
	OCR0B = 40;
	PORTB |= ( (1 << PB0) | (0 << PB1) | (0 << PB4) | (0 << PB5) );
	TCCR1B = 0x00;
	TCNT1 = 49912;
	TCCR1B &= ~((1 << CS01) | (1 << CS00));
	TCCR1B |= (1 << CS02);
	while((TIFR1 & (1 << TOV1) ) == 0);
	bot_stop();
	TCCR1B = 0x00;
}
*/
void timer2_init()
{
	TCCR2B = 0x00;
	TCNT2 = 0xFF;
	OCR2A = 0xFF;
	TCCR2A |= (1 << COM2A1);
	TCCR2A &= ~(1 << COM2A0);
	TCCR2A |= (1 << COM2B1);
	TCCR2A &= ~(1 << COM2B0);
	TCCR2A |= (1 << WGM20);
	TCCR2A |= (1 << WGM21);
	TCCR2B &= ~(1 << WGM22);
	TCCR2B |= ((1 << CS21) | (1 << CS22));
	TCCR2B &= ~(1 << CS20);
}

void hitting_sequence()
{   
	//timer0_init();
	//_delay_ms(500);
	TCNT0 = 0;
	OCR0A = 30;
	OCR0B = 30;
	PORTB |= ( (1 << PB0) | (0 << PB1) | (0 << PB4) | (1 << PB5) );
	_delay_ms(110);
	bot_stop();
	servo_init();
	timer2_init();
	
	buzzer_1();
	_delay_ms(1000);
	buzzer_1();
	
	OCR2A = 10;
	_delay_ms(1000);
	//OCR2B = 10;
	//_delay_ms(1000);
	OCR2A = 70;
	_delay_ms(1000);
	//OCR2B = 130;
	
	TCCR2B=0x00;
	//_delay_ms(2500);
	buzzer_1();
	
	TCNT0 = 0;
	OCR0A = 30;
	OCR0B = 30;
	PORTB |= ( (0 << PB0) | (1 << PB1) | (1 << PB4) | (0 << PB5) );
	_delay_ms(110);
	bot_stop();
	//TCCR0B=0x00;
}

void follow_line()
{
	int pwm1=30;
	int pwm2=50;
	int w[3];
	unsigned char s[3];
	
	s[0] = ADC_Conversion(0);
	s[1] = ADC_Conversion(1);
	s[2] = ADC_Conversion(2);
	
	
	for (int i=0; i<3; i++)
	{
		if (s[i]>=50)
		{
			w[i]=1; //Black
		}
		else if (s[i]<50)
		{
			w[i]=0; //White
		}
	}
	
	if( w[0]==0 && w[1]==0 && w[2]==1)   //left turn
	{   OCR0A = pwm1;
		OCR0B = pwm2;
		PORTB |= ( (1 << PB0) | (0 << PB1) | (1 << PB4) | (0 << PB5) );
	}
	if( w[0]==0 && w[1]==1 && w[2]==1)   //more left turn
	{   OCR0A = (pwm1)-10;
		OCR0B = (pwm2)+10;
		PORTB |= ( (1 << PB0) | (0 << PB1) | (1 << PB4) | (0 << PB5) );
	}
	if( w[0]==1 && w[1]==0 && w[2]==0)   // right turn
	{   OCR0A = pwm2;
		OCR0B = pwm1;
		PORTB |= ( (1 << PB0) | (0 << PB1) | (1 << PB4) | (0 << PB5) );
	}
	if( w[0]==1 && w[1]==1 && w[2]==0)   //more right turn
	{   OCR0A = (pwm2)+10;
		OCR0B = (pwm1)-10;
		PORTB |= ( (1 << PB0) | (0 << PB1) | (1 << PB4) | (0 << PB5) );
	}
	/*if( w[0]==1 && w[1]==1 && w[2]==1)   //stop
	{   OCR0A = pwm1;
		OCR0B = pwm1;
		PORTB |= ( (0 << PB0) | (1 << PB1) | (0 << PB4) | (1 << PB5) );
	}*/
	if( w[0]==1 && w[1]==0 && w[2]==1)   //straight
	{   OCR0A = pwm2;
		OCR0B = pwm2;
		PORTB |= ( (1 << PB0) | (0 << PB1) | (1 << PB4) | (0 << PB5) );
	}
	
}

/*void calibrate_bot()
{
	OCR0A = 40;
	OCR0B = 30;
	PORTB |= ( (1 << PB0) | (0 << PB1) | (0 << PB4) | (0 << PB5) );
	_delay_ms(100);
	bot_stop();
}
*/	

void bot_stop()
{
	//OCR0A = 0;
	//OCR0B = 0;
	TCCR0B = 0x00;
	PORTB &= ~( (1 << PB0) | (1 << PB1) | (1 << PB4) | (1 << PB5) );
	//timer0_init();
}

/*int main(void)
{
	line_follower_init();
	adc_init();
	timer0_init();
	while(1)
	{
		follow_line();
	}
	return 0;
}
*/

int main(void) 
{   buzzer_init();
	line_follower_init();
	adc_init();
	timer0_init();
	char rx_byte;
	int y=0;
	int z=0;
	
	uart0_init(UART_BAUD_SELECT(9600, F_CPU));
	uart0_flush();
	uart0_puts("*** All components intialized ***\n");
	

	while(1)
	{
		rx_byte = uart0_readByte();
		
		if(rx_byte != -1)
		{   
			//uart0_puts("rx");
			if(rx_byte == 'm')
			 {  
				//uart0_putc(rx_byte);
				follow_line();
				//_delay_ms(100);
				//rx_byte = uart0_readByte(); 
			 }
			 
			 else if(rx_byte == 's')
			 {
				 if(y==0)
				 {
				 //uart0_putc(rx_byte);
				 bot_stop();
				 //TCCR0B=0x00;
				 //calibrate_bot();
				 //timer1_init();
				 //_delay_ms(1500);
				 hitting_sequence();
				 _delay_ms(100);
				 timer0_init();
				 //rx_byte = uart0_readByte();
				 y=1;
				 }				 				 
			 }
			 
			 else if(rx_byte == 'f')
			 {
				 if(z==0)
				 {
				 //uart0_putc(rx_byte);
				 bot_stop();
				 _delay_ms(500);
				 buzzer_5();
				 _delay_ms(100);
				 //rx_byte = uart0_readByte();
				 z=1;
				 }				 				 
			 }
			 
		}
		
	}

	return 0;
}