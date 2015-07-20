/*
 * dht22_interrupt.c
 *
 * Created: 7/14/2015 12:00:32 AM
 *  Author: dragon
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Function Prototypes:
void init();

volatile uint8_t data[5]; // Five bytes for received data from DHT22 Sensor. 5th byte is checksum. 
volatile uint16_t dht_humidity; // Bytes 0 & 1 of the received data divided by 10
volatile uint16_t dht_temp; // Bytes 2 & 3 of received data divided by 10

int main(void)
{
	init(); // Initialize the chip: 
			//		- Set an interrupt for every two seconds
			//		- Set Port B pins as OUTPUT
			//		- That's it.
			
	/* In ISR: 
		- Write DHT22 routine.
		- Can only read every two seconds (hence interrupt) 
		- Or maybe it can? I'll check the datasheet.
	*/
	
	/* To write driver for DHT22:
		- need microsecond timing. use _delay_us function, because it's easy for now.
		- will receive 5 bytes of data (40 bits) 
			- First two bytes: Humidity
			- Second two bytes: Temperature in C
			- Final byte: Check sum (addition of all other bytes)
		- change pin to output and input dynamically. ( time to change ? )  
	*/
	
    while(1)
    {
		_delay_ms(1000); // This is to make sure I got my clock frequency correct with the F_CPU macro.
		PORTB ^= (1 << DDB0);
    }
}

void init(){
	// Setting up the timer:
	cli(); // disable global interrupts so we don't break our set up
	
	TCCR1B |= (1 << CS10 | 1 << CS12); // Enable counter register with prescale of 1024.
	OCR1A = 31249; // Counting 31250 cycles to equal two seconds: 16mhz / 1024 = 15625; 15625 * 2 = 31250. Timer starts at 0.
	TCCR1B |= (1 << WGM12); // Timer/Counter1 in clear timer on compare mode
	TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
	
	sei();
	
	// Set the following B pins as output:
	DDRB |= ((1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4));
}

ISR(TIMER1_COMPA_vect) {
	
		/* This triggers every two seconds based on the math in the init function. 
			- Purpose: Perform a read on the DHT22 sensor. 
			- 
		*/
		
		PORTB ^= ((1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4));
		
}