/*
 * ButtonBlink.c
 *
 * Created: 6/13/2015 7:24:21 PM
 *  Author: dragon
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

int main(void)
{
	int millis = 0;
	TCNT1 = 0; // Initialize the counter to zero. At least for now, to verify it works.
	
	// Set the following B pins as output: 
	DDRB |= ((1 << 1) | (1 << 2) | (1 << 3) | (1 << 4));
	
	TCCR1B |= (1 << CS10); // Enables the counter register with no prescaling.

    while(1)
    {
		while(TCNT1 <= 15999){ // Really really hacky way of waiting one millisecond. 
			
		}
		TCNT1 = 0; // Still hacky.
		millis ++; // Stop being so hacky. 
		
		// Toggle the LEDs!
		if(millis % 125 == 0){
			PORTB ^= (1 << 1);
		} 
		if (millis % 250 == 0){
			PORTB ^= (1 << 2);
		} 
		if (millis % 500 == 0){
			PORTB ^= (1 << 3);
		} 
		if (millis % 1000 == 0){
			PORTB ^= (1 << 4);
		}
		
		if (millis >= 1000) {
			millis = 0;
		}
	}
}