/*
 * dht22_interrupt.c
 *
 * Created: 7/14/2015 12:00:32 AM
 *  Author: dragon
 */ 

#define F_CPU 16000000UL
#define SENSOR_PIN DDB0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Function Prototypes:
void init();
void dht22_read();

 
volatile uint16_t dht_humidity; // Bytes 0 & 1 of the received data divided by 10
volatile uint16_t dht_temp; // Bytes 2 & 3 of received data divided by 10

int main(void)
{
	init(); // Initialize the chip: 
			//		- Set an interrupt for every two seconds
			//		- Set Port B pins as OUTPUT
			//		- That's it.
	

	
    while(1)
    {
		
    }
}

void init(){
	// Setting up the timer:
	cli(); // disable global interrupts so we don't break our set up
	
	TCCR1B |= (1 << CS10 | 1 << CS12); // Enable counter register with prescale of 1024.
	OCR1A = 31249; // Counting 31250 cycles to equal two seconds: 16mhz / 1024 = 15625; 15625 * 2 = 31250. Timer starts at 0.
	TCCR1B |= (1 << WGM12); // Timer/Counter1 in clear timer on compare mode
	TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
	
	// TODO: enable port B pins as output for binary LEDs. (future, use shift register maybe?);
	DDRB |= (1 << SENSOR_PIN);
	PORTB |= (1 << SENSOR_PIN); //Set Sensor pin as output, and pull it high.
	
	sei(); // Re-enable interrupts!
	
}

void dht22_read(){
	uint8_t data[5]; // Five bytes for received data from DHT22 Sensor. 5th byte is checksum.
	uint8_t rcv_byte; // Current byte being received. Will be added to the data[] array after loop.
	
	/* INITIALIZING Sensor: */

	//OUTPUT: low for 1-10ms, then high
	//INPUT: sensor pulls low for 80us, then high for 80us. then data transmission starts:
	
	// OUTPUT: 
	DDRB |= (1 << SENSOR_PIN); // Ensure DHT22 pin set as output: 
	PORTB |= (0 << SENSOR_PIN); // Set sensor pin low for 1-10ms:
	_delay_ms(2); //low for 2ms
	PORTB |= (1 << SENSOR_PIN);
	_delay_us(30); //high for 20-40us; 
	// INPUT: 
	DDRB &= ~(1 << SENSOR_PIN); // set sensor pin as input.
	_delay_us(40); // in middle of expected low from sensor: 
	if(PINB0 & 1){
		// something went wrong. flash leds maybe? I dunno.
		return;
	}
	_delay_us(60) ; // should be in the middle of the high signal from sensor
	if(PINB0 & 0){
		// something went wrong. cry yourself to sleep
		return;
	}
	// Sensor then pulls low for 80us, then high for 80us.

	/* FOR LOOPS read temperature and humidity date in from sensor. */
	for (uint8_t b = 0; b < 5; b++) { // Reading a total of five bytes from the sensor
		
		rcv_byte = 0; 
		
		for(uint8_t i = 0; i < 8; i++){ // 8 bits.
			// before each data bit, sensor drops low for 50us
			_delay_us(50);		
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	
		/* This triggers every two seconds based on the math in the init function. 
			- Purpose: Perform a read on the DHT22 sensor. 
			- Read function of sensor is called.
		*/
		dht22_read();
}