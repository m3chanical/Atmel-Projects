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

 
volatile uint16_t dht_humidity = 0xff; // Bytes 0 & 1 of the received data divided by 10
volatile uint16_t dht_temperature = 0xff; // Bytes 2 & 3 of received data divided by 10

int main(void)
{
	init(); // Initialize the chip: 
			//		- Set an interrupt for every two seconds
			//		- Set Port B pins as OUTPUT
			//		- That's it.
	

	
    while(1)
    {
		//PORTD = dht_temperature;
    }
}

void init(){
	// Setting up the timer:
	cli(); // disable global interrupts so we don't break our set up
	
	TCCR1B |= (1 << CS10 | 1 << CS12); // Enable counter register with prescale of 1024.
	OCR1A = 39100;	// Counting 31250 cycles to equal two seconds: 16mhz / 1024 = 15625; 15625 * 2 = 31250. Timer starts at 0.
					// TEMP: Increased time to about 2.5sec ******
	TCCR1B |= (1 << WGM12);		// Timer/Counter1 in clear timer on compare mode
	TIMSK1 |= (1 << OCIE1A);	// Enable timer compare interrupt
	
	sei(); // Re-enable interrupts!

	DDRB |= (1 << SENSOR_PIN);
	PORTB |= (1 << SENSOR_PIN); //Set Sensor pin as output, and pull it high.
	
	DDRD = 0xFF; //Set all PORT D pins as output. 
	DDRC |= (1 << DDC5); 
	
}

void dht22_read(){ 
	
	uint8_t data[5]; // Five bytes for received data from DHT22 Sensor. 5th byte is checksum.
	/* 
		- data[0] and data[1] -- relative humidity multiplied by ten
		- data[2] and data[3] -- temperature in Celsius multiplied by ten..
		- data[4]			  -- check sum of previous data: data[0] + ... + data[3] = data[4]
	*/
	
	uint8_t rcv_byte; // Current byte being received. Will be added to the data[] array after loop.
	uint8_t sum; // Used for addition of data bytes 0 through 3 to observe against checksum.
	
	/* INITIALIZING Sensor: */

	//OUTPUT: low for 1-10ms, then high
	//INPUT: sensor pulls low for 80us, then high for 80us. then data transmission starts:
	
	// OUTPUT: 
	DDRB |= (1 << SENSOR_PIN); // Ensure DHT22 pin set as output: 
	PORTB |= (0 << SENSOR_PIN); // Set sensor pin low for 1-10ms:
	_delay_ms(2); //low for 2ms
	PORTB |= (1 << SENSOR_PIN);
	_delay_us(30); //high for 20-40us; 
	PORTB &= ~(1 << SENSOR_PIN); // Set low before changing to input: 
	
	// INPUT: 
	DDRB &= ~(1 << SENSOR_PIN); // set sensor pin as input.
	
	_delay_us(40); // in middle of expected low from sensor: 

	if(PINB & (1 << PINB0)){		// If the signal on the pin is High, emit error:
		
		// something went wrong. flash leds maybe? I dunno.
		// Use Port C to drive LEDs as status indicators of last read. 
		//return;
		
	}
	
	_delay_us(60) ; // should be in the middle of the high signal from sensor
	
	if(PINB & ( 0 << PINB0)){	// If the signal is Low, emit error:  
		// something went wrong. cry yourself to sleep
		//return;
	}
	
	// Need to wait ~50 more us:
	_delay_us(50); 
	
	/* FOR LOOPS read temperature and humidity date in from sensor. */
	for (uint8_t b = 0; b < 5; b++) { // Reading a total of five bytes from the sensor
		
		rcv_byte = 0; 
		
		for(uint8_t i = 0; i < 8; i++){ // 8 bits per byte ;) 
			
			// Wait until the sensor pulls low again, to ensure
			//		we're waiting for the next bit to be sent
			while(PINB & (1 << PINB0)) {
				// Could probably add some error handling maybe? 
				//	I don't know how accurate the timing pulses match the datasheet.
			}
			
			// Wait until the sensor pulls back high, this
			//		means that the 50us "Start transmit 1 bit" has completed
			//		and the next bit of data is sent
			while( !(PINB & (1 << PINB0)) ){
				// Same as above loop. if I add in >50us error handling, how often will that error in fact be wrong?
			}
			
			// Then, we'll check the bit at about 50us through its transmit bit: 
			_delay_us(50);
			// ... and make our decision: 
			rcv_byte <<= 1;
			if(PINB & (1 << SENSOR_PIN))	// Read bit: 
				rcv_byte |= 1;
		}
		
		data[b] = rcv_byte; // Saves the byte just retrieved from the sensor to its appropriate position. (See datasheet).
	}
	
	sum = data[0] + data[1] + data[2] + data[3];
	if(data[4] != sum) { // Verify the sum of the data received against the checksum bits
		
		// cry some more because you failed.
		// TODO: Add in some kind of error handling. Whether it's a simple LED or other.
		
	}
	
	dht_humidity = ((data[0] << 8) | data[1]) / 10; // Data received is 2 bytes wide, so we need to combine bytes and divide by ten.
	dht_temperature = (((data[2] & 0b01111111) << 8) | data[3]) / 10; // Removing the sign bit and combining, then dividing by ten.
	
	if(data[2] & 0b10000000){ // If the temperature is recorded as negative: 
		dht_temperature *= -1;
	}
	
	dht_temperature = (dht_temperature * 1.8) + 32;

	dht_humidity = floor(dht_humidity);
	dht_temperature = floor(dht_temperature);
	
	PORTD = dht_temperature; // Binary LEDs!

}

ISR(TIMER1_COMPA_vect) {
	
		/* This triggers every two and a half-ish seconds based on the math in the init function. 
			- Purpose: Perform a read on the DHT22 sensor. 
			- Read function of sensor is called.
		*/
		PORTC ^= (1 << DDC5);
		dht22_read();
}