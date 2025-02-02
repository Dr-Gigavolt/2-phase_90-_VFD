/*
 * VFD_90deg.cpp
 *
 * Created: 09.01.2022 21:19:13
 * Author : Dr. GigaVolt
 
 * PWM outputs: 
 * Main pos: PSCOUT00, pin 1
 * Main neg: PSCOUT01, pin 32
 * 90° pos: PSCOUT10, pin 2
 * 90° neg: PSCOUT11, pin 31
 */ 

#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#define LENGHT 512								// length of the 1/2-sine LUT which must be stored in EEPROM, all 512 bytes used, amplitude 255
#define FMIN 5									// minimum frequency
#define FMAX 50									// maximum frequency
#define PER 800									// PSC period in PSC clocks
#define DT 20									// dead time in PSC clocks
#define PWM_IN 6								// PORTD 6, int0
#define FAN 7									// PORTD7, IGBT fan control
#define TEST 5									// debug outpput, PORTD 5
#define ARR_LEN 255

volatile float amp = 0;							// output amplitude to set sine amplitude inverse-proportional to frequency, 0 ... 1
volatile uint16_t cnt0 = 0, cnt90 = LENGHT/2;	// PSC counters, 0° und 90°
volatile uint8_t freq = FMIN;					// output frequency
volatile uint16_t rising = 0, falling = 0;		// time of the control PWM edges
volatile uint8_t array[ARR_LEN], index = 0;		// array for averaging (soft start / change / stop)
volatile bool new_sample = false;				// control PWM sample completed
volatile bool started = false;					// control PWM measurement started

// ******************************************************************************************
// ********************************* Functions **********************************************
// ******************************************************************************************

uint8_t readEEPROM(uint16_t addr)		// read value from EEPROM
{
	EEAR = addr;
	EECR = (1 << EERE);
	return EEDR;
}

void toggle_test(void)					// toggle pin for debugging
{
	static bool pin_on = false;
	if (pin_on) PORTD |= (1<<TEST);
	else PORTD &= ~(1<<TEST);
	pin_on = !pin_on;
}

// ******************************************************************************************
// ******************************** Interrupts **********************************************
// ******************************************************************************************

ISR(INT0_vect)							// interrupt by PWM control (0% ... 0Hz, 100% ... FMAX)
{
	if (PIND & (1 << PWM_IN))			// OC LED off, rising edge INT0
	{
		//PORTD |= (1<<TEST);
		rising = TCNT1;					// get time of rising edge
	}
	else								// OC LED on, falling edge INT0
	{
		//PORTD &= ~(1<<TEST);
		if (started)
		{
			array[index] = uint8_t((FMAX * uint32_t(rising - falling)) / uint32_t(TCNT1 - falling));
			TCNT1 = 0;
			if (index < ARR_LEN) index++;	// iterating through array for duty cycle averaging
				else index = 0;
			started = false;			// duty cycle measurement finished
			new_sample = true;			// new sample of duty cycle ready
		}
		else 
		{
			started = true;				// duty cycle measurement started
			falling = TCNT1;			// get time of falling edge
		}			
	}
}

// ******************************************************************************************

ISR(TIMER0_COMPA_vect)
{			
	
	PCNF0 |= (1 << PLOCK0);		// lock PSC0 update
	PCNF1 |= (1 << PLOCK1);		// lock PSC1 update
	
	sei();
	float sine_val0 = amp * float(readEEPROM(cnt0)) / 255.0;
	float sine_val90 = amp * float(readEEPROM(cnt90)) / 255.0;
	cli();
	
	// OCR0 for 0°
	if (cnt0 > LENGHT) sine_val0 = -sine_val0;
	OCR0RA = int(PER/2 * (sine_val0+1));
	OCR0RB = PER - OCR0RA;
	if (OCR0RA < DT) OCR0RA = DT;
	if (OCR0RB < DT) OCR0RB = DT;
	
	// OCR1 for 90°
	
	if (cnt90 > LENGHT) sine_val90 = -sine_val90;
	OCR1RA = int(PER/2 * (sine_val90+1));
	OCR1RB = PER - OCR1RA;
	if (OCR1RA < DT) OCR1RA = DT;
	if (OCR1RB < DT) OCR1RB = DT;
	
	PCNF0 &= ~(1 << PLOCK0);		// release PSC0 update
	PCNF1 &= ~(1 << PLOCK1);		// release PSC1 update
	
	cnt0 += freq;		
	cnt90 = cnt0 + LENGHT/2;
	if (cnt0 >= 2*LENGHT) cnt0 -= 2*LENGHT;	
	if (cnt90 >= 2*LENGHT) cnt90 -= 2*LENGHT;
}

// ******************************************************************************************

ISR(TIMER1_OVF_vect)
{
	uint8_t value;
	if (PIND & (1 << PWM_IN)) value = 0;		// inverted
		else value = FMAX;
	array[index] = value;
	if (index < ARR_LEN) index++;
		else index = 0;
	new_sample = true;		
}

// ******************************************************************************************
// *********************************** Main *************************************************
// ******************************************************************************************

int main(void)
{
	WDTCSR = (1 << WDE);										// set watchdog 16ms, reset on overflow
	
	DDRD = ((1 << FAN) | (1 << TEST));							// IGBT fan and debugging output	
	PORTD = (1 << PWM_IN);										// PWM input

	// external interrupt
	EICRA = (1 << ISC00);										// any logical change generates interrupt
	EIMSK = (1 << INT0);										// enable external INT0
	
	// set timer0 (DDS)
	TCCR0A = (1 << WGM01);										// CTC (Clear Timer on Compare match) mode (OCR0A)
	TCCR0B = ((1 << CS01) | (1 << CS00));						// prescaler 64 (250kHz, 4us)
	TIMSK0 = (1 << OCIE0A);										// output compare match A interrupt enable
	OCR0A = 240;												// 1042 Hz, 960us (1 Hz step for 10-bit PSC)

	// set timer1 (PWM in)
	TCCR1B = (1 << CS10);										// normal mode, start with no prescaler
	TIMSK1 = (1 << TOIE1);										// enable timer1 overflow interrupt
		
	// set PSC0 and 1 to 1-ramp mode
	PCNF0 = ((1 << POP0) | (1 << PMODE00));						// 16MHz clock; 2-ramp; pos. polarity
    PCNF1 = ((1 << POP1) | (1 << PMODE10));						// 16MHz clock; 2-ramp; pos. polarity
	PCTL0 = 0;													// no prescaler
	PCTL1 = (1 << PARUN1);										// no prescaler, PSC1 starts with PSC0
	PSOC0 = ((1 << POEN0B) | (1 << POEN0A));					// PSC0 output pins enabled
	PSOC1 = ((1 << POEN1B) | (1 << POEN1A));					// PSC1 output pins enabled
	
	// set PSC PWM 20kHz, 0% on start
	OCR0RA = PER/2;	
	OCR0RB = PER/2;
	OCR1RA = PER/2;			
	OCR1RB = PER/2;		
	OCR0SA = DT;
	OCR0SB = DT;
	OCR1SA = DT;		
	OCR1SB = DT;
	
	PCNF0 |= (1 << PLOCK0);		// lock PSC0 update
	PCNF1 |= (1 << PLOCK1);		// lock PSC1 update
	
	sei(); 
			
	while (1) 
    {		
		if (new_sample)											// wait for new element of ARR array
		{
			asm("WDR");
			//toggle_test();
			
			// calculate motor frequency as floating average of the array
			uint16_t sum = 0;
			for (uint8_t i = 0; i < ARR_LEN; i++)
				sum += array[i];
			freq = uint8_t(sum / ARR_LEN);
			
			if (freq < FMIN)									// stop motor and IGBT cooling
			{
				PCTL0 &= ~(1 << PRUN0);							// PWM stop
				PORTD &= ~(1 << FAN);							// IGBT fan off
				freq = FMIN;
			}
			else
			{
				PCTL0 |= (1 << PRUN0);							// PWM start
				PORTD |= (1 << FAN);							// IGBT fan on
				if (freq > FMAX) freq = FMAX;					// limit to FMAX
				amp = float(freq) / 50.0;						// amplitude inverse-proportional to frequency, 50 Hz ~ amp = 1
			}	
			new_sample = false;
		}			
    }
}


