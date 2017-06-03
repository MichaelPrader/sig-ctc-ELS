

#include "keydeb.h"

volatile uint8_t key_state;				// debounced and inverted key state:
					// bit = 1: key pressed
volatile uint8_t key_press;				// key press detect

volatile uint8_t key_rpt;				// key long press and repeat


void KeysInit(void)
{

	TCCR0A = 0;		// no OC, TOV at MAX, MAX is 0xFF	
	TCCR0B = 0 | _BV(CS02) | _BV(CS00);		// divide by 1024
	TIMSK = _BV(TOIE0);									// enable timer interrupt
	
	POS_IN_KEY_PORT &= ~POS_IN_KEY_MASK;
	POS_IN_KEY_DDR &= ~POS_IN_KEY_MASK;
    //UNLOCK_KEY_PORT &= ~UNLOCK_KEY_MASK;
    //UNLCOK_KEY_DDR &= ~UNLOCK_KEY_MASK;
}


uint8_t get_key_press( uint8_t key_mask )
{
  cli();					// read and clear atomic !
  key_mask &= key_press;                        // read key(s)
  key_press ^= key_mask;                        // clear key(s)
  sei();
  return key_mask;
}


uint8_t get_key_rpt( uint8_t key_mask )
{
  cli();					// read and clear atomic !
  key_mask &= key_rpt;                        	// read key(s)
  key_rpt ^= key_mask;                        	// clear key(s)
  sei();
  return key_mask;
}


uint8_t get_key_short( uint8_t key_mask )
{
  cli();			// read key state and key press atomic !
  return get_key_press( ~key_state & key_mask );
}


uint8_t get_key_long( uint8_t key_mask )
{
  return get_key_press( get_key_rpt( key_mask ));
}

