
/*
	Electric Lock Switch
	alpha version
    26.11.2016
	Copyright Michael Prader
		
	Fuse bits on ATtiny23123
	FF
	99
	C2

*/


//#define F_CPU     4000000

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
//#include <avr/eeprom.h>


#include "pins.h"


#include "keydeb.c"


#include "sig-defines.c"


/****************************/
// TIMINGS

#define SWITCH_POWER_TIMER  (4*4)       // in quarter seconds
//#define TRACK_RELAIS_POWER_TIMER (1*4)  // in quarter seconds
//#define LOCK_TIMEOUT_TIMER  (5*4)      // in quarter seconds

//#define ON_OFF_LOCKOUT	10			// in 10msec units



/****************************/

#define QUARTER_SECS_IN_10_MSEC 25

volatile uint8_t State;

#define QUARTER_SEC 0x01
#define UNLOCKED    0x02
#define TIMING_OUT  0x04

//volatile uint8_t OnOffLockout = ON_OFF_LOCKOUT;

#define EE_REQUESTED_POSITION_STORE_ADDRESS	0x10

void init(void);



ISR (TIMER0_OVF_vect)			// every 10ms
{
    static uint8_t ct0, ct1;//, rpt;
    uint8_t k, keyInput;
    // key debounce
    
    static uint8_t ticks = 0;

    TCNT0 = TIMER0_PRELOAD;	// preload for 10ms
  
  
  // compose input register by reading various input PIN registers (PINA and PIND)
    if (POS_IN_KEY_PIN & POS_IN_KEY_MASK) keyInput |= POS_IN_KEY_DATA_BYTE_MASK; else  keyInput &= ~POS_IN_KEY_DATA_BYTE_MASK;
    if (UNLOCK_KEY_PIN & POS_IN_KEY_MASK) keyInput |= UNLOCK_KEY_DATA_BYTE_MASK; else  keyInput &= ~UNLOCK_KEY_DATA_BYTE_MASK;

  
  
    k = key_state ^ keyInput;		// key changed ?  // "~" omitted for HIGH-active inputs; (LOW active: k = key_state ^ ~KEY_PIN;)
    ct0 = ~( ct0 & k );			// reset or count ct0
    ct1 = ct0 ^ (ct1 & k);		// reset or count ct1
    k&= ct0 & ct1;			// count until roll over ?
    key_state ^= k;			// then toggle debounced state
    key_press |= key_state & k;		// 0->1: key press detect
    /*
    if( (key_state & REPEAT_MASK) == 0 )	// check repeat function
     rpt = REPEAT_START;		// start delay
    if( --rpt == 0 ){
    rpt = REPEAT_NEXT;			// repeat delay
    key_rpt |= key_state & REPEAT_MASK;
    */

    //if (OnOffLockout) --OnOffLockout;
  
    ++ticks;
	if (ticks >= QUARTER_SECS_IN_10_MSEC)
	{
	   ticks = 0;
	   State |= QUARTER_SEC;
	}
  
}

int main(void)
{
	uint8_t i;
	uint8_t SwitchRequestedPosition, SwitchReadPosition;
	uint8_t SwitchPowerTimer = 0;//, TrackRelaisPowerTimer = 0;
//	uint8_t LockTimeoutTimer = 0;
	//uint8_t KeyMemory = 0;

    // activate MOSFET at wakeup
//	HOLD_DDR |= HOLD_MASK;
//	HOLD_PORT |= HOLD_MASK;
	
 

    // memorize high keys
  //  if (POS_IN_KEY_PIN & POS_IN_KEY_MASK) KeyMemory |= POS_IN_KEY_DATA_BYTE_MASK; else  KeyMemory &= ~POS_IN_KEY_DATA_BYTE_MASK;
   // if (UNLOCK_KEY_PIN & POS_IN_KEY_MASK) KeyMemory |= UNLOCK_KEY_DATA_BYTE_MASK; else  KeyMemory &= ~UNLOCK_KEY_DATA_BYTE_MASK;
	
	State = 0; // deactivate UNLOCKED, TIMING_OUT
	
	init();
	
	
	//SwitchRequestedPosition = TO_NORMAL;
	
	// deactivate switch motor
	SWITCH_POWER_DISABLE_PORT |= SWITCH_POWER_DISABLE_MASK;

    // deactivate relais
//    TRACK_RELAIS_PORT &= ~TRACK_RELAIS_MASK;
  //  TRACK_RELAIS_POWER_DISABLE_PORT &= ~TRACK_RELAIS_POWER_DISABLE_MASK;
    
	sei();
	
	
	
	//if (!(eeprom_is_ready())) eeprom_busy_wait();
	//SwitchRequestedPosition = eeprom_read_byte((uint8_t *) EE_REQUESTED_POSITION_STORE_ADDRESS);

	if ((SwitchRequestedPosition != TO_NORMAL) && (SwitchRequestedPosition != TO_REVERSE))  SwitchRequestedPosition = TO_NORMAL;
	
    /**************************/
    // TODO - could delay some 50-100 ms here?
    /**************************/
    

	// clear any keys previously pressed
	i = get_key_press(0xFF);
	i = get_key_short(0xFF);
	i = get_key_long(0xFF);
	i = get_key_rpt(0xFF);

	while(1)
    {
        // MAIN LOOP
        
        // read status of turnout; because of the capacitors, initial position will
		// be wrong until the capacitors have fully loaded
    	if (SWITCH_N_PIN & SWITCH_N_MASK) SwitchReadPosition |= TO_NORMAL; else SwitchReadPosition &= ~TO_NORMAL;
    	if (SWITCH_R_PIN & SWITCH_R_MASK) SwitchReadPosition |= TO_REVERSE; else SwitchReadPosition &= ~TO_REVERSE;
		
		SwitchReadPosition &= TO_NORMAL | TO_REVERSE;
		
		
        // read the MODE configuration switch
    	if (MODE_PIN & MODE_MASK)
    	{
    	    // SW1 is high (off)
    	    // in this mode, we work according to CTC input
    	    // if the TO_REQ_REVERSE output on the CTC controller is high, set switch to REVERSE
    	    if (CTC_IN_PIN & CTC_IN_MASK) SwitchRequestedPosition = TO_REVERSE; else SwitchRequestedPosition = TO_NORMAL;
    	    
    	    if ( get_key_press( POS_IN_KEY_DATA_BYTE_MASK))
            {
                // clear a key pressed
                asm("nop");
            }
            
            LED_0N_PORT |= LED_0N_MASK;
            LED_1R_PORT |= LED_1R_MASK;
            
    	} else {
    	    // SW1 is low (on)
    	    // in this mode, we disregard CTC input and activate manual control
			
            if ( get_key_press( POS_IN_KEY_DATA_BYTE_MASK))// || (KeyMemory & POS_IN_KEY_DATA_BYTE_MASK))
            {
                // key pressed, toggle switch position; in this mode, no lock is simulated
                if (SwitchRequestedPosition == TO_NORMAL) SwitchRequestedPosition = TO_REVERSE; else if (SwitchRequestedPosition == TO_REVERSE) SwitchRequestedPosition = TO_NORMAL;
                // clear stored key at startup
                //KeyMemory &= ~POS_IN_KEY_DATA_BYTE_MASK;
            }
    	}
    	
    	if (SwitchRequestedPosition == TO_NORMAL) SWITCH_POS_PORT |= SWITCH_POS_MASK; else SWITCH_POS_PORT &= ~SWITCH_POS_MASK;
    	
        if (SwitchRequestedPosition == TO_NORMAL)
        {
            LED_0N_PORT &= ~LED_0N_MASK;
            LED_1R_PORT |= LED_1R_MASK;
        } else if (SwitchRequestedPosition == TO_REVERSE) {
            LED_0N_PORT |= LED_0N_MASK;
            LED_1R_PORT &= ~LED_1R_MASK;
        }
            
    	
    	if (   SwitchRequestedPosition  != SwitchReadPosition )
    	{
    	    // switch state is not congruent with requested state
    	    SwitchPowerTimer = SWITCH_POWER_TIMER;
    	}
    	
    	if (SwitchPowerTimer) SWITCH_POWER_DISABLE_PORT &= ~SWITCH_POWER_DISABLE_MASK; else SWITCH_POWER_DISABLE_PORT |= SWITCH_POWER_DISABLE_MASK;
		//if (TrackRelaisPowerTimer) TRACK_RELAIS_POWER_DISABLE_PORT &= ~TRACK_RELAIS_POWER_DISABLE_MASK; else TRACK_RELAIS_POWER_DISABLE_PORT |= TRACK_RELAIS_POWER_DISABLE_MASK;

		
		if (State & QUARTER_SEC)
        {
            if (SwitchPowerTimer) --SwitchPowerTimer;
            //if (TrackRelaisPowerTimer) --TrackRelaisPowerTimer;
            //if (LockTimeoutTimer) --LockTimeoutTimer;
            
            /*if (State & UNLOCKED)
            {
                LED_0N_PORT &= ~LED_0N_MASK;
                LED_1R_PORT &= ~LED_1R_MASK;
            } else if (State & TIMING_OUT) {
                LED_0N_PORT ^= LED_0N_MASK;
                LED_1R_PORT ^= LED_1R_MASK;
            } else {
                LED_0N_PORT |= LED_0N_MASK;
                LED_1R_PORT |= LED_1R_MASK;
            }*/
            State &= ~QUARTER_SEC;
        }
        
        
       /* if ((SwitchPowerTimer == 0) && (TrackRelaisPowerTimer == 0) && !(State & (UNLOCKED | TIMING_OUT)))
        {
            // shutdown controller if no device is turning, and lock is not being opened or open
			if (!(eeprom_is_ready())) eeprom_busy_wait();
			eeprom_write_byte((uint8_t *) EE_REQUESTED_POSITION_STORE_ADDRESS, SwitchRequestedPosition);
			if (!(eeprom_is_ready())) eeprom_busy_wait();
            HOLD_PORT &= ~HOLD_MASK;
        }*/
    }
}



void init(void)
{
	
	ACSR = _BV(ACD);
	
	//inputs
	// keys are handled in KeysInit() initialization routine
	
	CTC_IN_PORT &= ~CTC_IN_MASK;
	CTC_IN_DDR &= ~CTC_IN_MASK;
	
	MODE_PORT &= ~MODE_MASK;
	MODE_DDR &= ~MODE_MASK;
	
	SWITCH_N_PORT &= ~SWITCH_N_MASK;
	SWITCH_N_DDR &= ~SWITCH_N_MASK;
	
	SWITCH_R_PORT &= ~SWITCH_R_MASK;
	SWITCH_R_DDR &= ~SWITCH_R_MASK;
	
	//outputs
	SWITCH_POS_DDR |= SWITCH_POS_MASK;
	//TRACK_RELAIS_DDR |= TRACK_RELAIS_MASK;
	
	SWITCH_POWER_DISABLE_DDR |= SWITCH_POWER_DISABLE_MASK;
	//TRACK_RELAIS_POWER_DISABLE_DDR |= TRACK_RELAIS_POWER_DISABLE_MASK;

	LED_0N_PORT |= LED_0N_MASK;
	LED_0N_DDR |= LED_0N_MASK;

	LED_1R_PORT |= LED_1R_MASK;
	LED_1R_DDR |= LED_1R_MASK;
	

	
	KeysInit();

}





