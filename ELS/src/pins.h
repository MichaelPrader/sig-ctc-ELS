
#ifndef PINS_H
#define PINS_H


// mode pin
#define MODE_PIN	PINB
#define	MODE_PORT	PORTB
#define MODE_DDR	DDRB
#define	MODE_MASK	0x04

// ctc input
#define CTC_IN_PIN	PIND
#define CTC_IN_PORT	PORTD
#define CTC_IN_DDR	DDRD
#define CTC_IN_MASK	0x01

// switch position output
#define SWITCH_POS_PIN	PIND
#define SWITCH_POS_PORT	PORTD
#define SWITCH_POS_DDR	DDRD
#define SWITCH_POS_MASK	0x08


// switch normal posiion input
#define SWITCH_N_PIN	PIND
#define SWITCH_N_PORT	PORTD
#define SWITCH_N_DDR	DDRD
#define SWITCH_N_MASK	0x04


// switch reverse posiion input
#define SWITCH_R_PIN	PINB
#define SWITCH_R_PORT	PORTB
#define SWITCH_R_DDR	DDRB
#define SWITCH_R_MASK	0x10


// disable power to switch
#define SWITCH_POWER_DISABLE_PIN	PIND
#define SWITCH_POWER_DISABLE_PORT	PORTD
#define SWITCH_POWER_DISABLE_DDR	DDRD
#define SWITCH_POWER_DISABLE_MASK	0x20

// track relais output
#define TRACK_RELAIS_PIN    PIND
#define TRACK_RELAIS_PORT   PORTD
#define TRACK_RELAIS_DDR    DDRD
#define TRACK_RELAIS_MASK   0x10


// disable power to track relais
#define TRACK_RELAIS_POWER_DISABLE_PIN  PIND
#define TRACK_RELAIS_POWER_DISABLE_PORT PORTD
#define TRACK_RELAIS_POWER_DISABLE_DDR  DDRD
#define TRACK_RELAIS_POWER_DISABLE_MASK 0x40

// LED0 for the N position side
#define LED_0N_PIN  PINB
#define LED_0N_PORT PORTB
#define LED_0N_DDR  DDRB
#define LED_0N_MASK 0x01

// LED1 for the R position side
#define LED_1R_PIN  PINB
#define LED_1R_PORT PORTB
#define LED_1R_DDR  DDRB
#define LED_1R_MASK 0x02

// HOLD LINE
#define HOLD_PIN    PINA
#define HOLD_PORT   PORTA
#define HOLD_DDR    DDRA
#define HOLD_MASK   0x01

#endif


