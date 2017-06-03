

#define TIMER0_PRELOAD	216	// for 10 msec with 4 MHz and prescaler 1024

#define UNLOCK_KEY_PIN  PINA
#define UNLOCK_KEY_DDR  DDRA
#define UNLOCK_KEY_PORT PORTA
#define UNLOCK_KEY_MASK 0x02    // PA2

#define UNLOCK_KEY_DATA_BYTE_MASK 0x01

#define POS_IN_KEY_PIN  PIND
#define POS_IN_KEY_DDR  DDRD
#define POS_IN_KEY_PORT PORTD
#define POS_IN_KEY_MASK 0x02    // PD2

#define POS_IN_KEY_DATA_BYTE_MASK    0x02

//#define REPEAT_MASK	(KEY_F_PLUS | KEY_F_MINUS | KEY_DC_PLUS | KEY_DC_MINUS)

//#define REPEAT_START	80		// after 800 ms
//#define REPEAT_NEXT	20		// every 200 ms


// prototypes
uint8_t get_key_press( uint8_t key_mask );
uint8_t get_key_rpt( uint8_t key_mask );
uint8_t get_key_short( uint8_t key_mask );
uint8_t get_key_long( uint8_t key_mask );
void KeysInit(void);
