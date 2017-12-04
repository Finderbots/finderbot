#include <avr/io.h>

int main(void) {
	DDRB |= _BV(PB0);
	DDRE |= _BV(PE6);

	PORTB |= _BV(PB0);

	while(1) {
		PORTE ^= _BV(PE6);
	}
}
