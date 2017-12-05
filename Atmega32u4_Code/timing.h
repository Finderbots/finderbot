#ifndef TIMING_H
#define TIMING_H

#include <avr/interrupt.h>
#include <util/delay.h>

 inline void delay( int ms )
  {
    for (int i = 0; i < ms; i++)
    {
       _delay_ms(1);
    }
 }

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))


// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
extern unsigned char timer0_fract;


unsigned long micros(void);
unsigned long millis(void);

void timing_init();

#endif /* TIMING_H */