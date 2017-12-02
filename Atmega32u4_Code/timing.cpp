#include "timing.h"

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
unsigned char timer0_fract = 0;

void timing_init(void)
{
  // this needs to be called before setup() or some functions won't
  // work there
  sei();
  
  // on the ATmega168, timer 0 is also used for fast hardware pwm
  // (using phase-correct PWM would mean that timer 0 overflowed half as often
  // resulting in different millis() behavior on the ATmega8 and ATmega168)
// #if defined(TCCR0A) && defined(WGM01)
//   sbi(TCCR0A, WGM01);
//   sbi(TCCR0A, WGM00);
// #endif  

  // set timer 0 prescale factor to 64

  sbi(TCCR0B, CS01);
  sbi(TCCR0B, CS00);
// #if defined(__AVR_ATmega128__)
//   // CPU specific: different values for the ATmega128
//   sbi(TCCR0, CS02);
// #elif defined(TCCR0) && defined(CS01) && defined(CS00)
//   // this combination is for the standard atmega8
//   sbi(TCCR0, CS01);
//   sbi(TCCR0, CS00);
// #elif defined(TCCR0B) && defined(CS01) && defined(CS00)
//   // this combination is for the standard 168/328/1280/2560
//   sbi(TCCR0B, CS01);
//   sbi(TCCR0B, CS00);
// #elif defined(TCCR0A) && defined(CS01) && defined(CS00)
//   // this combination is for the __AVR_ATmega645__ series
//   sbi(TCCR0A, CS01);
//   sbi(TCCR0A, CS00);
// #else
//   #error Timer 0 prescale factor 64 not set correctly
// #endif

  // enable timer 0 overflow interrupt
    sbi(TIMSK0, TOIE0);
// #if defined(TIMSK) && defined(TOIE0)
//   sbi(TIMSK, TOIE0);
// #elif defined(TIMSK0) && defined(TOIE0)
//   sbi(TIMSK0, TOIE0);
// #else
//   #error  Timer 0 overflow interrupt not set correctly
// #endif

}

unsigned long millis()
{
  unsigned long m;
  uint8_t oldSREG = SREG;

  // disable interrupts while we read timer0_millis or we might get an
  // inconsistent value (e.g. in the middle of a write to timer0_millis)
  cli();
  m = timer0_millis;
  SREG = oldSREG;

  return m;
}



unsigned long micros() {
  unsigned long m;
  uint8_t oldSREG = SREG, t;
  
  cli();
  m = timer0_overflow_count;
#if defined(TCNT0)
  t = TCNT0;
#elif defined(TCNT0L)
  t = TCNT0L;
#else
  #error TIMER 0 not defined
#endif

  
#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) && (t < 255))
    m++;
#else
  if ((TIFR & _BV(TOV0)) && (t < 255))
    m++;
#endif

  SREG = oldSREG;
  
  return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

