#define F_CPU           (9600000UL) // Default clock speed

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define WAKES_PER_HOUR    (480) // Watchdog wakes up approximately every 7.5 seconds. 3600 / 7.5 = 480
#define WAKES_PER_MINUTE  (WAKES_PER_HOUR / 60) // 8 wakes per minute

/*
   Target window: 18 to 36 hours
   Min wakes = 18 * 480 = 8640
   Max wakes = 36 * 480 = 17280
   Varince range = 17280 - 8640 = 8640
*/
#define MIN_WAKES       (18 * WAKES_PER_HOUR)
#define MAX_WAKES       (36 * WAKES_PER_HOUR)
#define VARIANCE_RANGE  (MAX_WAKES - MIN_WAKES)

volatile uint16_t wake_counter = 0;
uint16_t current_target = MIN_WAKES;  // Initial target
static uint16_t lfsr_state = 0xACE1;  // Seed for random generator


ISR(WDT_vect) {
  wake_counter++;
}

// Galois LFSR
uint16_t get_random_variance() {
  lfsr_state ^= TCNT0;
  uint8_t lsb = lfsr_state & 1;
  lfsr_state >>= 1;
  if (lsb) {
    lfsr_state ^= 0xB400;   // Tap polynomial
  }
  return lfsr_state % VARIANCE_RANGE;
}

void tone_burst_100hz(uint8_t duration_ms) {
  uint16_t cycles = duration_ms / 10;  // 10 ms = 1 cycle

  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << PB3);
    PORTB &= ~(1 << PB4);
    _delay_ms(5);

    PORTB &= ~(1 << PB3);
    PORTB |= (1 << PB4);
    _delay_ms(5);
  }

  // Set PB3 and PB4 LOW to save power
  PORTB &= ~((1 << PB3) | (1 << PB4));
}

void tone_burst_4khz(uint8_t duration_ms) {
  uint16_t cycles = duration_ms * 4;  // 1 ms = 4 cycles

  for (uint16_t i = 0; i < cycles; i++) {
    PORTB |= (1 << PB3);
    PORTB &= ~(1 << PB4);
    _delay_us(125);

    PORTB &= ~(1 << PB3);
    PORTB |= (1 << PB4);
    _delay_us(125);
  }

  // Set PB3 and PB4 LOW to save power
  PORTB &= ~((1 << PB3) | (1 << PB4));
}

void make_cricket_chirp() {
  // Chirp 1
  for (uint8_t i = 0; i < 10; i++) {
    tone_burst_4khz(15);
    _delay_us(40);
  }
  _delay_ms(300);

  // Chirp 2
  for (uint8_t i = 0; i < 10; i++) {
    tone_burst_4khz(15);
    _delay_us(40);
  }
}

void power_on_beep() {
  tone_burst_100hz(250);
  _delay_ms(100);
  tone_burst_100hz(250);
}

void setup() {
  // --- PIN CONFIGURATION ---
  // PB3, PB4 output
  // PB0, PB1, PB2 as input
  DDRB |= (1 << PB3) | (1 << PB4);
  DDRB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2));

  // Enable pull-ups in unused inputs
  PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB2);

  // Ensure beeper starts off
  PORTB &= ~((1 << PB3) | (1 << PB4));


  // --- POWER SAVING ---
  // Disable Analog Comparator (saves ~70 uA)
  ACSR |= (1 << ADC);

  // Disable ADC (saves ~200 uA)
  ADCSRA &= ~(1 << ADEN);


  // --- WATCHDOG TIMER SETUP ---
  // Enable interrupt mode
  // Prescaler: 8 seconds (WDP3 = 1, WDP0 = 1)
  MCUSR &= ~(1 << WDRF);              // Clear reset flag
  WDTCR |= (1 << WDCE) | (1 << WDE);  // Start timed sequence

  // Set Interrupt Enable and 8s prescaler
  WDTCR = (1 << WDTIE) | (1 << WDP3) | (1 << WDP0);

  // Enable global interrupts
  sei();


  // --- TIMER 0 ---
  TCCR0B |= (1 << CS00);

}

void loop() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_on_beep();

  while (1) {
    if (wake_counter >= current_target) {
      make_cricket_chirp();
      current_target = MIN_WAKES + get_random_variance();
      wake_counter = 0;
    }

    sleep_enable();
    sleep_cpu();  // Halt here until watchdog interrupt fires
    sleep_disable();
  }
}
