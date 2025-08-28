#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "waveforms.hpp"

#define SAMPLE_RATE 31372   // Timer2 Fast PWM (~31.372 kHz at 16 MHz)
#define TABLE_SIZE 256

volatile uint32_t phaseAcc1 = 0;
volatile uint32_t phaseAcc2 = 0;
uint32_t phaseStep1, phaseStep2;

ISR(TIMER2_OVF_vect) {
  phaseAcc1 += phaseStep1;
  phaseAcc2 += phaseStep2;

  uint8_t sample1 = pgm_read_byte(&sineTable[phaseAcc1 >> 24]);
  uint8_t sample2 = pgm_read_byte(&sineTable[phaseAcc2 >> 24]);

  uint16_t mixed = sample1 + sample2;   // Mischung (0-510)
  OCR2B = mixed >> 2;                   // auf 8 Bit begrenzen (0-255)
}

void setFrequency1(float freq) {
  phaseStep1 = (uint32_t)((freq * (1ULL << 32)) / SAMPLE_RATE);
}
void setFrequency2(float freq) {
  phaseStep2 = (uint32_t)((freq * (1ULL << 32)) / SAMPLE_RATE);
}

int main() {
  DDRB |= (1 << PB3);

  // Timer2 Fast PWM setup (Pin 3 = OC2B)
  TCCR2A = _BV(COM2B1) | _BV(WGM20) | _BV(WGM21); // Fast PWM, OC2B active
  TCCR2B = _BV(WGM22) | _BV(CS20);                // Prescaler 1
  OCR2A = (F_CPU / SAMPLE_RATE) - 1;              // Top-Wert
  OCR2B = 128;                                    // Start = Mitte
  TIMSK2 = _BV(TOIE2);                            // Interrupt aktivieren

  setFrequency1(440.0);  // A4
  setFrequency2(659.0);  // E5
  
  sei();
  
  while(true) {
	  
  }
  
  return 0;
}