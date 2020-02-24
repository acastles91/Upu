/*
   - Applies to Arduino-Due board, PWM pins only 6, 7, 8 & 9.
   - Libary does not operate on the TIO pins.
   - Unique frequencies set via PWM Clock-A ("CLKA") and Clock-B ("CLKB")
     Therefore, up to two unique frequencies allowed.
   - Set max duty cycle counts (pwm_max_duty_Ncount) equal to 255
     per Arduino approach: this value is best SUITED for low frequency
     applications (2hz to 40,000hz) such as PWM motor drivers,
     38khz infrared transmitters, etc.
   - Future library versions will address high frequency applications.
   - Arduino's "wiring_analog.c" function was very helpful in this effort.
*/

#include "pwm01.h"

int pwm_pin6 = 6;
int pwm_pin7 = 7;
int pwm_pin8 = 8;
int pwm_pin9 = 9;

uint32_t  pwm_period1 = 200;
uint32_t  pwm_period2 = 300;
uint32_t  pwm_freq1 = 400;
uint32_t  pwm_freq2 = 500;
uint32_t  pwm_duty = 32767;                     // = 50% Duty cycle

void setup() {
  pwm_set_resolution(16);                       // Set PWM Resolution

  pwm_setup(pwm_pin6, pwm_freq1, 1);            // Pin 6 freq set to "pwm_freq1" on clock A
  pwm_setup(pwm_pin7, pwm_freq1, 1);            // Pin 7 freq set to "pwm_freq1" on clock A
  pwm_setup(pwm_pin8, pwm_freq2, 1);            // Pin 8 freq set to "pwm_freq2" on clock A
  pwm_setup(pwm_pin9, pwm_freq2, 2);            // Pin 9 freq set to "pwm_freq2" on clock B

  pwm_write_duty(pwm_pin6, pwm_duty);           // Write duty Cycle After PWM Setup on Pin 6
  pwm_write_duty(pwm_pin7, pwm_duty);           // Write duty Cycle After PWM Setup on Pin 7
  pwm_write_duty(pwm_pin8, pwm_duty);           // Write duty Cycle After PWM Setup on Pin 8
  pwm_write_duty(pwm_pin9, pwm_duty);           // Write duty Cycle After PWM Setup on Pin 9

  delay(5000);                                  // PWM continues to stream in the background!

  pwm_set_period(pwm_pin8, pwm_period1);        // set period to "pwm_period1" on Pin 8
  delay(500);
  pwm_set_period(pwm_pin8, pwm_period2);        // set period to "pwm_period2" on Pin 8
  delay(500);
  pwm_set_clockA_freq(pwm_freq1);               // set freq to "pwm_freq1" on clock A
  delay(500);
  pwm_set_clockB_freq(pwm_freq2);               // set freq to "pwm_freq2" on clock B
  delay(500);
  pwm_set_clockAB_freqs(pwm_freq2, pwm_freq1);  // set freq to "pwm_freq2" on Clock A and "pwm_freq1" on clock B
  delay(500);
  pwm_setup(pwm_pin8, pwm_freq1, 2);            // configure clock B as source for Pin 8 with freq "pwm_freq1" 
  pwm_write_duty(pwm_pin8, pwm_duty);           // activate Pin 8 and write duty cycle
  delay(500);

  pwm_stop(pwm_pin6);                           // Stop PWM on Pin 6
  pwm_stop(pwm_pin7);                           // Stop PWM on Pin 7
  pwm_stop(pwm_pin8);                           // Stop PWM on Pin 8
  pwm_stop(pwm_pin9);                           // Stop PWM on Pin 9
}

void loop() {}
