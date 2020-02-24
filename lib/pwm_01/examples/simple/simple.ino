/*  
   - Applies to Arduino-Due board, PWM pins 6, 7, 8 & 9.
   - Libary Does not operate on the TIO pins.
   - Unique frequencies set via PWM Clock-A ("CLKA") and Clock-B ("CLKB")
     Therefore, up to two unique frequencies allowed.
   - Set max duty cycle counts (pwm_max_duty_Ncount) equal to 255
     per Arduino approach.  This value is best SUITED for low frequency
     applications (2hz to 40,000hz) such as PWM motor drivers, 
     38khz infrared transmitters, etc.
   - Future library versions will address high frequency applications.
   - Arduino's "wiring_analog.c" function was very helpful in this effort.
*/

#include "pwm01.h"

void setup()
{
   uint32_t  pwm_duty = 32767;
   uint32_t  pwm_freq1 = 5;
   uint32_t  pwm_freq2 = 5000;

   // Set PWM Resolution
   pwm_set_resolution(16);  

   // Setup PWM Once (Up to two unique frequencies allowed
   //-----------------------------------------------------    
   pwm_setup( 6, pwm_freq1, 1);  // Pin 6 freq set to "pwm_freq1" on clock A
   pwm_setup( 7, pwm_freq2, 2);  // Pin 7 freq set to "pwm_freq2" on clock B
   pwm_setup( 8, pwm_freq2, 2);  // Pin 8 freq set to "pwm_freq2" on clock B
   pwm_setup( 9, pwm_freq2, 2);  // Pin 9 freq set to "pwm_freq2" on clock B
     
   // Write PWM Duty Cycle Anytime After PWM Setup
   //-----------------------------------------------------    
   pwm_write_duty( 6, pwm_duty );  // 50% duty cycle on Pin 6
   pwm_write_duty( 7, pwm_duty );  // 50% duty cycle on Pin 7
   pwm_write_duty( 8, pwm_duty );  // 50% duty cycle on Pin 8
   pwm_write_duty( 9, pwm_duty );  // 50% duty cycle on Pin 9

   delay(30000);  // 30sec Delay; PWM signal will still stream
       
   // Force PWM Stop On All Pins
   //-----------------------------    
   pwm_stop( 6 );
   pwm_stop( 7 );
   pwm_stop( 8 );
   pwm_stop( 9 );
}

void loop()
{  
}

