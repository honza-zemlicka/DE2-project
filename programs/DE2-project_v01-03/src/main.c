// -Eppur si muove-  line-follower v01-01
#include <avr/io.h>        // AVR device-specific IO definitions
#include <avr/interrupt.h> // Interrupts standard C library for AVR-GCC
#include <stdio.h>         // C library. Needed for `sprintf`
#include "timer.h"
#include <gpio.h>
// #include <oled.h>
#include <uart.h>
#include <util/delay.h>

// I2C adresses: RGB 0x29, OLED 0x3c

#define MOTOR_LF PD5 // Arduino D5 (PWM) - left front
#define MOTOR_RF PD6 // Arduino D6 (PWM) - right front
#define MOTOR_LB PB1 // Arduino D9 (PWM) - left back
#define MOTOR_RB PB2 // Arduino D10 (PWM) - right back

#define SENSOR_RR PC0 // Arduino A0
#define SENSOR_CR PC1 // Arduino A1
#define SENSOR_CL PC2 // Arduino A2
#define SENSOR_LL PC3 // Arduino A3

#define USER_LED PB5 // Arduino D13
#define BUTTON PD3   // Arduino D3, pullup

/*
#define EYES1_TRIG
#define EYES1_ECHO
*/

// variables

// calibration arrays: [0] = white, [1] = black
// typical values: 1000 white, 300 black
uint16_t S1K[2] = {0, 0};
uint16_t S2K[2] = {0, 0};
uint16_t S3K[2] = {0, 0};
uint16_t S4K[2] = {0, 0};

int8_t error, correction;
uint8_t S1, S2, S3, S4;

// calibration timer counter
volatile uint8_t calib_tick = 0;

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
static uint8_t constrain(int8_t val, uint8_t min, uint8_t max);

void auto_calibration();
void robot_stop();

int main(void)
{
  gpio_mode_output(&DDRD, MOTOR_LF);
  gpio_mode_output(&DDRD, MOTOR_RF);
  gpio_mode_output(&DDRB, MOTOR_LB);
  gpio_mode_output(&DDRB, MOTOR_RB);
  gpio_mode_output(&DDRB, USER_LED);

  gpio_mode_input(&DDRC, SENSOR_CR);
  gpio_mode_input(&DDRC, SENSOR_CL);
  gpio_mode_input(&DDRC, SENSOR_RR);
  gpio_mode_input(&DDRC, SENSOR_LL);
  gpio_mode_input_pullup(&DDRD, BUTTON);

  adc_init();

  // control variables
  float Kp = 40;
  uint8_t speed = 200;

  tim2_ovf_16ms()        // set overflow
      tim2_ovf_enable(); // enable overflow interrupt
  sei();                 // enables global interrupts

  pwm_init();
  pwm_write(&PORTD, MOTOR_LF, 0);
  pwm_write(&PORTD, MOTOR_RF, 0);
  pwm_write(&PORTB, MOTOR_LB, 0);
  pwm_write(&PORTB, MOTOR_RB, 0);

  _delay_ms(800);
  auto_calibration();

  // main loop
  while (1)
  {
    // read sensors and map values to 0-100 range based on calibration
    S1 = map(analog_read(SENSOR_CR), S1K[1], S1K[0], 0, 100);
    S2 = map(analog_read(SENSOR_CL), S2K[1], S2K[0], 0, 100);
    S3 = map(analog_read(SENSOR_RR), S3K[1], S3K[0], 0, 100);
    S4 = map(analog_read(SENSOR_LL), S4K[1], S4K[0], 0, 100);

    // calculate difference between center sensors
    error = S1 - S2;
    correction = error * Kp;

    // apply motor power with correction
    pwm_write(&PORTD, MOTOR_LF, speed + correction);
    pwm_write(&PORTD, MOTOR_RF, speed - correction);

    /*if(count == 20)
    {
      ultrasound();
      count = 0;
    }*/

    if (gpio_read(&PIND, BUTTON) == 0) // button pressed
    {
      _delay_ms(50); // debounce

      if (gpio_read(&PIND, BUTTON) == 0) // check if still pressed
      {
        while (gpio_read(&PIND, BUTTON) == 0) // waiting for button release
        {
          _delay_ms(10);
        }
        robot_stop();
      }
    }
  }
  return 0;
}

// functions

void auto_calibration()
{
  gpio_write_high(&PORTB, USER_LED); // indicates calibration start

  // initialize max (white) and min (black) with current values
  S1K[0] = S1K[1] = analog_read(SENSOR_CR);
  S2K[0] = S2K[1] = analog_read(SENSOR_CL);
  S3K[0] = S3K[1] = analog_read(SENSOR_RR);
  S4K[0] = S4K[1] = analog_read(SENSOR_LL);

  calib_tick = 0;

  // spins robot for a duration controlled by Timer2 ISR
  while (calib_tick < 200)
  {
    pwm_write(&PORTB, MOTOR_LB, 120);
    pwm_write(&PORTD, MOTOR_RF, 120);

    S1 = analog_read(SENSOR_CR);
    S2 = analog_read(SENSOR_CL);
    S3 = analog_read(SENSOR_RR);
    S4 = analog_read(SENSOR_LL);

    // update max (white) and min (black) values
    if (S1 > S1K[0])
      S1K[0] = S1;
    else if (S1 < S1K[1])
      S1K[1] = S1;

    if (S2 > S2K[0])
      S2K[0] = S2;
    else if (S2 < S2K[1])
      S2K[1] = S2;

    if (S3 > S3K[0])
      S3K[0] = S3;
    else if (S3 < S3K[1])
      S3K[1] = S3;

    if (S4 > S4K[0])
      S4K[0] = S4;
    else if (S4 < S4K[1])
      S4K[1] = S4;
  }

  // stop motors after calibration
  pwm_write(&PORTB, MOTOR_LB, 0);
  pwm_write(&PORTD, MOTOR_RF, 0);

  // waits for button press to break calibration loop
  while (1)
  {
    if (gpio_read(&PIND, BUTTON) == 0)
    {
      gpio_write_low(&PORTB, USER_LED);
      break;
    }
  }
}

// map function to remap a number between ranges
static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
  return (int32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// constrain function for limiting value in a given range
static uint8_t constrain(int8_t val, uint8_t min, uint8_t max)
{
  uint8_t motor = 0;
  if (val <= min)
    motor = min;
  else if (val >= max)
    motor = max;
  else
    motor = val;
  return motor;
}

ISR(TIMER2_OVF_vect)
{
  calib_tick++; // increments counter for calibration timing
}

void robot_stop()
{
  pwm_write(&PORTD, MOTOR_LF, 0);
  pwm_write(&PORTD, MOTOR_RF, 0);
  pwm_write(&PORTB, MOTOR_LB, 0);
  pwm_write(&PORTB, MOTOR_RB, 0);

  gpio_write_high(&PORTB, USER_LED); // stop indication

  // loop until button is pressed again to resume
  while (1)
  {
    if (gpio_read(&PIND, BUTTON) == 0) // button pressed
    {
      _delay_ms(50);                     // debounce
      if (gpio_read(&PIND, BUTTON) == 0) // check if still pressed
      {
        while (gpio_read(&PIND, BUTTON) == 0)
        { // waiting for button release
          _delay_ms(10);
        }
        break; // exit to main loop
      }
    }
    _delay_ms(100);
  }

  gpio_write_low(&PORTB, USER_LED);
}