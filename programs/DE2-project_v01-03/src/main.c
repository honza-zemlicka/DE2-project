// -Eppur si muove-  line-follower v01-01

#include <avr/io.h>        // AVR device-specific IO definitions
#include <avr/interrupt.h> // Interrupts standard C library for AVR-GCC
#include <stdio.h>         // C library. Needed for `sprintf`
#include "timer.h"
#include <gpio.h>
// #include <oled.h>
#include <uart.h>
#include <util/delay.h>

// pro I2C -> RGB adresa 0x29, OLED adresa 0x3c

#define MOTOR_LF PD5 // Arduino D5 (PWM)
#define MOTOR_RF PD6 // Arduino D6 (PWM)
#define MOTOR_LB PB1 // Arduino D9 (PWM)
#define MOTOR_RB PB2 // Arduino D10 (PWM)

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

uint16_t S1K[2] = {0, 0};
uint16_t S2K[2] = {0, 0};
uint16_t S3K[2] = {0, 0};
uint16_t S4K[2] = {0, 0};

// 1000 White - 300 Black

// [0] white
// [1] black

int8_t error, korekce;

uint8_t S1, S2, S3, S4;

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
static uint8_t constrain(int8_t val, uint8_t min, uint8_t max);

void auto_calibration();
void robot_stop();

volatile uint8_t calib_tick = 0;

int main(void)
{
  // adc_init();

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

  float Kp = 40;
  uint8_t rychlost = 200;

  tim2_ovf_16ms()        // set overflow
  tim2_ovf_enable(); // Enable overflow interrupt
  sei();                 // Enables interrupts by setting the global interrupt mask

  pwm_init();
  pwm_write(&PORTD, MOTOR_LF, 0);
  pwm_write(&PORTD, MOTOR_RF, 0);
  pwm_write(&PORTB, MOTOR_LB, 0);
  pwm_write(&PORTB, MOTOR_RB, 0);

  _delay_ms(800);
  auto_calibration();

  while (1)
  {
    S1 = map(analog_read(SENSOR_CR), S1K[1], S1K[0], 0, 100);
    S2 = map(analog_read(SENSOR_CL), S2K[1], S2K[0], 0, 100);
    S3 = map(analog_read(SENSOR_RR), S3K[1], S3K[0], 0, 100);
    S4 = map(analog_read(SENSOR_LL), S4K[1], S4K[0], 0, 100);

    error = S1 - S2;
    korekce = error * Kp;

    pwm_write(&PORTD, MOTOR_LF, rychlost + korekce);
    pwm_write(&PORTD, MOTOR_RF, rychlost - korekce);

    /*if(count == 20)
    {
      ultrasound();
      count = 0;
    }*/

    if (gpio_read(&PIND, BUTTON) == 0)
    {
      // pridat debounce
      _delay_ms(300);
      robot_stop();
    }
  }
  return 0;
}

void auto_calibration()
{
  gpio_write_high(&PORTB, USER_LED);

  S1K[0] = S1K[1] = analog_read(SENSOR_CR);
  S2K[0] = S2K[1] = analog_read(SENSOR_CL);
  S3K[0] = S3K[1] = analog_read(SENSOR_RR);
  S4K[0] = S4K[1] = analog_read(SENSOR_LL);

  calib_tick = 0;

  while (calib_tick < 200)
  {
    pwm_write(&PORTB, MOTOR_LB, 120);
    pwm_write(&PORTD, MOTOR_RF, 120);

    S1 = analog_read(SENSOR_CR);
    S2 = analog_read(SENSOR_CL);
    S3 = analog_read(SENSOR_RR);
    S4 = analog_read(SENSOR_LL);

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
  pwm_write(&PORTB, MOTOR_LB, 0);
  pwm_write(&PORTD, MOTOR_RF, 0);

  while (1)
  {
    if (gpio_read(&PIND, BUTTON) == 0)
    {
      gpio_write_low(&PORTB, USER_LED);
      break;
    }
  }
}

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
  return (int32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
  calib_tick++;
}

void robot_stop()
{
  while (gpio_read(&PIND, BUTTON) == 1)
  {
    _delay_ms(300);
    // pridat debounce
    gpio_write_high(&PORTB, USER_LED);
    pwm_write(&PORTD, MOTOR_LF, 0);
    pwm_write(&PORTD, MOTOR_RF, 0);
    pwm_write(&PORTB, MOTOR_LB, 0);
    pwm_write(&PORTB, MOTOR_RB, 0);
  }
  gpio_write_low(&PORTB, USER_LED);
}