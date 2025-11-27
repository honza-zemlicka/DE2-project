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

#define MOTOR_L PD5 // Arduino D5 (PWM)
#define MOTOR_R PD6 // Arduino D6 (PWM)

#define SENSOR_RR PC0 // Arduino A0
#define SENSOR_CR PC1 // Arduino A1
#define SENSOR_CL PC2 // Arduino A2
#define SENSOR_LL PC3 // Arduino A3

#define USER_LED PB5 // Arduino D13
#define BUTTON PD7   // Arduino D7, pullup

/*
#define EYES1_TRIG
#define EYES1_ECHO
*/

/*uint16_t S1K[2] = {100, 0};
uint16_t S2K[2] = {100, 0};
uint16_t S3K[2] = {100, 0};
uint16_t S4K[2] = {100, 0};*/

uint16_t S1K[2] = {1024, 0};
uint16_t S2K[2] = {1024, 0};
uint16_t S3K[2] = {1024, 0};
uint16_t S4K[2] = {1024, 0};

// [0] white
// [1] black

int8_t error, korekce, paused = 0;

uint16_t S1, S2, S3, S4;

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
static inline int16_t constrain(uint16_t val, uint16_t min, uint16_t max);
void calibration();

volatile uint8_t count = 0;

int main(void)
{
  gpio_mode_output(&DDRD, MOTOR_L);
  gpio_mode_output(&DDRD, MOTOR_R);
  gpio_mode_output(&DDRB, USER_LED);

  gpio_mode_input(&DDRC, SENSOR_CR);
  gpio_mode_input(&DDRC, SENSOR_CL);
  gpio_mode_input(&DDRC, SENSOR_RR);
  gpio_mode_input(&DDRC, SENSOR_LL);
  gpio_mode_input_pullup(&DDRD, BUTTON);

  adc_init();

  float Kp = 30;
  uint8_t rychlost = 95;
  float treshold = 1.2;

  tim2_ovf_16ms()       // set overflow
  tim2_ovf_enable();    // Enable overflow interrupt
  sei();                // Enables interrupts by setting the global interrupt mask

  //_delay_ms(1000);
  calibration();
  //settings();

  pwm_init();
  pwm_write(&PORTD, MOTOR_L, 0);
  pwm_write(&PORTD, MOTOR_L, 0);

  while (1)
  {
    S1 = map(analog_read(SENSOR_CR), S1K[0], S1K[1], 0, 100);
    S2 = map(analog_read(SENSOR_CL), S2K[0], S2K[1], 0, 100);
    S3 = map(analog_read(SENSOR_RR), S3K[0], S3K[1], 0, 100);
    S4 = map(analog_read(SENSOR_LL), S4K[0], S4K[1], 0, 100);

    error = S1 - S2;
    korekce = error * Kp;

    // pwm_write(&PORTD, MOTOR_L, constrain(rychlost - korekce, 0, 255));
    // pwm_write(&PORTD, MOTOR_R, constrain(rychlost + korekce, 0, 255));

    pwm_write(&PORTD, MOTOR_L, rychlost + korekce);
    pwm_write(&PORTD, MOTOR_R, rychlost - korekce);

    /*if (S3 < 40)
    {
      // pwm_write(&PORTD, MOTOR_L, rychlost + 2*korekce);
      // pwm_write(&PORTD, MOTOR_R, rychlost - 2*korekce);
    }
    else if (S4 < 40)
    {
      // pwm_write(&PORTD, MOTOR_L, rychlost + 2*korekce);
      // pwm_write(&PORTD, MOTOR_R, rychlost - 2*korekce);
    }
    else*/

    /*if (gpio_read(&PIND, BUTTON) == 0)
    {
      _delay_ms(30);
      if (gpio_read(&PIND, BUTTON) == 0)
      {
        while (1)
        {
          if (gpio_read(&PIND, BUTTON) == 0)
          {
            _delay_ms(30);
            if (gpio_read(&PIND, BUTTON) == 0)
              break;
          }
        }
      }
    }*/

    /*if (count == 20)
    {
      ultrasound();
      rgb_color();
      count = 0;
    }*/
  }
  return 0;
}

void calibration()
{
  gpio_write_high(&PORTB, USER_LED);

  while (1)
  {
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

    if (gpio_read(&PIND, BUTTON) == 0)
    {
      gpio_write_low(&PORTB, USER_LED);
      break;
    }
  }
}

/*void settings()
{
  if(command = xx)
    rychlost += 10;
  if(command = xx)
    rychlost -= 10;
  if(command = xx)
    zesileni += 10;
  if(command = xx)
    zesileni -= 10;
}*/
/*
zvlast tlacitko pro rychlost +10, -10, +1, -1
                    zesileni +10, -10, +1, -1
                    spusteni
                    zastaveni
*/

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
  return (int32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline int16_t constrain(uint16_t val, uint16_t min, uint16_t max)
{
  if (val < 0)
    val = 0;
  else if (val > 255)
    val = 255;
  else
    val;
}


ISR(TIMER2_OVF_vect)
{
  count++;
}


