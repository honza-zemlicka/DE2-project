// -Eppur si muove-  line-follower v01-04

#include <avr/io.h>        // AVR device-specific IO definitions
#include <avr/interrupt.h> // Interrupts standard C library for AVR-GCC
#include <stdio.h>         // C library. Needed for `sprintf`
//#include "timer.h"
#include <gpio.h>
#include <util/delay.h>
#include <oled.h>
#include "robot_definitions.h"

uint16_t S1K[2] = {0, 0};
uint16_t S2K[2] = {0, 0};
uint16_t S3K[2] = {0, 0};
uint16_t S4K[2] = {0, 0};

// 1000 White - 300 Black

// [0] white
// [1] black

int8_t error, correction;

uint8_t S1, S2, S3, S4;

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
static uint8_t constrain(int8_t val, uint8_t min, uint8_t max);

void auto_calibration();
void robot_stop();

volatile uint8_t calib_tick = 0;

uint8_t motor_speed = 255;
uint8_t gain_kp = 10;   // jako float, ale jak zobrazit na displeji?
char display_buffer[16];

uint8_t robot_run = 0;

volatile uint32_t ir_data = 0;
volatile uint8_t bit_index = 0;
volatile uint32_t frame_received = 0;
volatile uint8_t rx_bytes = 0;

volatile uint8_t last_time = 0;
volatile uint8_t measured = 0;
volatile uint8_t length = 0;
volatile uint8_t reception_complete = 0;

const char *encode_cmd(uint8_t command);

const char *message;

void update_display()
{
  oled_gotoxy(17, 0);
  sprintf(display_buffer, "%2d", gain_kp);
  oled_puts(display_buffer);

  oled_gotoxy(15, 2);
  sprintf(display_buffer, "%3d", motor_speed);
  oled_puts(display_buffer);

  oled_display();
}

void timer2_init()
{
  TCCR2A = 0;                                       // normal mode
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024, 64us
  TCNT2 = 0;
}

void int0_init()
{
  EICRA = (1 << ISC01) | (1 << ISC00); // INT0 = rising edge
  EIMSK = (1 << INT0);                 // enable INT0
}

ISR(INT0_vect)
{
  uint8_t measured = TCNT2;
  uint8_t length = measured - last_time;

  // Start bit
  if (length > 60 && length < 100) // 5 062 us / 64 = 79 tics
  {
    ir_data = 0;
    bit_index = 0;
    frame_received = 0;
  }

  // High
  if (length > 25 && length < 45) // 2 249 us / 64 = 35 tics
  {
    // ir_data |= (1 << bit_index);
    ir_data <<= 1;
    ir_data |= 1;
    bit_index++;
  }

  // Low
  if (length > 7 && length < 25) // 1 124 us / 64 = 17,5 tics
  {
    // ir_data |= (0 << bit_index);
    ir_data <<= 1;
    bit_index++;
  }

  if (bit_index == frame_length)
  {
    EIMSK &= ~(1 << INT0); // disable INT0
    rx_bytes++;
    gpio_toggle(&PORTB, PB5);

    TCNT2 = 0;
    reception_complete = 1;
    encode_frame();
    // return;
  }
  last_time = measured;
}

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

  timer2_init();
  int0_init();
  sei();

  twi_init();
  oled_init(OLED_DISP_ON);
  oled_clrscr();
  oled_charMode(DOUBLESIZE);

  oled_gotoxy(0, 0);
  oled_puts("Kp    =");
  oled_gotoxy(0, 2);
  oled_puts("Speed =");
  update_display();

  adc_init();

  /*float gain_kp = 40;
  uint8_t motor_speed = 200;*/

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
    correction = error * gain_kp;

    pwm_write(&PORTD, MOTOR_LF, motor_speed + correction);
    pwm_write(&PORTD, MOTOR_RF, motor_speed - correction);

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

/*void ultrasound()
{

}*/