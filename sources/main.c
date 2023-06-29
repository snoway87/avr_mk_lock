#ifndef __AVR_ATmega32__
#define __AVR_ATmega32__
#endif

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#define SENSORS 4
#define SENSOR1_PIN (1 << PA0)
#define SENSOR2_PIN (1 << PA2)
#define SENSOR3_PIN (1 << PA4)
#define SENSOR4_PIN (1 << PA6)

#define SENSOR1_ECHO (1 << PA1)
#define SENSOR2_ECHO (1 << PA3)
#define SENSOR3_ECHO (1 << PA5)
#define SENSOR4_ECHO (1 << PA7)


volatile uint8_t sensors_on[SENSORS];
volatile uint16_t sensors_time[SENSORS];
volatile float actual_distance = -1.;
volatile float target_distance = 20.;

void
sensors_start(uint8_t sensor_pin) {
  do {
    /* Запущен ли датчик? */
    if(sensor_pin == SENSOR1_PIN && sensors_on[0]) break;
    if(sensor_pin == SENSOR2_PIN && sensors_on[1]) break;
    if(sensor_pin == SENSOR3_PIN && sensors_on[2]) break;
    if(sensor_pin == SENSOR4_PIN && sensors_on[3]) break;
    /* Отправляем пачку импульсов на датчик */
    for(int i = 0; i < 10; i ++ ) {
      PORTA |= sensor_pin;
      while(1) { _delay_us(10); }
      PORTA &= ~sensor_pin;
      while(1) { _delay_us(10); }
    }
    /* Сбрасываем  время замера */
    for(int i = 0; i < SENSORS; i ++ ) {
      ATOMIC_BLOCK(ATOMIC_FORCEON) {
        sensors_time[i] = 0;
      }
    }
    /* Включаем датчик */
    if(sensor_pin == SENSOR1_PIN) sensors_on[0] = 1;
    if(sensor_pin == SENSOR2_PIN) sensors_on[1] = 1;
    if(sensor_pin == SENSOR3_PIN) sensors_on[2] = 1;
    if(sensor_pin == SENSOR4_PIN) sensors_on[3] = 1;
  }while(0);
}

#define BUTTON_DELAY  20            /* Защита от дребезга 20мс */
#define BUTTONS 4
#define BUTTON_SB1_PIN (1 << PC4)   /* Войти/выйти в режим конфигурации */
#define BUTTON_SB2_PIN (1 << PC5)   /* Добавить расстояние +10м */
#define BUTTON_SB3_PIN (1 << PC6)   /* Уменьшить расстояние -10м */
#define BUTTON_SB4_PIN (1 << PC7)   /* Сбросить расстояние в 0 */

#define IC_MODE_IDLE    0
#define IC_MODE_CONFIG  1
volatile uint8_t ic_mode;           /* Режим микроконтроллера */

uint8_t
get_button_state(uint8_t button_pin, uint8_t * button_delay) {
  #define MS 1
  if(!(PINC & button_pin))
    *button_delay += MS;
  else
    if(*button_delay)
      *button_delay -= MS;
  return (*button_delay >= BUTTON_DELAY)? 1 : 0;
}

//
// Timer Interrupt
// -----------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  static uint8_t button_delay[BUTTONS];
  static float sensors_distance[SENSORS];

  /* Обработка датчиков */
  for(int i = 0; i < SENSORS; i ++ ) {
    /* Если датчик включен и на порту ECHO уровень */
    if(sensors_on[i] && (PINA & (1 << (i + 1)))) {
      sensors_time[i] ++; /* +1 мс */
    } else
    /* Если датчик включен и на порту ECHO нет уровня */
    if(sensors_on[i] && sensors_time[i] && !(PINA & (1 << (i + 1)))) {
      sensors_distance[i] = ((float) sensors_time[i] * 16.5);   /* считаем расстояние в см */
      sensors_on[i] = 0;                                        /* отключаем датчик */
      if(actual_distance < 0.)
        actual_distance = sensors_distance[i];
      else {
        if(actual_distance > sensors_distance[i])
          actual_distance = sensors_distance[i];
      }
    }
  }

  /* Обработка кнопок */
  if(get_button_state(BUTTON_SB1_PIN, &button_delay[0])) {
    /* Выбираем режим работы */
    ic_mode = (ic_mode == IC_MODE_CONFIG)? IC_MODE_IDLE : IC_MODE_CONFIG;
  }
  /* В режиме конфигурации */
  if(ic_mode == IC_MODE_CONFIG) {
    /* Увеличивыем дистацию */
    if(get_button_state(BUTTON_SB2_PIN, &button_delay[0])) {
      target_distance += (target_distance >= 200.)? 10. : 200.;
    }
    /* Уменьшаем дистацию */
    if(get_button_state(BUTTON_SB3_PIN, &button_delay[0])) {
      target_distance -= (target_distance >= 10.)? 10. : 0.;
    }
    /* Сбрасываем в 0 */
    if(get_button_state(BUTTON_SB4_PIN, &button_delay[0])) target_distance = 0.;
  }
}

//
// LCD
// -----------------------------------------------------
#define LCD_EN (1 << PC2)
#define LCD_RS (1 << PC0)
#define LCD_RW (1 << PC1)

void
lcd_command(uint8_t ch) {
  PORTD = ch;
  PORTC &= ~(LCD_RS | LCD_RW);
  PORTC |= LCD_EN;
	_delay_us(1);
  PORTC &= ~LCD_EN;
	_delay_ms(3);
}

void
lcd_char(uint8_t ch) {
  PORTD = ch;
  PORTC |= LCD_RS;
  PORTC &= ~LCD_RW;
  PORTC |= LCD_EN;
  _delay_us(1);
  PORTC &= ~LCD_EN;
	_delay_ms(1);
}

void
lcd_print (char *str)
{
  for(int i = 0; str[i]!= 0; i++)
    lcd_char(str[i]);
}

void
lcd_init (void)
{
	_delay_ms(20);
	lcd_command (0x38);	/* 8 битный режим */
	lcd_command (0x0c);	/* Включаем дисплей */
	lcd_command (0x06);	/* Автоинкримент курсора */
	lcd_command (0x01);	/* Очищаем дисплей */
	lcd_command (0x80);	/* Курсор на насчало */
}

//
// Entry point
// -----------------------------------------------------
int
main( void ) {
  /* Конфигурация портов датчиков */
  DDRA |= (SENSOR1_PIN | SENSOR2_PIN | SENSOR3_PIN | SENSOR4_PIN);
  DDRA &= ~(SENSOR1_ECHO | SENSOR2_ECHO | SENSOR3_ECHO | SENSOR4_ECHO);

  /* Конфигурация портов кнопок и LCD */
  DDRC &= ~(BUTTON_SB1_PIN |
            BUTTON_SB2_PIN |
            BUTTON_SB3_PIN |
            BUTTON_SB4_PIN |
            LCD_EN | LCD_RS | LCD_RW);

  /* Конфигурация PB4 в 1 */
  DDRB |= (1 << PB4);
  PORTB |= (1 << PB4);

  /* Timer1 (16 bit) - 1000Hz */
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);           /* Делитель -> 16Mhz/64 */
  OCR1A = 249;                                                  /* Прерывание по совпадению (16Mhz/64/1000Hz ~ 250 - 1 = 249) */
  TIMSK |= ( 1 << OCIE1A );                                     /* Включаем прерывание */

  lcd_init();
  sei();

  float _target_distance;
  float _actual_distance;
  char lcd_string[128];
  while(1) {
    /* Читаем текущие значения дистанции */
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      _target_distance = target_distance;
      _actual_distance = actual_distance;
    }

    /* Общий режим работы  */
    if(ic_mode == IC_MODE_IDLE) {
      sensors_start( SENSOR2_PIN );
      sensors_start( SENSOR3_PIN );
      sensors_start( SENSOR1_PIN );
      sensors_start( SENSOR4_PIN );

      /* Управляем PB4 от дистанции */
      if(_actual_distance <= _target_distance && _actual_distance > _target_distance - 5.) {
        PORTB &= ~(1 << PB4);
      } else {
        PORTB |= (1 << PB4);
      }
      /* Отображаем данные на LCD */
      sprintf(lcd_string, "%.1f cm", _actual_distance);
      lcd_print(lcd_string);
      /* Опрашиваем 1 раз в сек */
      _delay_ms(1000);

    /* Режим конфигурации  */
    } else {
      sprintf(lcd_string, "CONF: %.1f cm", _target_distance);
      lcd_print(lcd_string);
      _delay_ms(1);
    }
  }
}
