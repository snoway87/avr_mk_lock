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
#include <avr/eeprom.h>

#define DS1990_KEY_LEN 8

//
// 1-Wire
// -----------------------------------------------------
#define ONEWIRE_PORT   PORTB
#define ONEWIRE_DDR    DDRB
#define ONEWIRE_PIN    PINB
#define ONEWIRE_NUM    (1 << PB0)     /* Этот пин должен быть подтянут к линии питания!!! */


void     onew_low() { ONEWIRE_DDR |= ONEWIRE_NUM; }
void     onew_high() { ONEWIRE_DDR &= ~ONEWIRE_NUM; }
uint8_t  onew_read() { return ONEWIRE_PIN & ONEWIRE_NUM; }

uint8_t onew_reset() {
  // Запускаем процедуру опроса ключей
  onew_low();
  _delay_us(640);
  onew_high();
  _delay_us(2);

  // Ждем 80 мксек импульса от ключа
  for(int i = 0; i < 80; i ++ ) {
    if(!onew_read()) {
      // Ждем окончания импульса от ключа
      while(!onew_read()) {};
      return 1;
    }
    _delay_us(1);
  }
  return 0;
}

void onew_send(uint8_t b) {
  // Передаем один байт информации на ключ
  for(int i = 0; i < 8; i ++ ) {
    onew_low();
    if(b & (1 << i)) {
      _delay_us(5);
      onew_high();
      _delay_us(90);
    }else {
      _delay_us(90);
      onew_high();
      _delay_us(5);
    }
  }
}

uint8_t onew_recv( void ) {
  // Читаем байт информации от ключа
  uint8_t res = 0;
  for( int i = 0; i < 8; i ++ ) {
    res >>= 1;
    onew_low();
    _delay_us(2);
    onew_high();
    _delay_us(8);
    if(onew_read())
      res |= 0x80;
    _delay_us(80);
  }
  return res;
}

uint8_t ds1990_get_key(uint8_t * key) {
  // Читаем номер ключа
  if(!onew_reset())
    return 0;
  onew_send(0x33);
  for(int i = 0; i < DS1990_KEY_LEN; i ++ ) {
    key[i] = onew_recv();
  }
  return 1;
}

//
// GLCD 128x64
// -----------------------------------------------------
#define GLCD_EXEC_RS      (1 << PA2)//(1 << PC0)
#define GLCD_EXEC_RW      (1 << PA3)//(1 << PC1)
#define GLCD_EXEC_EN      (1 << PD6) //(1 << PC2)
#define GLCD_EXEC_CS1     (1 << PB0) //(1 << PC3)
#define GLCD_EXEC_CS2     (1 << PB1) //(1 << PC4)
#define GLCD_EXEC_RST     (1 << PD7) //(1 << PC5)

#define GLCD_DATA_PORT  PORTC
#define GLCD_DATA_DDR   DDRC
#define GLCD_EXEC_PORT  PORTC
#define GLCD_EXEC_DDR   DDRC

#define GLCD_DISPLAY_OFF    0x3e
#define GLCD_DISPLAY_ON     0x3f

#define GLCD_X_ADDR         0xb8
#define GLCD_Y_ADDR         0x40
#define GLCD_Z_ADDR         0xc0

#define GLCD_X_TOTAL        8
#define GLCD_Y_TOTAL        64

#define GLCD_FONT_WIDTH     5
#define GLCD_MAX_LENGTH     8

char font[][5] = {
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20 (Space)
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c �
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ->
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f <-
};


void glcd_exec(uint8_t value, uint8_t is_command) {
  GLCD_DATA_PORT = value;             /* Передаем байт на GLCD */
  if(is_command) {
    DDRA &= ~GLCD_EXEC_RS;  /* Передаем команду */
  } else {
    DDRA |= GLCD_EXEC_RS;   /* Передаем данные */
  }
  DDRA &= ~GLCD_EXEC_RW;    /* Выбрать операцию записи */
  DDRD |= GLCD_EXEC_EN;     /* Начать передау */
  _delay_us(10);
  DDRD &= ~GLCD_EXEC_EN;    /* Закончить передачу */
  _delay_us(10);
}

void glcd_clear( void ) {
  // Параллельно чистим обе части экрана
  DDRB |= GLCD_EXEC_CS1 | GLCD_EXEC_CS2;
  for(int i = 0; i < GLCD_X_TOTAL; i ++) {
    glcd_exec(GLCD_X_ADDR + i, 1);
    for(int j = 0; j < GLCD_Y_TOTAL; j ++) {
      glcd_exec(0, 0);
    }
  }
  // Устанавливаем курсор
  glcd_exec(GLCD_Y_ADDR, 1);
  glcd_exec(GLCD_X_ADDR, 1);
}

void glcd_print(uint8_t x_offset, char * str) {
  for(int i = 0; str[i] != 0 || i < GLCD_MAX_LENGTH; i++ ) {
    // Первые 4 символа рисуем на левой части дисплея
    if( i == 0 ) {
      DDRB |=  GLCD_EXEC_CS1;
      DDRB &= ~GLCD_EXEC_CS2;
      glcd_exec(GLCD_X_ADDR + x_offset, 1);
    }
    // Вторые 4 символа рисуем на правой части дисплея
    if( i == 4 ) {
      DDRB &= ~GLCD_EXEC_CS1;
      DDRB |=  GLCD_EXEC_CS2;
      glcd_exec(GLCD_X_ADDR + x_offset, 1);
    }

    // Выводим один символ
    for(int j = 0; j < GLCD_FONT_WIDTH; j ++) {
      glcd_exec(font[str[i] - 32][j], 0);
    }
    // После каждого символа (не пробела!) рисуем отступ
    if((str[i] - 32) != 0) {
      glcd_exec(0, 0);
    }
  }
  glcd_exec(GLCD_Y_ADDR, 1);
}

void glcd_init( void ) {
  GLCD_DATA_DDR = 0xff;                 /* Настраиваем порт на выход OUTs */

  DDRA |= GLCD_EXEC_RS  | GLCD_EXEC_RW;
  DDRD |= GLCD_EXEC_EN  | GLCD_EXEC_RST;
  DDRB |= GLCD_EXEC_CS1 | GLCD_EXEC_CS2;

  PORTD |= GLCD_EXEC_RST;                   /* Снимаем ресет */
  PORTB |= GLCD_EXEC_CS1 | GLCD_EXEC_CS2;   /* Используем левую и правую части дисплея */

  //GLCD_EXEC_DDR = 0xff;               /* Настраиваем порт на выход OUTs */
  //GLCD_EXEC_PORT = GLCD_EXEC_CS1 |      /* Используем левую часть дисплея */
  //                 GLCD_EXEC_CS2 |      /* Используем правую часть дисплея */
  //                 GLCD_EXEC_RST ;      /* Снимаем ресет */

  _delay_ms(20);
  glcd_exec(GLCD_DISPLAY_OFF, 1);       /* Выключаем дисплей */
  // Устанавливаем курсор
  glcd_exec(GLCD_Y_ADDR, 1);
  glcd_exec(GLCD_X_ADDR, 1);
  glcd_exec(GLCD_Z_ADDR, 1);
  glcd_exec(GLCD_DISPLAY_ON, 1);        /* Включаем дисплей */
}


//
// EEPROM
// -----------------------------------------------------
#define EEPROM_ADDR 0x10

void eeprom_save_key(uint8_t * key) {
  for(int i = 0; i < DS1990_KEY_LEN; i ++ ) {
    eeprom_write_byte((uint8_t *) EEPROM_ADDR + i, key[i]);
  }
}

uint8_t eeprom_compare_key(uint8_t * key) {
  volatile uint8_t value;
  for(int i = 0; i < DS1990_KEY_LEN; i ++ ) {
     value = eeprom_read_byte((uint8_t *) EEPROM_ADDR + i);
    if(value != key[i])
      return 0;
  }
  return 1;
}

//
// Entry point
// -----------------------------------------------------
int main( void ) {
  uint8_t ds1990_own_key[DS1990_KEY_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t ds1990_rem_key[DS1990_KEY_LEN];

  glcd_init();
  eeprom_save_key(ds1990_own_key);
  while( 1 ) {

    // Если есть ключ на линии
    if(ds1990_get_key(ds1990_rem_key))
    // Если номера ключей совпали, открываем замок
    if(eeprom_compare_key(ds1990_rem_key)) {
      glcd_print(0,"LOCK IS");
      glcd_print(1,"OPENED");
      // Ждем 10 сек и стираем дисплей
      _delay_ms(10000);
      glcd_clear();
    }

    // Каждую секунду
    _delay_ms(1000);
  }
}
