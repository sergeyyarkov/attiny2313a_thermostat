;
; Project name: thermostat
; Description: Electronic thermostat on AVR Microcontroller
; Source code: https://github.com/sergeyyarkov/attiny2313a_thermostat
; Device: ATtiny2313A
; Device Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf
; Package: SOIC-20W_7.5x12.8mm_P1.27mm
; Assembler: AVR macro assembler 2.2.7
; Clock frequency: 8 MHz External Crystal Oscillator
; Fuses: lfuse: 0xCF, hfuse: 0x9F, efuse: 0xFF, lock: 0xFF
;
; Written by Sergey Yarkov 22.01.2023

.INCLUDE "tn2313Adef.inc"
.LIST

.DEF TEMP_REG_A       = r16
.DEF TEMP_REG_B       = r17

.EQU DIGIT_1_PIN              = PD2
.EQU DIGIT_2_PIN              = PD3
.EQU DIGIT_3_PIN              = PD4
.EQU DIGIT_4_PIN              = PD5

.EQU LED_POWER_PIN            = PD6

.EQU USI_LATCH_PIN            = PB0         ; ST_CP on 74HC595
.EQU USI_DO_PIN               = PB6         ; DS on 74HC595
.EQU USI_CLK_PIN              = PB7         ; SH_CP on 74HC595

.EQU MCU_STATE_DEFAULT        = 0x01        ; Режим измерения температуры и сравнения с заданными параметрами
.EQU MCU_STATE_PROGRAM        = 0x02        ; Режим настройки параметров в EEPROM
.EQU MCU_STATE_ERROR          = 0x03        ; Режим ошибки, например когда не подключен датчик

; **** СЕГМЕНТ ДАННЫХ ********************************************
.DSEG
.ORG SRAM_START

MCU_STATE:      .BYTE 1                     ; Текущее состояние МК
NUMBER:         .BYTE 2
DIGITS:         .BYTE 4                     ; Ячейки, где хранятся символы, для вывода на индикатор
CURRENT_DIGIT:  .BYTE 1                     ; Номер разряда индикатора, который сейчас горит

; **** СЕГМЕНТ КОДА **********************************************
.CSEG

.ORG 0x00     
  rjmp 	RESET_vect

.ORG 0x000D   
  rjmp  TIMER0_COMPA_vect

; **** ДИНАМИЧЕСКАЯ ИНДИКАЦИЯ ************************************
TIMER0_COMPA_vect:
  lds       r20,  CURRENT_DIGIT
  cpi       r20,  5
  brge      reset_digit_idx
  rjmp      _indicate_1

  reset_digit_idx:
    clr     r20
    sts     CURRENT_DIGIT, r20

    _indicate_1:
    cpi       r20, 0
    brne      _indicate_2

    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_3_PIN
    cbi       PORTD, DIGIT_4_PIN
    sbi       PORTD, DIGIT_1_PIN
    lds       TEMP_REG_A, DIGITS+3
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

    _indicate_2:
    cpi       r20, 1
    brne      _indicate_3

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_3_PIN
    cbi       PORTD, DIGIT_4_PIN
    sbi       PORTD, DIGIT_2_PIN
    lds       TEMP_REG_A, DIGITS+2
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

    _indicate_3:
    cpi       r20, 2
    brne      _indicate_4

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_4_PIN
    sbi       PORTD, DIGIT_3_PIN
    lds       TEMP_REG_A, DIGITS+1
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

    _indicate_4:
    cpi       r20, 3
    brne      _indicate_exit

    cbi       PORTD, DIGIT_1_PIN
    cbi       PORTD, DIGIT_2_PIN
    cbi       PORTD, DIGIT_3_PIN
    sbi       PORTD, DIGIT_4_PIN
    lds       TEMP_REG_A, DIGITS
    rcall     DISPLAY_DECODER
    rcall     USI_TRANSMIT

    _indicate_exit:
    inc       r20
    sts       CURRENT_DIGIT, r20
reti

RESET_vect:
  ldi       TEMP_REG_A, LOW(RAMEND)
  out       SPL, TEMP_REG_A

MCU_INIT:
  ; **** ИНИЦИАЛИЗАЦИЯ ПОРТОВ ************************************
  ldi       r16, (1<<LED_POWER_PIN) | (1<<DIGIT_1_PIN) | (1<<DIGIT_2_PIN) | (1<<DIGIT_3_PIN) | (1<<DIGIT_4_PIN)
  out       DDRD, r16
  ldi       r16, (1<<USI_CLK_PIN) | (1<<USI_DO_PIN) | (1<<USI_LATCH_PIN) | (1<<PB2)
  out       DDRB, r16
  
  ; **** ИНИЦИАЛИЗАЦИЯ ТАЙМЕРОВ **********************************
  ldi       r16, (1<<COM0A0) | (1<<WGM01)
  out       TCCR0A, r16

  ldi       r16,  (1<<CS02) | (1<<CS00)
  out       TCCR0B, r16

  ldi       r16, 40
  out       OCR0A, r16

  ldi       r16, (1<<OCIE0A)
  out       TIMSK, r16

  clr       r1
  ldi	      ZL,LOW(2*DISPLAY_SYMBOLS)
	ldi	      ZH,HIGH(2*DISPLAY_SYMBOLS)

  ; загружаем число, которое нужно показать на индикатор
  .EQU DISPLAY_NUMBER = 279
  ldi   r22,    LOW(DISPLAY_NUMBER)
  ldi   r23,    HIGH(DISPLAY_NUMBER)

  sts   CURRENT_DIGIT, r1

  sei

  rjmp      LOOP

; **** ПОДПРОГРАММЫ **********************************************
.INCLUDE "div16u.asm"
.INCLUDE "usi.asm"

; **** ГЛАВНЫЙ ЦИКЛ **********************************************
LOOP:  
  ; **** ПОЛУЧЕНИЕ ЦИФР ИЗ 16-ТИ БИТНОГО ЧИСЛА *******************
  ; Описание: Перемещает цифры числа в соответствующие ячейки памяти в SRAM
  ;           путем деления этого числа несколько раз
  GET_DIGITS:
    cli
    ldi   r24,    4                     ; максимум 4 цифры, т.к индикатор четырех разрядный
    ldi   XL, LOW(DIGITS)
    ldi   XH, HIGH(DIGITS)

    .equ  dividend_addr = $0060
    .equ  divisor       = 10

    ; загружаем делимое в адрес SRAM делимого
    sts   dividend_addr,     r22
    sts   dividend_addr+1,   r23

    ; четыре раза производим деление для получения остатков
    DIV_LOOP:
      ; заполняем нужные регистры
      lds   dd16uL, dividend_addr
      lds   dd16uH, dividend_addr+1
      ldi   dv16uL, LOW(divisor)
      ldi   dv16uH, HIGH(divisor)
      
      rcall div16u                      ; делим

      st   X+,    drem16uL              ; сохраняем остаток в ячейку по указателю

      ; обновляем делимое
      sts  dividend_addr, dres16uL
      sts  dividend_addr+1, dres16uH

      dec   r24                         ; декрементируем счетчик цикла
      brne  DIV_LOOP                    ; делим еще раз если не 0
      sei

  rcall       TOGGLE_POWER_LED
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY
  rcall       DELAY

  rjmp      LOOP

DISPLAY_DECODER:
  ldi	      ZL, LOW(2*DISPLAY_SYMBOLS)
	ldi	      ZH, HIGH(2*DISPLAY_SYMBOLS)

  ldi       TEMP_REG_B, 0
  add       ZL, TEMP_REG_A
  adc       ZH, TEMP_REG_B

  lpm
ret

DELAY:
  push      r16
  push      r17
  ldi       r16, 255
  _DELAY_1:
    ldi     r17, 255   
  _DELAY_2:
    dec     r17         
    nop                 
    nop                
    nop                 
    brne    _DELAY_2    

    dec     r16
    brne    _DELAY_1    

  pop       r17
  pop       r16
ret                    

TOGGLE_POWER_LED:
  push      r16
  push      r17
  ldi       r16,   (1<<LED_POWER_PIN)
  in        r17,   PORTD
  eor       r17,   r16
  out       PORTD, r17
  pop       r17
  pop       r16
ret

DISPLAY_SYMBOLS:
      ; HGFEDCBA    HGFEDCBA
  .DB 0b11000000, 0b11111001          ; 0, 1
  .DB 0b10100100, 0b10110000          ; 2, 3
  .DB 0b10011001, 0b10010010          ; 4, 5
  .DB 0b10000010, 0b11111000          ; 6, 7
  .DB 0b10000000, 0b10010000          ; 8, 9

; **** СЕГМЕНТ EEPROM ********************************************
; .ESEG
; INFO:       .DB "AVR Thermostat. Written by Sergey Yarkov 22.01.2023"